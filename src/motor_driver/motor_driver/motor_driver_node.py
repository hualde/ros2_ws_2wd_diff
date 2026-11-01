import math

import board
import busio
import rclpy
from adafruit_motorkit import MotorKit
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.kit = MotorKit(i2c=i2c)

        # Parámetros (ajustables sin recompilar, ver -p en ros2 run)
        self.declare_parameter('wheel_separation', 0.125)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('odom_rate_hz', 20.0)
        self.declare_parameter('min_throttle', 0.6)
        # Orientacion del robot respecto al cableado de los motores:
        #   reverse_direction=True -> el "frente" pasa a ser el lado contrario
        #   (invierte el sentido de los dos motores e intercambia izquierda/derecha).
        #   swap_motors=True -> intercambia motor1/motor2 (si gira al reves).
        self.declare_parameter('reverse_direction', True)
        self.declare_parameter('swap_motors', True)
        # Correccion de deriva al ir recto (multiplica el throttle final de cada rueda).
        # Si se desvia a la DERECHA, la rueda izquierda va mas rapida: baja left_gain.
        # Si se desvia a la IZQUIERDA, baja right_gain.
        self.declare_parameter('left_gain', 1.0)
        self.declare_parameter('right_gain', 1.0)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear = self.get_parameter('max_linear').value
        odom_rate = self.get_parameter('odom_rate_hz').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.reverse_direction = self.get_parameter('reverse_direction').value
        self.swap_motors = self.get_parameter('swap_motors').value
        self.left_gain = float(self.get_parameter('left_gain').value)
        self.right_gain = float(self.get_parameter('right_gain').value)

        # Velocidad comandada actualmente (la que llega por /cmd_vel)
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Pose estimada (lazo abierto, sin encoders)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Watchdog: si no llega cmd_vel en 0.5s, para los motores (seguridad)
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_check)

        # Timer de integración de odometría, a frecuencia fija
        self.last_odom_time = self.get_clock().now()
        self.create_timer(1.0 / odom_rate, self.update_odometry)

        self.get_logger().info(
            f'motor_driver_node iniciado (wheel_separation={self.wheel_separation}, '
            f'max_linear={self.max_linear}, min_throttle={self.min_throttle}, '
            f'reverse_direction={self.reverse_direction}, swap_motors={self.swap_motors}, '
            f'left_gain={self.left_gain}, right_gain={self.right_gain})')

    def compensate_deadband(self, throttle):
        if throttle == 0.0:
            return 0.0
        sign = 1.0 if throttle > 0 else -1.0
        magnitude = abs(throttle)
        # Reescala la magnitud de [0,1] a [min_throttle, 1], preservando proporcionalidad
        compensated = self.min_throttle + magnitude * (1.0 - self.min_throttle)
        return sign * min(compensated, 1.0)

    def set_wheels(self, left, right):
        """Envia el throttle de cada rueda (marco del robot) al motor fisico."""
        left = max(-1.0, min(1.0, left * self.left_gain))
        right = max(-1.0, min(1.0, right * self.right_gain))
        motor_a, motor_b = left, right          # motor1 = izquierda, motor2 = derecha
        if self.reverse_direction:
            # Dar la vuelta al robot: cada motor gira al reves y la rueda
            # que era la izquierda pasa a ser la derecha.
            motor_a, motor_b = -right, -left
        if self.swap_motors:
            motor_a, motor_b = motor_b, motor_a
        self.kit.motor1.throttle = motor_a
        self.kit.motor2.throttle = motor_b

    def stop_motors(self):
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0

    def cmd_vel_callback(self, msg: Twist):
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

        v_left = self.current_linear - self.current_angular * (self.wheel_separation / 2.0)
        v_right = self.current_linear + self.current_angular * (self.wheel_separation / 2.0)

        throttle_left = max(-1.0, min(1.0, v_left / self.max_linear))
        throttle_right = max(-1.0, min(1.0, v_right / self.max_linear))

        throttle_left = self.compensate_deadband(throttle_left)
        throttle_right = self.compensate_deadband(throttle_right)

        self.set_wheels(throttle_left, throttle_right)

        self.last_cmd_time = self.get_clock().now()

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.stop_motors()
            self.current_linear = 0.0
            self.current_angular = 0.0

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        # Integración simple (Euler) de la pose, usando la velocidad comandada
        delta_theta = self.current_angular * dt
        delta_x = self.current_linear * math.cos(self.theta) * dt
        delta_y = self.current_linear * math.sin(self.theta) * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Cuaternión solo con yaw (rotación en el plano)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publica el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        odom_msg.twist.twist.linear.x = self.current_linear
        odom_msg.twist.twist.angular.z = self.current_angular

        self.odom_publisher.publish(odom_msg)

        # Publica la transformada TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # La PCA9685 mantiene el último PWM: hay que parar los motores al salir
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
