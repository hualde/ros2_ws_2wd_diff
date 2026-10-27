"""Filtro de seguridad: frena el avance si la OAK-D ve un obstáculo delante.

Flujo:
    teleop (teclado/mando) --/cmd_vel_teleop--> obstacle_stop_node --/cmd_vel--> motor_driver_node
                                                     ^
                                  /oak/spatial_grid (oak_spatial_grid_node)

- Solo limita el AVANCE (linear.x > 0). Girar y retroceder siempre están permitidos,
  para poder salir de una situación bloqueada.
- Entre stop_distance y slow_distance reduce la velocidad de forma proporcional.
- Si deja de recibir datos de la cámara (stale_timeout), bloquea el avance (fail-safe).
"""

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')

        self.declare_parameter('stop_distance', 0.40)    # m: por debajo, no avanza
        self.declare_parameter('slow_distance', 0.70)    # m: por debajo, reduce velocidad
        self.declare_parameter('grid_cols', 3)
        # Filas de la rejilla 3x3 que se tienen en cuenta (0 = arriba, 1 = centro, 2 = abajo).
        # El suelo ya lo filtra oak_spatial_grid_node (filtro de altura), así que se vigilan todas.
        self.declare_parameter('rows_to_check', [0, 1, 2])
        self.declare_parameter('stale_timeout', 0.5)     # s sin datos de cámara -> bloquear
        self.declare_parameter('cmd_timeout', 0.5)       # s sin órdenes -> dejar de publicar

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.cols = int(self.get_parameter('grid_cols').value)
        self.rows_to_check = list(self.get_parameter('rows_to_check').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.min_distance = None          # distancia mínima válida en las celdas vigiladas
        self.last_grid_time = None
        self.last_cmd = None
        self.last_cmd_time = None
        self.was_blocked = False

        self.create_subscription(PoseArray, '/oak/spatial_grid', self.grid_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blocked_pub = self.create_publisher(Bool, '/obstacle_stop/blocked', 10)
        self.distance_pub = self.create_publisher(Float32, '/obstacle_stop/min_distance', 10)

        # Republica la última orden filtrada a 10 Hz, para que el freno actúe
        # aunque el obstáculo aparezca mientras se mantiene una orden.
        self.create_timer(0.1, self.publish_filtered)

        self.get_logger().info(
            f'obstacle_stop_node iniciado (stop={self.stop_distance} m, '
            f'slow={self.slow_distance} m, filas={self.rows_to_check}). '
            f'Escucha /cmd_vel_teleop y publica /cmd_vel')

    def grid_callback(self, msg: PoseArray):
        distances = []
        for i, pose in enumerate(msg.poses):
            row = i // self.cols
            if row not in self.rows_to_check:
                continue
            d = pose.position.x
            if d > 0.0:               # 0 = la cámara no tiene profundidad válida en esa celda
                distances.append(d)

        self.min_distance = min(distances) if distances else None
        self.last_grid_time = self.get_clock().now()

        self.distance_pub.publish(
            Float32(data=float(self.min_distance) if self.min_distance is not None else -1.0))

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.publish_filtered()

    def forward_scale(self):
        """Factor (0..1) que se aplica a la velocidad de avance."""
        now = self.get_clock().now()
        if self.last_grid_time is None or \
                (now - self.last_grid_time).nanoseconds / 1e9 > self.stale_timeout:
            return 0.0, 'sin datos de la cámara'

        if self.min_distance is None:
            return 1.0, None                   # nada válido delante: vía libre
        if self.min_distance <= self.stop_distance:
            return 0.0, f'obstáculo a {self.min_distance:.2f} m'
        if self.min_distance < self.slow_distance:
            scale = (self.min_distance - self.stop_distance) / \
                (self.slow_distance - self.stop_distance)
            return scale, None
        return 1.0, None

    def publish_filtered(self):
        if self.last_cmd is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            return   # sin órdenes recientes: el watchdog de motor_driver para los motores

        out = Twist()
        out.linear.x = self.last_cmd.linear.x
        out.angular.z = self.last_cmd.angular.z

        scale, reason = self.forward_scale()
        blocked = False
        if out.linear.x > 0.0:
            out.linear.x *= scale
            blocked = scale == 0.0

        if blocked and not self.was_blocked:
            self.get_logger().warn(f'Avance bloqueado: {reason}')
        elif not blocked and self.was_blocked:
            self.get_logger().info('Vía libre')
        self.was_blocked = blocked

        self.blocked_pub.publish(Bool(data=blocked))
        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = ObstacleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
