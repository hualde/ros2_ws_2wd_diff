#!/usr/bin/env python3
"""Publica el MPU6050 (I2C) como sensor_msgs/Imu en /imu/data_raw.

Sin orientacion (covarianza[0] = -1): solo velocidad angular y aceleracion lineal.
Al arrancar calibra el sesgo del giroscopio con el robot QUIETO.
"""
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus

G = 9.80665
DEG2RAD = 0.017453292519943295
PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CFG, ACCEL_CFG, ACCEL_XOUT_H = 0x6B, 0x19, 0x1A, 0x1B, 0x1C, 0x3B


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.declare_parameter('bus', 1)
        self.declare_parameter('address', 0x68)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibrate_gyro', True)
        self.declare_parameter('calib_samples', 300)
        self.declare_parameter('gyro_variance', 4e-5)    # (rad/s)^2
        self.declare_parameter('accel_variance', 4e-2)   # (m/s^2)^2

        self.addr = self.get_parameter('address').value
        self.frame = self.get_parameter('frame_id').value
        self.bus = SMBus(self.get_parameter('bus').value)

        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)   # reset
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)   # reloj PLL gyro X
        self.bus.write_byte_data(self.addr, CONFIG, 3)          # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 9)      # 1 kHz / 10
        self.bus.write_byte_data(self.addr, GYRO_CFG, 0x00)     # +-250 dps
        self.bus.write_byte_data(self.addr, ACCEL_CFG, 0x00)    # +-2 g
        time.sleep(0.1)

        self.bias = [0.0, 0.0, 0.0]
        if self.get_parameter('calibrate_gyro').value:
            self.calibrate()

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.errors = 0
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def read_raw(self):
        d = bytes(self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14))
        ax, ay, az, _t, gx, gy, gz = struct.unpack('>7h', d)
        return ax, ay, az, gx, gy, gz

    def calibrate(self):
        n = self.get_parameter('calib_samples').value
        self.get_logger().info(f'Calibrando giroscopio ({n} muestras): no muevas el robot...')
        s = [0.0, 0.0, 0.0]
        for _ in range(n):
            r = self.read_raw()
            for i in range(3):
                s[i] += r[3 + i] / 131.0 * DEG2RAD
            time.sleep(0.005)
        self.bias = [v / n for v in s]
        self.get_logger().info('Sesgo gyro (rad/s): %.4f %.4f %.4f' % tuple(self.bias))

    def tick(self):
        try:
            ax, ay, az, gx, gy, gz = self.read_raw()
        except OSError as e:
            self.errors += 1
            if self.errors % 50 == 1:
                self.get_logger().warn(f'Error I2C ({self.errors}): {e}')
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.linear_acceleration.x = ax / 16384.0 * G
        m.linear_acceleration.y = ay / 16384.0 * G
        m.linear_acceleration.z = az / 16384.0 * G
        m.angular_velocity.x = gx / 131.0 * DEG2RAD - self.bias[0]
        m.angular_velocity.y = gy / 131.0 * DEG2RAD - self.bias[1]
        m.angular_velocity.z = gz / 131.0 * DEG2RAD - self.bias[2]
        m.orientation_covariance[0] = -1.0
        gv = self.get_parameter('gyro_variance').value
        av = self.get_parameter('accel_variance').value
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = gv
            m.linear_acceleration_covariance[i] = av
        self.pub.publish(m)


def main():
    rclpy.init()
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
