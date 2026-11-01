#!/usr/bin/env python3
"""Puente en el PC: recibe el JPEG de la Jetson y lo publica descomprimido en local.

Por la WiFi solo viaja /camera/image_raw/compressed (~0,5 MB/s). Este nodo lo
descomprime aqui y publica /camera/image_local (sensor_msgs/Image) para RViz.

Uso (dentro del contenedor ros2_foxy):
  source /opt/ros/foxy/setup.bash
  export ROS_DOMAIN_ID=42
  python3 /workspace/jpeg_relay.py
En RViz: display Image, topic /camera/image_local.
"""
import array

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class JpegRelay(Node):
    def __init__(self):
        super().__init__('jpeg_relay')
        self.pub = self.create_publisher(Image, '/camera/image_local', 1)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.cb,
            qos_profile_sensor_data)
        self.count = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info('/camera/image_raw/compressed -> /camera/image_local')

    def cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        out = Image()
        out.header = msg.header
        out.height, out.width = frame.shape[:2]
        out.encoding = 'bgr8'
        out.step = int(frame.strides[0])
        out.data = array.array('B', frame.tobytes())
        self.pub.publish(out)
        self.count += 1

    def report(self):
        self.get_logger().info(f'{self.count / 5.0:.1f} imagenes/s')
        self.count = 0


def main():
    rclpy.init()
    node = JpegRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
