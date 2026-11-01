"""Publica la imagen de una camara en ROS 2 usando GStreamer.

Fuentes (parametro 'source'):
  csi  camara CSI de cable plano con driver en la Jetson (p. ej. RPi v2 IMX219),
       a traves de nvarguscamerasrc (ISP de la Jetson).
  usb  webcam UVC en 'device' (por defecto /dev/video0).

Publica:
  /camera/image_raw             sensor_msgs/Image (bgr8)
  /camera/image_raw/compressed  sensor_msgs/CompressedImage (jpeg), para verla
                                en el PC por WiFi sin saturar la red.

Solo se convierte/codifica lo que tiene suscriptores.
Con 'pipeline' se puede pasar un pipeline completo que acabe en
"appsink name=sink" entregando video/x-raw,format=BGR.
"""
import array

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402

try:
    import cv2
except ImportError:  # sin OpenCV no hay JPEG, pero la imagen raw sigue saliendo
    cv2 = None


class RpiCameraNode(Node):
    def __init__(self):
        super().__init__('rpi_camera_node')

        self.declare_parameter('source', 'csi')          # csi | usb
        self.declare_parameter('sensor_id', 0)           # csi
        self.declare_parameter('device', '/dev/video0')  # usb
        # Modo de captura del sensor (tiene que existir en la camara).
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('fps', 15)
        # Tamano publicado.
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('upside_down', False)     # gira la imagen 180 grados
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('pipeline', '')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.jpeg_params = (
            [int(cv2.IMWRITE_JPEG_QUALITY), int(p('jpeg_quality'))] if cv2 else None)

        pipeline = p('pipeline') or self.default_pipeline(p)
        self.get_logger().info(f'Pipeline: {pipeline}')

        self.raw_pub = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data)
        self.jpeg_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)

        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline)
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_sample)
        self.bus = self.pipeline.get_bus()
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.report_bus_errors()
            raise RuntimeError('No se pudo arrancar la camara (mira los errores de arriba).')

        # Los errores de GStreamer (camara desconectada, etc.) se leen del bus.
        self.create_timer(0.5, self.report_bus_errors)
        self.get_logger().info('rpi_camera_node iniciado, publicando en /camera/image_raw')

    @staticmethod
    def default_pipeline(p):
        cw, ch, fps = int(p('capture_width')), int(p('capture_height')), int(p('fps'))
        w, h = int(p('width')), int(p('height'))
        flip = 2 if p('upside_down') else 0
        if p('source') == 'usb':
            src = (f'v4l2src device={p("device")} ! '
                   f'video/x-raw,width={cw},height={ch},framerate={fps}/1 ! '
                   f'videoflip method={flip} ! ')
        else:
            src = (f'nvarguscamerasrc sensor-id={int(p("sensor_id"))} ! '
                   f'video/x-raw(memory:NVMM),width={cw},height={ch},framerate={fps}/1 ! '
                   f'nvvidconv flip-method={flip} ! video/x-raw,format=BGRx ! ')
        return (src + f'videoscale ! videoconvert ! '
                f'video/x-raw,width={w},height={h},format=BGR ! '
                'appsink name=sink emit-signals=true drop=true max-buffers=1 sync=false')

    def on_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        want_raw = self.raw_pub.get_subscription_count() > 0
        want_jpeg = self.jpeg_params and self.jpeg_pub.get_subscription_count() > 0
        if not (want_raw or want_jpeg):
            return Gst.FlowReturn.OK

        s = sample.get_caps().get_structure(0)
        width, height = s.get_value('width'), s.get_value('height')
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            data = bytes(info.data)
        finally:
            buf.unmap(info)
        stamp = self.get_clock().now().to_msg()

        if want_raw:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height, msg.width = height, width
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = len(data) // height
            # array.array evita que Foxy valide byte a byte en Python (muy lento)
            msg.data = array.array('B', data)
            self.raw_pub.publish(msg)

        if want_jpeg:
            step = len(data) // height
            frame = np.frombuffer(data, np.uint8).reshape(height, step)[:, :width * 3]
            frame = frame.reshape(height, width, 3)
            ok, jpeg = cv2.imencode('.jpg', frame, self.jpeg_params)
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.format = 'jpeg'
                msg.data = array.array('B', jpeg.tobytes())
                self.jpeg_pub.publish(msg)
        return Gst.FlowReturn.OK

    def report_bus_errors(self):
        while True:
            m = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING)
            if m is None:
                return
            if m.type == Gst.MessageType.ERROR:
                err, dbg = m.parse_error()
                self.get_logger().error(f'GStreamer: {err.message} ({dbg})')
            else:
                err, dbg = m.parse_warning()
                self.get_logger().warn(f'GStreamer: {err.message}')

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main():
    rclpy.init()
    node = RpiCameraNode()
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
