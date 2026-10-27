import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import depthai as dai


def frame_to_imgmsg(frame, encoding, frame_id, stamp):
    """Construye un sensor_msgs/Image manualmente, sin depender de cv_bridge."""
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(frame.strides[0])
    msg.data = frame.tobytes()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')

        self.declare_parameter('fps', 15)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)

        # QoS de sensor (best effort, cola pequeña): mucho mas barato que
        # el reliable por defecto al publicar imagenes en la Jetson Nano.
        self.rgb_pub = self.create_publisher(
            Image, '/oak/rgb/image_raw', qos_profile_sensor_data)

        self.pipeline = self.build_pipeline()
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(
            name='rgb', maxSize=2, blocking=False)

        # El timer corre al doble de la frecuencia de la camara para no
        # perder frames por desfases entre el reloj del timer y el de la camara.
        self.create_timer(0.5 / self.fps, self.timer_callback)
        self.get_logger().info(
            f'oak_camera_node iniciado ({self.width}x{self.height} @ {self.fps} fps), '
            f'publicando en /oak/rgb/image_raw (QoS sensor_data / best effort)')

    def build_pipeline(self):
        pipeline = dai.Pipeline()

        # --- Camara RGB unicamente ---
        # La camara ya va por USB 3, asi que cuando haga falta se puede volver
        # a anadir aqui el bloque estereo / profundidad.
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.width, self.height)
        # Interleaved=True: la camara entrega HxWx3 listo para usar,
        # sin que el host tenga que convertir de planar a interleaved.
        cam_rgb.setInterleaved(True)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.fps)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def timer_callback(self):
        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is None:
            return

        now = self.get_clock().now().to_msg()
        img_msg = frame_to_imgmsg(
            rgb_frame.getFrame(), 'bgr8', 'oak_rgb_camera_optical_frame', now)
        self.rgb_pub.publish(img_msg)

    def destroy_node(self):
        self.device.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()