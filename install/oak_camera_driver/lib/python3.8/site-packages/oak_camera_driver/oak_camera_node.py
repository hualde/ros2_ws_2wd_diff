import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai


def cv2_to_imgmsg(cv_image, encoding, frame_id, stamp):
    """Construye un sensor_msgs/Image manualmente, sin depender de cv_bridge."""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(cv_image.strides[0])
    msg.data = cv_image.tobytes()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')

        self.declare_parameter('fps', 30.0)
        fps = self.get_parameter('fps').value

        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)

        self.pipeline = self.build_pipeline()
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)

        self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info('oak_camera_node iniciado, publicando en /oak/rgb/image_raw y /oak/depth/image_raw')

    def build_pipeline(self):
        pipeline = dai.Pipeline()

        # --- Cámara RGB únicamente, en baja resolución ---
        # (el pipeline de estéreo/profundidad se ha quitado temporalmente:
        # con USB2 no hay ancho de banda para RGB + 2 cámaras mono + profundidad a la vez.
        # Cuando se resuelva el tema de USB3, se puede volver a añadir el bloque estéreo.)
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(320, 240)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(15)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is not None:
            img_msg = cv2_to_imgmsg(
                rgb_frame.getCvFrame(), 'bgr8', 'oak_rgb_camera_optical_frame', now)
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