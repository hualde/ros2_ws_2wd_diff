import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import depthai as dai


class OakSpatialGridNode(Node):
    def __init__(self):
        super().__init__('oak_spatial_grid_node')

        self.declare_parameter('grid_rows', 3)
        self.declare_parameter('grid_cols', 3)
        self.rows = self.get_parameter('grid_rows').value
        self.cols = self.get_parameter('grid_cols').value

        self.pub = self.create_publisher(PoseArray, '/oak/spatial_grid', 10)

        self.pipeline = self.build_pipeline()
        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(name='spatial_data', maxSize=4, blocking=False)

        self.create_timer(0.1, self.timer_callback)  # sondea a 10 Hz
        self.get_logger().info(
            f'oak_spatial_grid_node iniciado ({self.rows}x{self.cols} ROIs), '
            f'publicando en /oak/spatial_grid')

    def build_pipeline(self):
        pipeline = dai.Pipeline()

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
        spatial_calc.setWaitForConfigInput(False)

        margin = 0.02  # pequeño margen entre celdas de la rejilla
        cell_w = 1.0 / self.cols
        cell_h = 1.0 / self.rows

        # Añade una ROI por cada celda de la rejilla, fila por fila
        for row in range(self.rows):
            for col in range(self.cols):
                config = dai.SpatialLocationCalculatorConfigData()
                config.depthThresholds.lowerThreshold = 100     # 10 cm
                config.depthThresholds.upperThreshold = 10000   # 10 m

                x0 = col * cell_w + margin
                x1 = (col + 1) * cell_w - margin
                y0 = row * cell_h + margin
                y1 = (row + 1) * cell_h - margin

                top_left = dai.Point2f(x0, y0)
                bottom_right = dai.Point2f(x1, y1)
                config.roi = dai.Rect(top_left, bottom_right)
                spatial_calc.initialConfig.addROI(config)

        stereo.depth.link(spatial_calc.inputDepth)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName('spatial_data')
        spatial_calc.out.link(xout.input)

        return pipeline

    def timer_callback(self):
        data = self.queue.tryGet()
        if data is None:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'oak_camera_frame'

        # NOTA: el convenio de signos de X/Y de DepthAI puede variar según versión.
        # Verifica empíricamente (mover la mano a la derecha/arriba y comprobar
        # que el signo de x/y en RViz cambia como esperas) y ajusta si hace falta.
        for roi_data in data.getSpatialLocations():
            coords = roi_data.spatialCoordinates
            pose = Pose()
            pose.position.x = coords.z / 1000.0   # profundidad -> adelante (metros)
            pose.position.y = -coords.x / 1000.0  # lateral (metros)
            pose.position.z = -coords.y / 1000.0  # vertical (metros)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)

    def destroy_node(self):
        self.device.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = OakSpatialGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()