"""Rejilla de distancias a partir de la profundidad de la OAK-D.

Recibe la imagen de profundidad completa en la Jetson y la divide en una
rejilla (3x3 por defecto). Para cada celda calcula un PERCENTIL BAJO de la
profundidad (por defecto el 10 %): es casi tan sensible como el mínimo para
detectar un objeto que ocupa parte de la celda, pero ignora los píxeles de
ruido sueltos que hacían que el mínimo diera siempre ~0,36 m.

Publica /oak/spatial_grid (PoseArray) en el frame oak_camera_link:
  x = distancia hacia delante, y = lateral (izquierda +), z = vertical (arriba +).
  Las poses se ordenan fila a fila, de arriba abajo y de izquierda a derecha.
  Una celda sin datos fiables se publica con x = y = z = 0.
"""

import math

import depthai as dai
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

# Campo de visión de las cámaras estéreo (OV7251) de la OAK-D Lite
HFOV_DEG = 73.0
VFOV_DEG = 58.0


class OakSpatialGridNode(Node):
    def __init__(self):
        super().__init__('oak_spatial_grid_node')

        self.declare_parameter('grid_rows', 3)
        self.declare_parameter('grid_cols', 3)
        self.declare_parameter('percentile', 10.0)          # % más cercano de cada celda
        self.declare_parameter('min_valid_fraction', 0.05)  # mínimo de píxeles válidos por celda
        self.declare_parameter('min_depth', 0.38)           # m: por debajo, la cámara no mide bien
        self.declare_parameter('max_depth', 6.0)            # m
        self.declare_parameter('fps', 10)
        self.declare_parameter('upside_down', True)         # cámara montada boca abajo
        self.declare_parameter('frame_id', 'oak_camera_link')
        # Filtro de suelo: se descartan los puntos que están a menos de floor_margin
        # del suelo (el suelo no es un obstáculo) y los que están por encima de
        # max_height (una mesa por encima del robot no lo bloquea).
        self.declare_parameter('camera_height', 0.07)   # m: altura del centro de la cámara
        self.declare_parameter('camera_tilt_deg', 0.0)  # grados, + = mirando hacia abajo
        self.declare_parameter('floor_margin', 0.03)    # m
        self.declare_parameter('max_height', 0.30)      # m sobre el suelo

        self.rows = int(self.get_parameter('grid_rows').value)
        self.cols = int(self.get_parameter('grid_cols').value)
        self.percentile = float(self.get_parameter('percentile').value)
        self.min_valid_fraction = float(self.get_parameter('min_valid_fraction').value)
        self.min_depth_mm = float(self.get_parameter('min_depth').value) * 1000.0
        self.max_depth_mm = float(self.get_parameter('max_depth').value) * 1000.0
        self.fps = int(self.get_parameter('fps').value)
        self.upside_down = bool(self.get_parameter('upside_down').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_tilt = math.radians(float(self.get_parameter('camera_tilt_deg').value))
        self.floor_margin = float(self.get_parameter('floor_margin').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.row_tan = None   # tangente del ángulo vertical de cada fila (se calcula 1 vez)

        self.pub = self.create_publisher(PoseArray, '/oak/spatial_grid', 10)

        self.device = dai.Device(self.build_pipeline())
        self.queue = self.device.getOutputQueue(name='depth', maxSize=2, blocking=False)

        self.create_timer(0.5 / self.fps, self.timer_callback)
        self.get_logger().info(
            f'oak_spatial_grid_node iniciado ({self.rows}x{self.cols}, percentil '
            f'{self.percentile:.0f}, rango {self.min_depth_mm / 1000:.2f}-'
            f'{self.max_depth_mm / 1000:.1f} m, upside_down={self.upside_down}, '
            f'cámara a {self.camera_height:.2f} m, alturas válidas '
            f'{self.floor_margin:.2f}-{self.max_height:.2f} m), '
            f'publicando en /oak/spatial_grid')

    def build_pipeline(self):
        pipeline = dai.Pipeline()

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        # Los sensores OV7251 de la OAK-D Lite solo admiten 480P/400P
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_left.setFps(self.fps)
        mono_right.setFps(self.fps)

        # Misma configuración que depth_test.py noext, que mide bien.
        # (Con extended disparity la profundidad salía con mucho ruido.)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName('depth')
        stereo.depth.link(xout.input)
        return pipeline

    def timer_callback(self):
        msg_in = self.queue.tryGet()
        if msg_in is None:
            return

        depth = msg_in.getFrame()                  # uint16, milímetros, 0 = sin dato
        if self.upside_down:
            depth = depth[::-1, ::-1]              # girar 180°: arriba/izquierda reales
        h, w = depth.shape

        # Altura sobre el suelo de cada píxel. Para la fila i, el rayo forma un ángulo
        # hacia abajo phi = (i/h - 0.5) * VFOV + tilt; un punto a distancia d (hacia
        # delante) queda a d * tan(phi) por debajo de la cámara.
        if self.row_tan is None or self.row_tan.shape[0] != h:
            rows_idx = (np.arange(h) + 0.5) / h - 0.5
            phi = rows_idx * math.radians(VFOV_DEG) + self.camera_tilt
            self.row_tan = np.tan(phi).astype(np.float32)[:, None]
        d_m = depth.astype(np.float32) / 1000.0
        height = self.camera_height - d_m * self.row_tan
        usable = (depth >= self.min_depth_mm) & (depth <= self.max_depth_mm) & \
            (height > self.floor_margin) & (height < self.max_height)

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        for r in range(self.rows):
            for c in range(self.cols):
                y0, y1 = r * h // self.rows, (r + 1) * h // self.rows
                x0, x1 = c * w // self.cols, (c + 1) * w // self.cols
                cell = depth[y0:y1, x0:x1]
                valid = cell[usable[y0:y1, x0:x1]]

                pose = Pose()
                pose.orientation.w = 1.0
                # Basta con pocos píxeles: tras el filtro de suelo, una celda puede
                # quedar casi vacía y aun así contener un obstáculo pequeño.
                if valid.size >= self.min_valid_fraction * cell.size:
                    d = float(np.percentile(valid, self.percentile)) / 1000.0
                    # Ángulo del centro de la celda respecto al eje de la cámara
                    u = ((x0 + x1) / 2.0) / w - 0.5    # derecha +
                    v = ((y0 + y1) / 2.0) / h - 0.5    # abajo +
                    pose.position.x = d
                    pose.position.y = -d * math.tan(math.radians(u * HFOV_DEG))
                    pose.position.z = -d * math.tan(math.radians(v * VFOV_DEG))
                out.poses.append(pose)

        self.pub.publish(out)

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
