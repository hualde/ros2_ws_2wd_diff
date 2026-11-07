# Robot diferencial con Jetson Nano y ROS 2 Foxy

Robot diferencial de dos ruedas controlado con una **Jetson Nano 4GB** y **ROS 2 Foxy**.
Objetivo: **SLAM y navegación autónoma** con un **lidar 2D** (RPLIDAR C1). El robot lleva además
un IMU (MPU6050) para mejorar la odometría y una cámara Raspberry Pi para ver la imagen.

> La versión anterior con cámara OAK-D Lite (rejilla de profundidad, freno por obstáculos y
> explorador autónomo) está en el historial de git (commit `770916a`).

## Arquitectura

| Máquina | Función |
|---|---|
| **Jetson Nano** (`192.168.1.34`) | Va en el robot. Ejecuta los nodos de motores, lidar, IMU, EKF, cámara, modelo y SLAM. ROS 2 Foxy nativo. |
| **PC (Omarchy)** | Estación de trabajo. Contenedor Docker `ros2_foxy` con RViz2, teleoperación y el puente de imagen. |

```
 JETSON NANO
   robot_state_publisher ── TF fijas del URDF
   motor_driver_node     ── /cmd_vel ──► motores ; publica /odom (sin TF si hay EKF)
   mpu6050_node          ── /imu/data_raw (100 Hz)
   ekf_filter_node       ── /odom + /imu/data_raw ──► /odometry/filtered + TF odom→base_link
   sllidar_node          ── /scan (RPLIDAR C1, frame laser)
   slam_toolbox          ── /scan + TF ──► /map + TF map→odom   (solo al mapear)
   rpi_camera_node       ── /camera/image_raw, /camera/image_raw/compressed
            │
            │ WiFi: solo el JPEG (~0,5 MB/s)
            ▼
 PC (contenedor ros2_foxy)
   jpeg_relay.py         ── /camera/image_raw/compressed ──► /camera/image_local
   RViz2                 ── /camera/image_local, /odom, /tf, modelo
```

Las dos máquinas se comunican por red con ROS 2 (DDS):

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_STATIC_PEERS=<IP de la otra máquina>   # en la Jetson: IP del PC; en el PC: 192.168.1.34
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Hardware

- Jetson Nano 4GB (devkit B01, dos conectores CSI)
- Placa **Adafruit DC Motor + Stepper FeatherWing** (PCA9685, I2C `0x60`)
- 2 motores DC amarillos TT, sin encoder, con ruedas de 65 mm
- Rueda loca (bola) a 9 cm por detrás del eje de las ruedas motrices
- **RPLIDAR C1** por USB (puerto `/dev/serial/by-id/usb-Silicon_Labs_CP2102N...`), frame `laser`,
  girado 180° en el URDF
- **IMU MPU6050** (6 ejes: acelerómetro + giroscopio, sin magnetómetro) en el bus **I2C 1**
  (pines 3 SDA y 5 SCL de la Jetson), dirección `0x68`. Va encima de la Jetson, en el lado
  **derecho** del robot. Ejes medidos: X delante, Y izquierda, Z arriba (igual que ROS).
  La FeatherWing de los motores va en otro bus (I2C 0, pines 27/28).
- Cámara **Raspberry Pi v1.3** (sensor OV5647) en el conector **CAM0**, montada boca abajo
- Alimentación **separada**:
  - Jetson: batería LiPo 3S 3000 mAh, vía placa reductora a 5V (jack barrel con jumper J48 puesto).
    No bajar de ~10,5 V: a 8,3 V la Jetson se reiniciaba.
  - Motores: portapilas 4×AA (6V, alcalinas) a los bornes "motor power" de la FeatherWing.
  - Los GND de todo deben estar unidos. No unir los positivos.
- Conexión I2C (lógica) Jetson ↔ FeatherWing: 3.3V, GND, SDA y SCL.

> Comprueba que la Jetson ve la placa con `sudo i2cdetect -y -r <bus>`: debe aparecer `60`.

### Orientación del robot

- **Frente = ruedas grandes.** La rueda loca va detrás.
- Convenio ROS: `linear.x > 0` avanza hacia las ruedas grandes; `angular.z > 0` gira a la
  izquierda (antihorario visto desde arriba).
- El cableado de los motores no coincide con este convenio, y se corrige por software con
  los parámetros `reverse_direction` y `swap_motors` (ver más abajo).

## Paquetes (`~/ros2_ws/src`)

| Paquete | Contenido |
|---|---|
| `motor_driver` | Nodo que mueve los motores a partir de `/cmd_vel` y publica la odometría |
| `robot_description` | Modelo URDF del robot y launch principal (`robot.launch.py`) |
| `rpi_camera` | Nodo que publica la imagen de la cámara CSI (o de una webcam USB) |
| `mpu6050_imu` | Nodo del IMU MPU6050: publica `/imu/data_raw` |
| `sllidar_ros2` | Driver del RPLIDAR (de Slamtec), se lanza con el C1 |

Fuera del workspace: `tools/pc/jpeg_relay.py`, el puente de imagen que se ejecuta en el PC.

### `motor_driver_node`

- Suscribe `/cmd_vel` (`geometry_msgs/Twist`) y calcula la velocidad de cada rueda.
- Compensa la zona muerta de los motores.
- Para los motores si no recibe `/cmd_vel` durante 0,5 s (*watchdog*) y al cerrar el nodo.
- Publica `/odom` y el TF `odom → base_link` (estimación en lazo abierto, sin encoders).

> **No hay freno por obstáculos:** `/cmd_vel` va directo a los motores. Volverá con el lidar.

Parámetros:

| Parámetro | Valor | Descripción |
|---|---|---|
| `wheel_separation` | 0.125 | Distancia entre ruedas (m) |
| `max_linear` | 0.5 | Velocidad lineal a throttle 1.0 (m/s) |
| `min_throttle` | 0.6 | Throttle mínimo para vencer la zona muerta |
| `odom_rate_hz` | 20.0 | Frecuencia de la odometría |
| `reverse_direction` | true | Invierte el frente (ruedas grandes delante) |
| `swap_motors` | true | Intercambia motor 1 y motor 2 (giro correcto) |
| `left_gain` | 1.0 | Ganancia rueda izquierda. Bajar si se desvía a la derecha |
| `right_gain` | 1.0 | Ganancia rueda derecha. Bajar si se desvía a la izquierda |

`reverse_direction` y `swap_motors` se definen en el código; el resto se puede pasar como
argumento del launch, por ejemplo `ros2 launch motor_driver motor_driver.launch.py left_gain:=0.95`.

### `rpi_camera_node`

Captura con GStreamer (`nvarguscamerasrc`, usando el ISP de la Jetson) y publica:

| Topic | Tipo | Uso |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | Para nodos que corran **en la Jetson** |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG) | Para verla **desde el PC** |

Los dos con QoS *Best Effort*. Solo se convierte o codifica lo que tiene suscriptores, y el
frame es `camera_optical_frame`.

| Parámetro (argumento del launch) | Valor | Descripción |
|---|---|---|
| `source` | `csi` | `csi` (cámara de cable plano) o `usb` (webcam UVC en `device`) |
| `capture_width` / `capture_height` | 1280 / 720 | Modo de captura del sensor |
| `fps` | 15 | Imágenes por segundo |
| `width` / `height` | 640 / 480 | Tamaño publicado |
| `upside_down` | true | Gira la imagen 180° (la cámara va boca abajo) |
| `jpeg_quality` | 70 | Calidad del JPEG |

```bash
ros2 launch rpi_camera rpi_camera.launch.py               # solo la cámara
ros2 launch rpi_camera rpi_camera.launch.py fps:=30 width:=320 height:=240
```

> **No te suscribas a `/camera/image_raw` desde el PC.** Son ~14 MB/s y Fast DDS manda una
> copia a cada IP del PC: satura la WiFi de la Jetson y se pierde todo, incluido el JPEG.

### Driver de la cámara (OV5647)

La Jetson Nano no trae driver para la cámara RPi v1.3 (solo para la v2, IMX219). Está instalado
el driver comunitario [akandr/ov5647-jetson](https://github.com/akandr/ov5647-jetson), adaptado
a la Nano B01 con un overlay propio que pone la OV5647 en los dos conectores
(CAM0 → `/dev/video0`, CAM1 → `/dev/video1`). Está en `~/ov5647-jetson` en la Jetson.

- El arranque usa `FDT /boot/kernel-ov5647-merged.dtb` (en `/boot/extlinux/extlinux.conf`).
- Comprobar: `dmesg | grep ov5647` debe decir `OV5647 detected (chip id 0x5647)`.
- Tras actualizar L4T/JetPack: `cd ~/ov5647-jetson && sudo ./install.sh` y reiniciar.
- Desinstalar: `sudo ./install.sh --uninstall` y reiniciar.
- La cámara solo se detecta al arrancar: cambia el cable con la Jetson apagada (o recarga el
  driver con `sudo rmmod ov5647 && sudo modprobe ov5647`).
- Cable plano: en la Jetson, contactos hacia el disipador; en la cámara, contactos hacia la placa.

### `mpu6050_node`

Lee el MPU6050 por I2C y publica `sensor_msgs/Imu` en `/imu/data_raw` a 100 Hz (frame `imu_link`):
velocidad angular (rad/s) y aceleración (m/s²), **sin orientación** (el chip no tiene magnetómetro).
Al arrancar **calibra el sesgo del giroscopio**: deja el robot **quieto los ~2 primeros segundos**.

Parámetros: `bus` (1), `address` (0x68), `rate_hz` (100), `frame_id` (`imu_link`),
`calibrate_gyro` (true), `calib_samples` (300).

```bash
ros2 launch mpu6050_imu mpu6050.launch.py        # solo el IMU
```

### ¿Qué es el EKF? (`robot_localization`)

EKF = *Extended Kalman Filter* (filtro de Kalman extendido). Es un algoritmo que **combina
varias medidas imperfectas de la misma cosa para obtener una estimación mejor que cualquiera
de ellas por separado**, ponderando cada una según lo fiable que sea.

Aquí el problema es que el robot **no tiene encoders**: `motor_driver` calcula la odometría
suponiendo que las ruedas hicieron lo que se les ordenó. Eso falla sobre todo al girar
(patinan, deriva, zona muerta de los motores) y un error de giro estropea todo el mapa.

El nodo `ekf_node` del paquete estándar `robot_localization` (viene con ROS, no es nuestro) fusiona:

| Fuente | Qué se usa | Por qué |
|---|---|---|
| `/odom` (motores) | Solo la velocidad hacia delante `vx` | Es lo único razonable que dan los motores |
| `/imu/data_raw` (giroscopio) | Solo la velocidad de giro `vyaw` | El giroscopio mide el giro **real**, no el ordenado |

y publica `/odometry/filtered` y el TF `odom → base_link`. Como consecuencia, **cuando el EKF
está activo, `motor_driver` ya no publica ese TF** (`publish_tf:=false`); solo debe publicarlo
uno. La configuración está en `robot_description/config/ekf.yaml`.

Límites: la posición hacia delante sigue siendo una estimación (no hay encoders), y el giro
del IMU acumula un pequeño error con el tiempo (sin magnetómetro no hay referencia absoluta).
`slam_toolbox` corrige ambas cosas comparando los escaneos del lidar. Si algo falla, se puede
desactivar con `with_ekf:=false` y volver a la odometría directa de los motores.

### `robot_description`

Contiene el modelo y los lanzamientos principales:

| Fichero | Contenido |
|---|---|
| `urdf/robot.urdf` | Cuerpo, ruedas, rueda loca, Jetson, cámara, lidar (`laser`) e IMU (`imu_link`) |
| `launch/robot.launch.py` | Lanza todo el robot (ver «Lanzamientos») |
| `launch/slam.launch.py` | Lanza `slam_toolbox` en modo mapeo |
| `config/ekf.yaml` | Parámetros del EKF |
| `config/slam_params.yaml` | Parámetros de `slam_toolbox` |

Árbol de TF: `map → odom → base_link → (laser, imu_link, camera_link, ruedas...)`.
`slam_toolbox` publica `map → odom`, el EKF `odom → base_link` y `robot_state_publisher` el resto.

Las medidas del cuerpo, cámara, lidar e IMU son aproximadas; las posiciones de la cámara
(`camera_joint`) y del IMU (`imu_joint`, solo x/y) son provisionales. Los ejes del IMU sí están medidos.

### Compilar

Siempre desde la raíz del workspace, **nunca desde `src`**:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

`build/`, `install/` y `log/` no se suben a git (`.gitignore`).

## Lanzamientos

Siempre en la Jetson, con `source ~/diag/env.sh` hecho y, si es posible, dentro de `tmux`.
Cada comando va en su propia terminal (o ventana de tmux).

### Robot completo: `robot.launch.py`

```bash
ros2 launch robot_description robot.launch.py
```

Lanza: `robot_state_publisher`, `motor_driver_node`, `rpi_camera_node`, el lidar, `mpu6050_node`
y `ekf_filter_node`. Argumentos (todos `true` por defecto):

| Argumento | Qué activa |
|---|---|
| `with_motors` | `motor_driver_node` (motores y `/odom`) |
| `with_camera` | `rpi_camera_node` |
| `with_lidar` | RPLIDAR C1 (`/scan`) |
| `with_imu` | `mpu6050_node` (`/imu/data_raw`) |
| `with_ekf` | `ekf_filter_node`. Con `false`, `motor_driver` publica él el TF `odom→base_link` |

### Casos de uso

| Quiero... | Comando |
|---|---|
| Mapear (SLAM) | Terminal 1: `ros2 launch robot_description robot.launch.py with_camera:=false`<br>Terminal 2: `ros2 launch robot_description slam.launch.py`<br>Terminal 3: teleop (ver «Mandar órdenes») |
| Solo ver el modelo en RViz | `ros2 launch robot_description robot.launch.py with_motors:=false with_camera:=false with_lidar:=false with_imu:=false with_ekf:=false` |
| Probar el IMU y el EKF sin lidar ni cámara | `ros2 launch robot_description robot.launch.py with_camera:=false with_lidar:=false` |
| Volver a la odometría directa (sin IMU) | `ros2 launch robot_description robot.launch.py with_imu:=false with_ekf:=false` |
| Solo el lidar | `ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=<by-id> frame_id:=laser` |

Para mapear, lanza la cámara solo si la necesitas: la Nano va justa de CPU.
Empieza con el robot **quieto 2-3 s** (calibración del giroscopio) y muévelo despacio.

### Comprobaciones rápidas

```bash
ros2 node list                                   # ekf_filter_node, motor_driver_node, mpu6050_node, ...
ros2 topic hz /imu/data_raw                      # ~100 Hz
ros2 topic hz /scan                              # frecuencia del lidar
ros2 topic hz /odometry/filtered                 # ~30 Hz
ros2 run tf2_ros tf2_echo odom base_link         # el TF existe y se mueve al girar el robot
ros2 run tf2_tools view_frames                   # dibuja el árbol de TF (frames.pdf)
```

### Guardar el mapa

Con `slam.launch.py` en marcha y el mapa terminado:

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/maps/mapa1'}"
```

Guarda el grafo de poses (`.posegraph` y `.data`), con el que `slam_toolbox` puede relocalizarse
(`mode: localization` y `map_file_name` en `slam_params.yaml`). Para exportar la imagen `.pgm/.yaml`
(la que usa Nav2) hará falta `nav2_map_server`, que aún no está instalado.

> Estado: `slam.launch.py` está escrito pero **pendiente de probar con el robot real**.

## Cómo activar los motores

> **Importante:** `ros2 topic pub` o el teclado solo publican `/cmd_vel`. Los motores solo se
> mueven si `motor_driver_node` está corriendo en la Jetson.

### 1. Preparar el robot
1. Para las primeras pruebas, levanta el robot para que las ruedas no toquen el suelo.
2. Enciende el portapilas de los motores (4×AA) y alimenta la Jetson con su batería.

### 2. Lanzar el robot (Jetson, por SSH)

```bash
ssh jetson@192.168.1.34
source ~/diag/env.sh            # ROS + workspace + ROS_DOMAIN_ID=42 + perfil DDS por UDP
ros2 launch robot_description robot.launch.py
```

Arranca todo el robot; los argumentos y los casos de uso están en «Lanzamientos».
Solo los motores: `ros2 launch motor_driver motor_driver.launch.py`.

Deja esta terminal abierta (los errores salen aquí). Para que siga aunque se cierre SSH, usa `tmux`:

```bash
tmux new -s robot      # lanzar el launch; salir con Ctrl+B y luego D; volver con: tmux attach -t robot
```

Para parar nodos usa `Ctrl+C` (SIGINT), no `kill`: con SIGTERM Fast DDS deja restos en
`/dev/shm` y los nodos nuevos no se descubren (se limpian con `rm -f /dev/shm/fastrtps_*`).

### 3. Mandar órdenes de movimiento

**Prueba rápida con un topic** (corta con `Ctrl+C`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"     # avanza hacia las ruedas grandes
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 3.0}}"    # gira a la izquierda
```

Con `-r 10` se publica 10 veces por segundo, necesario por el watchdog de 0,5 s.
Con valores pequeños de `angular.z` los motores pueden pitar sin girar: girar en el sitio
necesita más fuerza (ver solución de problemas).

**Teclado** (en el contenedor del PC o en la Jetson):

```bash
apt install -y ros-foxy-teleop-twist-keyboard      # solo la primera vez (en la Jetson, con sudo)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teclas: `i` avanzar, `,` retroceder, `j`/`l` girar, `k` parar, `q`/`z` subir/bajar velocidad.

**Mando (joystick)**:

```bash
apt install -y ros-foxy-joy ros-foxy-teleop-twist-joy      # solo la primera vez
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Mantén pulsado el botón de activación (en Xbox, LB) mientras mueves el stick.
Conecta el mando a la máquina donde corre el comando (lo más sencillo, la Jetson).

### 4. Visualizar en RViz2 (PC, Omarchy)


```bash
xhost +local:docker                                        # en el host, no en el contenedor
docker start ros2_foxy
docker exec -it -e DISPLAY=$DISPLAY ros2_foxy bash
ros2 topic list                                            # debe mostrar /cmd_vel, /odom, /tf, /camera/...
rviz2
```

En RViz:
- *Fixed Frame* = `odom` (o `map` si `slam.launch.py` está en marcha)
- Añade `RobotModel` (*Description Topic* = `/robot_description`), `TF` y `Odometry`
  (`/odometry/filtered`).
- Para el lidar, `LaserScan` con `/scan`; para el mapa, `Map` con `/map`.

**Imagen de la cámara.** El display `Image` de RViz2 en Foxy no sabe mostrar imágenes
comprimidas, así que en el PC corre un puente que descomprime el JPEG. `tools/pc/jpeg_relay.py`
está copiado en el volumen del contenedor (`/workspace/jpeg_relay.py`). En otra terminal del
contenedor:

```bash
export ROS_DOMAIN_ID=42
python3 /workspace/jpeg_relay.py          # cada 5 s muestra las imágenes/s recibidas
```

En RViz, display `Image` con topic `/camera/image_local`.

Alternativa sin RViz (necesita `ros-foxy-image-transport-plugins` y `ros-foxy-rqt-image-view`):
`ros2 run rqt_image_view rqt_image_view`.

`robot_state_publisher` solo lee el URDF al arrancar: tras cambiar el modelo, recompila y relanza.

## Calibración

| Ajuste | Cómo |
|---|---|
| Deriva al ir recto | Si tira a la derecha, baja `left_gain` (0.95, 0.92...). Si tira a la izquierda, baja `right_gain` |
| Gira mal o no arranca a baja velocidad | Sube `min_throttle` |
| Velocidad real | Cronometra 1 m con `linear.x` fijo y ajusta `max_linear` |

Cuando tengas los valores, ponlos como `default_value` en `launch/motor_driver.launch.py`.
Si cambias la alimentación de los motores o las pilas se gastan, hay que recalibrar.

## Solución de problemas

| Síntoma | Causa probable |
|---|---|
| Publico `/cmd_vel` pero no se mueve nada | `motor_driver_node` no está corriendo (`ros2 node list` vacío, `Subscription count: 0` en `ros2 topic info /cmd_vel -v`) |
| El nodo muere al recibir la primera orden | Error de I2C: la placa no responde. Revisa cableado y `i2cdetect` |
| Responde el nodo pero las ruedas no giran | Sin alimentación en motor power (pilas, interruptor, polaridad) |
| Los motores pitan pero no giran | Throttle insuficiente. Sube `min_throttle` o el valor de `angular.z`; revisa las pilas |
| Avanza hacia la rueda loca | Revisa `reverse_direction` |
| Con `angular.z > 0` gira a la derecha | Revisa `swap_motors` |
| Se desvía al ir recto | Ajusta `left_gain` / `right_gain` |
| Los cambios no tienen efecto | Has compilado desde `src` o no has relanzado. Compila desde `~/ros2_ws` y relanza |
| `ros2 topic list` no muestra los topics de la Jetson | Revisa `ROS_DOMAIN_ID=42`, `ROS_STATIC_PEERS`, `RMW_IMPLEMENTATION`; reinicia con `ros2 daemon stop`; comprueba el ping |
| La Jetson cambia de IP | Reserva la IP (MAC `98:48:27:CA:AB:8D`) en el router o pon una IP fija |
| `rpi_camera_node`: `No cameras available` | El driver no detecta la cámara: `dmesg \| grep ov5647`. Revisa la orientación del cable y que esté en CAM0/CAM1; reinicia |
| `ov5647 ... error during i2c read probe (-121)` | La cámara no responde: cable al revés o suelto (también el conector pequeño del sensor) |
| En RViz no se ve la imagen | Usa `/camera/image_local` con `jpeg_relay.py` en marcha. Si estás suscrito a `/camera/image_raw` desde el PC, la WiFi se satura |
| La imagen sale al revés | Cambia `upside_down` |
| El nodo de la cámara va lento y usa mucha CPU | Con `rclpy` de Foxy, asigna `array.array('B', ...)` a `msg.data`, no `bytes` |

## Limitaciones conocidas

- Sin encoders: la distancia recorrida es una estimación de lo ordenado, no de lo recorrido. El giro
  se corrige con el giroscopio del IMU (EKF), pero sin magnetómetro acumula un pequeño error.
- La deriva cambia con la velocidad y la carga de las pilas.
- Hay lidar, pero todavía no hay freno por obstáculos ni navegación autónoma (falta Nav2).
- La WiFi USB de la Jetson (rtl88x2bu) no aguanta imagen sin comprimir hacia el PC.
- La cámara usa un driver de terceros, que no forma parte de JetPack.
- Foxy está en fin de vida; se mantiene por compatibilidad con la Jetson Nano (JetPack 4.x).

## Próximos pasos

1. ~~Montar el lidar~~ y ~~el IMU~~ (hecho). Medir la posición real del IMU (x/y) en el URDF.
2. Probar el EKF girando el robot y comprobar el yaw de `/odometry/filtered`.
3. Probar SLAM con `slam_toolbox` y crear el primer mapa.
4. Instalar Nav2 (`ros-foxy-navigation2`, `ros-foxy-nav2-bringup`) y navegación autónoma.
5. Rehacer el freno por obstáculos sobre `/scan`.
6. Ajustar la posición real de la cámara en el URDF.
