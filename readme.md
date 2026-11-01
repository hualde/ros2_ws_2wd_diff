# Robot diferencial con Jetson Nano y ROS 2 Foxy

Robot diferencial de dos ruedas controlado con una **Jetson Nano 4GB** y **ROS 2 Foxy**.
Objetivo: recorrer una zona de forma autónoma. La navegación se hará con un **lidar 2D**
(pendiente de montar); mientras tanto el robot lleva una cámara Raspberry Pi para ver la imagen.

> La versión anterior con cámara OAK-D Lite (rejilla de profundidad, freno por obstáculos y
> explorador autónomo) está en el historial de git (commit `770916a`).

## Arquitectura

| Máquina | Función |
|---|---|
| **Jetson Nano** (`192.168.1.34`) | Va en el robot. Ejecuta los nodos de motores, cámara y modelo. ROS 2 Foxy nativo. |
| **PC (Omarchy)** | Estación de trabajo. Contenedor Docker `ros2_foxy` con RViz2, teleoperación y el puente de imagen. |

```
 JETSON NANO
   robot_state_publisher ── TF fijas del URDF
   motor_driver_node     ── /cmd_vel ──► motores ; publica /odom + TF odom→base_link
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

### `robot_description`

Modelo URDF (`urdf/robot.urdf`) con cuerpo, ruedas, rueda loca, Jetson y cámara.
`robot_state_publisher` publica los TF fijos del robot; `motor_driver` publica `odom → base_link`.

Las medidas del cuerpo y de la cámara son aproximadas; la posición de la cámara
(`camera_joint`) es provisional.

### Compilar

Siempre desde la raíz del workspace, **nunca desde `src`**:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

`build/`, `install/` y `log/` no se suben a git (`.gitignore`).

## Cómo activar los motores

> **Importante:** `ros2 topic pub` o el teclado solo publican `/cmd_vel`. Los motores solo se
> mueven si `motor_driver_node` está corriendo en la Jetson.

### 1. Preparar el robot
1. Para las primeras pruebas, levanta el robot para que las ruedas no toquen el suelo.
2. Enciende el portapilas de los motores (4×AA) y alimenta la Jetson con su batería.

### 2. Lanzar el robot (Jetson, por SSH)

```bash
ssh jetson@192.168.1.34
cd ~/ros2_ws && source install/setup.bash
ros2 launch robot_description robot.launch.py
```

Arranca el modelo (`robot_state_publisher`), los motores (`motor_driver_node`) y la cámara
(`rpi_camera_node`).

- Sin motores: `ros2 launch robot_description robot.launch.py with_motors:=false`
- Sin cámara: `ros2 launch robot_description robot.launch.py with_camera:=false`
- Solo los motores: `ros2 launch motor_driver motor_driver.launch.py`

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
- *Fixed Frame* = `odom`
- Añade `RobotModel` (*Description Topic* = `/robot_description`), `TF` y `Odometry` (`/odom`).

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

- Sin encoders: la odometría es una estimación de lo ordenado, no de lo recorrido.
- La deriva cambia con la velocidad y la carga de las pilas; sin encoders o IMU nunca irá perfectamente recto.
- Sin sensor de distancia (hasta que llegue el lidar): no hay freno ni navegación autónoma.
- La WiFi USB de la Jetson (rtl88x2bu) no aguanta imagen sin comprimir hacia el PC.
- La cámara usa un driver de terceros, que no forma parte de JetPack.
- Foxy está en fin de vida; se mantiene por compatibilidad con la Jetson Nano (JetPack 4.x).

## Próximos pasos

1. Montar el lidar 2D y su driver (`/scan`), y añadirlo al URDF.
2. Rehacer el freno por obstáculos y el explorador sobre `/scan`.
3. SLAM con `slam_toolbox` para crear el mapa.
4. Navegación autónoma con Nav2.
5. Ajustar la posición real de la cámara en el URDF.
