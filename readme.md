# Robot diferencial con Jetson Nano y ROS 2 Foxy

Robot diferencial de dos ruedas controlado con una **Jetson Nano 4GB** y **ROS 2 Foxy**.
Objetivo: recorrer una zona de forma autónoma y hacer reconocimiento con una cámara OAK-D Lite.

## Arquitectura

| Máquina | Función |
|---|---|
| **Jetson Nano** (`192.168.1.34`) | Va en el robot. Ejecuta los nodos de motores, cámara y modelo. ROS 2 Foxy nativo. |
| **PC (Omarchy)** | Estación de trabajo. Contenedor Docker `ros2_foxy` con RViz2 y teleoperación. |

Las dos máquinas se comunican por red con ROS 2 (DDS):

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_STATIC_PEERS=<IP de la otra máquina>   # en la Jetson: IP del PC; en el PC: 192.168.1.34
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Hardware

- Jetson Nano 4GB
- Placa **Adafruit DC Motor + Stepper FeatherWing** (PCA9685, I2C `0x60`)
- 2 motores DC amarillos TT, sin encoder, con ruedas de 65 mm
- Rueda loca (bola) a 9 cm por detrás del eje de las ruedas motrices
- Cámara **OAK-D Lite** (USB 3)
- Alimentación **separada**:
  - Jetson: batería LiPo 3S 3000 mAh, vía placa reductora a 5V (jack barrel con jumper J48 puesto).
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
| `oak_camera_driver` | Nodos de la cámara OAK-D (`oak_camera_node`, `oak_spatial_grid_node`) |

### `motor_driver_node`

- Suscribe `/cmd_vel` (`geometry_msgs/Twist`) y calcula la velocidad de cada rueda.
- Compensa la zona muerta de los motores.
- Para los motores si no recibe `/cmd_vel` durante 0,5 s (*watchdog*) y al cerrar el nodo.
- Publica `/odom` y el TF `odom → base_link` (estimación en lazo abierto, sin encoders).

Parámetros:

| Parámetro | Valor | Descripción |
|---|---|---|
| `wheel_separation` | 0.125 | Distancia entre ruedas (m) |
| `max_linear` | 0.5 | Velocidad lineal a throttle 1.0 (m/s) |
| `min_throttle` | 0.35 | Throttle mínimo para vencer la zona muerta |
| `odom_rate_hz` | 20.0 | Frecuencia de la odometría |
| `reverse_direction` | true | Invierte el frente (ruedas grandes delante) |
| `swap_motors` | true | Intercambia motor 1 y motor 2 (giro correcto) |
| `left_gain` | 1.0 | Ganancia rueda izquierda. Bajar si se desvía a la derecha |
| `right_gain` | 1.0 | Ganancia rueda derecha. Bajar si se desvía a la izquierda |

`reverse_direction` y `swap_motors` se definen en el código; el resto se puede pasar como
argumento del launch, por ejemplo `ros2 launch motor_driver motor_driver.launch.py left_gain:=0.95`.

### `robot_description`

Modelo URDF (`urdf/robot.urdf`) con cuerpo, ruedas, rueda loca, Jetson y cámara.
`robot_state_publisher` publica los TF fijos del robot; `motor_driver` publica `odom → base_link`.

Las medidas del cuerpo y de la cámara son aproximadas (sacadas de una foto); la posición de la
cámara es provisional hasta que se monte.

### Compilar

Siempre desde la raíz del workspace, **nunca desde `src`**:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

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

Arranca el modelo (`robot_state_publisher`) y los motores (`motor_driver_node`).
En el log debe aparecer `motor_driver_node iniciado (...)`.

- Solo el modelo, sin motores: `ros2 launch robot_description robot.launch.py with_motors:=false`
- Solo los motores: `ros2 launch motor_driver motor_driver.launch.py`

Deja esta terminal abierta (los errores salen aquí). Para que siga aunque se cierre SSH, usa `tmux`:

```bash
tmux new -s robot      # lanzar el launch; salir con Ctrl+B y luego D; volver con: tmux attach -t robot
```

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
ros2 topic list                                            # debe mostrar /cmd_vel, /odom, /tf
rviz2
```

En RViz:
- *Fixed Frame* = `odom`
- Añade `RobotModel` (*Description Topic* = `/robot_description`), `TF` y `Odometry` (`/odom`).
- Para la cámara, `Image` en `/oak/rgb/image_raw` con *Reliability* = *Best Effort*.

`robot_state_publisher` solo lee el URDF al arrancar: tras cambiar el modelo, recompila y relanza.

## Calibración

| Ajuste | Cómo |
|---|---|
| Deriva al ir recto | Si tira a la derecha, baja `left_gain` (0.95, 0.92...). Si tira a la izquierda, baja `right_gain` |
| Gira mal o no arranca a baja velocidad | Sube `min_throttle` (0.5) |
| Velocidad real | Cronometra 1 m con `linear.x` fijo y ajusta `max_linear` |

Cuando tengas los valores, ponlos como `default_value` en `launch/motor_driver.launch.py`.
Si cambias la alimentación de los motores o las pilas se gastan, hay que recalibrar.

## Solución de problemas

| Síntoma | Causa probable |
|---|---|
| Publico `/cmd_vel` pero no se mueve nada | `motor_driver_node` no está corriendo (`ros2 node list` vacío, `Subscription count: 0` en `ros2 topic info /cmd_vel -v`) |
| El log de `motor_driver_node` dice `oak_camera_node iniciado` | `motor_driver_node.py` se ha sobrescrito con el código de la cámara. Restaura el archivo correcto desde git |
| El nodo muere al recibir la primera orden | Error de I2C: la placa no responde. Revisa cableado y `i2cdetect` |
| Responde el nodo pero las ruedas no giran | Sin alimentación en motor power (pilas, interruptor, polaridad) |
| Los motores pitan pero no giran | Throttle insuficiente. Sube `min_throttle` o el valor de `angular.z`; revisa las pilas |
| Avanza hacia la rueda loca | Revisa `reverse_direction` |
| Con `angular.z > 0` gira a la derecha | Revisa `swap_motors` |
| Se desvía al ir recto | Ajusta `left_gain` / `right_gain` |
| Los cambios no tienen efecto | Has compilado desde `src` o no has relanzado. Compila desde `~/ros2_ws` y relanza |
| `ros2 topic list` no muestra los topics de la Jetson | Revisa `ROS_DOMAIN_ID=42`, `ROS_STATIC_PEERS`, `RMW_IMPLEMENTATION`; reinicia con `ros2 daemon stop`; comprueba el ping |
| La Jetson cambia de IP | Reserva la IP (MAC `98:48:27:CA:AB:8D`) en el router o pon una IP fija |

## Limitaciones conocidas

- Sin encoders: la odometría es una estimación de lo ordenado, no de lo recorrido.
- La deriva cambia con la velocidad y la carga de las pilas; sin encoders o IMU nunca irá perfectamente recto.
- La OAK-D con `depthai` ocupa unos 3 núcleos de la Nano (coste fijo, medido con `tegrastats`),
  lo que limita su uso a la vez que SLAM y Nav2.
- Foxy está en fin de vida; se mantiene por compatibilidad con la Jetson Nano (JetPack 4.x).

## Próximos pasos

1. Montar la cámara y ajustar su posición en el URDF.
2. Elegir el sensor de navegación (OAK-D o un LiDAR 2D como RPLIDAR A1).
3. SLAM con `slam_toolbox` para crear el mapa.
4. Navegación autónoma con Nav2.
5. Reconocimiento con la OAK-D.