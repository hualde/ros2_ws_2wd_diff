# Robot diferencial con Jetson Nano y ROS 2 Foxy

Robot diferencial de dos ruedas controlado con una **Jetson Nano 4GB** y **ROS 2 Foxy**.
Este README explica cómo **activar los motores** y moverlos.

## Arquitectura

| Máquina | Función |
|---|---|
| **Jetson Nano** (`192.168.1.34`) | Va en el robot. Ejecuta `motor_driver_node`, que mueve los motores. ROS 2 Foxy nativo. |
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
- 2 motores DC amarillos TT, sin encoder (motor 1 = rueda izquierda, motor 2 = rueda derecha)
- Alimentación **separada**:
  - Jetson: batería LiPo 3S 3000 mAh, vía placa reductora a 5V (jack barrel con jumper J48 puesto).
  - Motores: portapilas 4×AA (6V, alcalinas) a los bornes "motor power" de la FeatherWing.
  - Los GND de todo deben estar unidos. No unir los positivos.
- Conexión I2C (lógica) Jetson ↔ FeatherWing: 3.3V, GND, SDA y SCL.

> Comprueba que la Jetson ve la placa con `sudo i2cdetect -y -r <bus>`: debe aparecer `60`.

## Software en la Jetson

Workspace: `~/ros2_ws`, paquete `motor_driver`.

El nodo `motor_driver_node`:
- Suscribe `/cmd_vel` (`geometry_msgs/Twist`) y calcula la velocidad de cada rueda.
- Compensa la zona muerta de los motores.
- Para los motores si no recibe `/cmd_vel` durante 0,5 s (*watchdog*).
- Publica `/odom` y el TF `odom → base_link` (estimación en lazo abierto, sin encoders).

Parámetros (en `launch/motor_driver.launch.py`):

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `wheel_separation` | 0.125 | Distancia entre ruedas (m) |
| `max_linear` | 0.5 | Velocidad lineal a throttle 1.0 (m/s) |
| `min_throttle` | 0.35 | Throttle mínimo para vencer la zona muerta |
| `odom_rate_hz` | 20.0 | Frecuencia de la odometría |

### Compilar (solo la primera vez o tras cambios)

```bash
cd ~/ros2_ws
colcon build --packages-select motor_driver
source install/setup.bash
```

## Cómo activar los motores

> **Importante:** `ros2 topic pub` o el teclado solo publican `/cmd_vel`. Los motores solo se
> mueven si `motor_driver_node` está corriendo en la Jetson.

### 1. Preparar el robot
1. Levanta el robot para que las ruedas no toquen el suelo en las primeras pruebas.
2. Enciende el portapilas de los motores (4×AA) y alimenta la Jetson con su batería.

### 2. Lanzar el nodo de motores (Jetson, por SSH)

```bash
ssh <usuario>@192.168.1.34
cd ~/ros2_ws && source install/setup.bash
ros2 launch motor_driver motor_driver.launch.py
```

Deja esta terminal abierta (los errores salen aquí). Para que siga aunque se cierre SSH, usa `tmux`:

```bash
tmux new -s robot      # lanzar el launch; salir con Ctrl+B y luego D; volver con: tmux attach -t robot
```

### 3. Mandar órdenes de movimiento

**Prueba rápida con un topic** (la velocidad máxima es 0.5):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"      # avanzar
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"     # girar en el sitio
```

Con `-r 10` se publica 10 veces por segundo, necesario por el watchdog de 0,5 s. `Ctrl+C` detiene el robot.

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

En RViz: *Fixed Frame* = `odom`, y añade `TF` y `Odometry` (`/odom`).

## Solución de problemas

| Síntoma | Causa probable |
|---|---|
| Publico `/cmd_vel` pero no se mueve nada | `motor_driver_node` no está corriendo (`ros2 node list` vacío, `Subscription count: 0` en `ros2 topic info /cmd_vel -v`) |
| El nodo muere al recibir la primera orden | Error de I2C: la placa no responde. Revisa cableado y `i2cdetect` |
| Responde el nodo pero las ruedas no giran | Sin alimentación en motor power (pilas, interruptor, polaridad) |
| Una rueda gira al revés | Invierte los dos cables de ese motor en la FeatherWing |
| El robot se desvía al ir recto | Revisa la compensación de zona muerta de cada motor y recalibra `max_linear` / `min_throttle` |
| `ros2 topic list` no muestra los topics de la Jetson | Revisa `ROS_DOMAIN_ID=42`, `ROS_STATIC_PEERS`, `RMW_IMPLEMENTATION`; reinicia con `ros2 daemon stop`; comprueba el ping |
| La Jetson cambia de IP | Reserva la IP (MAC `98:48:27:CA:AB:8D`) en el router o pon una IP fija |

## Limitaciones conocidas

- Sin encoders: la odometría es una estimación de lo ordenado, no de lo recorrido.
- Si cambias la alimentación de los motores, recalibra `max_linear` y `min_throttle`.
- Foxy está en fin de vida; se mantiene por compatibilidad con la Jetson Nano (JetPack 4.x).