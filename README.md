# 🐕‍🦺 Control del Unitree B2 con ROS 2

Este repositorio contiene el stack ROS 2 para controlar un **Unitree B2** usando el **Ai Motion Interface** de Unitree, incluyendo:

- Descripción del robot (URDF/TF).
- Driver ROS 2 ↔ SDK de Unitree (Ai Motion).
- Teleoperación con mando (p. ej. PS5) vía `joy` + `twist_mux` + `joy_estop`.
- Servicios ROS 2 para cambiar modos, niveles de velocidad, autorecuperación, etc.
- Interfaces propias (`b2_interfaces`) para encapsular las llamadas a la API del B2.

El workspace de ejemplo utilizado es: `~/prueba_b2`.

---

## 📦 Dependencias previas

### 1. Repo oficial de Unitree ROS 2

Clonar y actualizar el repo oficial:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2
git pull
````

### 2. Compilar `cyclonedds` según el repo de Unitree

Dentro del workspace de `cyclonedds` (según documentación del repo `unitree_ros2`):

```bash
rm -rf build install log

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

colcon build --packages-select cyclonedds
```

> ⚠️ Sigue los pasos exactos del README de `unitree_ros2`, ya que pueden cambiar.

### 3. Configuración de Ethernet

Asegúrate de tener configurado el enlace Ethernet con el B2 según la guía de Unitree:

* IP del PC en el rango adecuado.
* Capacidad de hacer ping al B2.
* El SDK de Unitree debe ser capaz de comunicarse correctamente con el robot antes de usar ROS 2.

---

## 🧱 Paquetes principales de este stack

En el workspace `~/prueba_b2` se utilizan, entre otros, los siguientes paquetes:

* `b2_description` → URDF / descripción cinemática del B2.
* `b2_interfaces` → mensajes y servicios específicos para el B2.
* `b2_driver` → nodo que hace de **puente** entre ROS 2 y el SDK Ai Motion.
* `b2_bringup` → launch files para levantar todo (driver, TF, teleop, etc.).
* Paquetes auxiliares reutilizados:

  * `joy_estop` → capa de teleop con deadman + e-stop desde el mando.
  * `twist_mux` → multiplexor de velocidades (`cmd_vel`).

---

## 🔧 `b2_description`

Paquete estándar de descripción del robot:

* Contiene el **URDF/Xacro** del B2 con:

  * Links: `base_link`, patas (`FL_*`, `FR_*`, `RL_*`, `RR_*`), `imu_link`, `lidar_link`, etc.
  * Joints con los nombres correctos para publicar `joint_states`.
* Se usa en el bringup para lanzar `robot_state_publisher` y publicar la TF tree.

Se incluye normalmente desde `b2_bringup` en el launch principal.

---

## 📡 `b2_interfaces`

Paquete de **interfaces ROS 2** propias para el B2.

### Servicios definidos (ejemplos)

En el directorio `srv/`:

#### `Mode.srv`

Para cambiar el modo de movimiento:

```srv
string mode       # ej: "balance_stand", "stand_down", "classic_walk", "free_walk", ...
---
bool success
string message
```

#### `SpeedLevel.srv`

Para ajustar el nivel de velocidad:

```srv
int32 level      # -1: lento, 0: normal, 1: rápido
---
bool success
string message
```

#### `ContinuousGait.srv`

Para activar/desactivar marcha continua:

```srv
bool flag        # true = activar, false = desactivar
---
bool success
string message
```

Además se usan servicios estándar:

* `std_srvs/SetBool` → para `autorecover` (activar/desactivar auto-recuperación).

Este paquete se instala y se usa como dependencia en `b2_driver` y `b2_bringup`.

---

## 🧠 `b2_driver`: puente ROS 2 ↔ Ai Motion (B2)

`b2_driver` define el nodo `b2_driver::B2Driver` (hereda de `rclcpp::Node`), que hace de puente entre:

* ROS 2 (topics, servicios, teleop, Nav2 en el futuro)
* Y el SDK de Unitree (Ai Motion) vía la API `SportClient`.

### Internals

* Usa un cliente `SportClient` basado en los ejemplos oficiales (`ros2_b2_sport_client.{h,cpp}`).
* Publica mensajes `unitree_api::msg::Request` en el topic que espera el SDK (`api/sport/request` o similar).
* Codifica los comandos en JSON (`Move`, `StandUp`, `StandDown`, `BalanceStand`, `FreeWalk`, `ClassicWalk`, etc.) usando los `ROBOT_SPORT_API_ID_*`.
* Se suscribe al estado de deporte `SportModeState`:

  * Topic típico: `/sportmodestate` o `rt/sportmodestate`.
  * Campos relevantes: `mode`, `error_code`, `position`, `imu_state`, etc.

### Topics y servicios del driver

#### Suscripciones

* `cmd_vel` (`geometry_msgs/Twist`)

  → (Remapeado a `/b2/cmd_vel` en el launch).

  Callback típico:

  ```cpp
  void B2Driver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Lógica de cooldown / stop si hace mucho que no llega cmd_vel
    // y llamada a SportClient::Move(vx, vy, wz)

    sport_client_.Move(req_, msg->linear.x, msg->linear.y, msg->angular.z);
  }
  ```

* `/sportmodestate` (`unitree_go::msg::SportModeState`)

  Para leer el modo actual, errores, posición, etc.

#### Publicaciones

* `/error_code` (`std_msgs/String` o similar)
  → Texto o código de error reportado por el B2.
* `/odom`, `/joint_states`, etc.
  → Se pueden derivar de `SportModeState` y estados de bajo nivel, si se implementan.

#### Servicios expuestos

* `/mode` (`b2_interfaces/srv/Mode`)

  Mapea strings de modo a llamadas del `SportClient`:

  ```cpp
  if (mode == "damp") {
    sport_client_.Damp(req_);
  } else if (mode == "balance_stand") {
    sport_client_.BalanceStand(req_);
  } else if (mode == "stand_up") {
    sport_client_.StandUp(req_);
  } else if (mode == "stand_down") {
    sport_client_.StandDown(req_);
  } else if (mode == "recovery_stand") {
    sport_client_.RecoveryStand(req_);
  } else if (mode == "free_walk") {
    sport_client_.FreeWalk(req_);
  } else if (mode == "classic_walk") {
    sport_client_.ClassicWalk(req_, true);
  } else if (mode == "fast_walk") {
    sport_client_.FastWalk(req_, true);
  }
  ```

* `/speed_level` (`b2_interfaces/srv/SpeedLevel`)

  → llama a `SportClient::SpeedLevel(level)`.

* `/autorecover` (`std_srvs/SetBool`)

  → llama a `SportClient::AutoRecoverySet(flag)`.

Se pueden añadir más (Euler, TrajectoryFollow, VisionWalk, etc.) siguiendo el mismo patrón.

### Registro como componente

El driver se compila como componente para ser cargado en un `component_container`:

```cpp
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(b2_driver::B2Driver)
```

---

## 🚀 `b2_bringup`: lanzamiento completo

El paquete `b2_bringup` contiene el launch principal `b2_bringup.launch.py` que arranca:

* `robot_state_publisher` (con el URDF de `b2_description`).
* Un `component_container` (`b2_container`) con el `b2_driver`.
* `twist_mux` con su `twist_mux.yaml`.
* `joy_node` (mando).
* `joy_estop_node` (deadman + e-stop).
* (Opcional) nodos de RViz u otros.

### Launch del driver (`b2_driver.launch.py`)

Ejemplo simplificado de cómo se carga el componente:

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

composable_node = ComposableNode(
    package='b2_driver',
    plugin='b2_driver::B2Driver',
    name='b2_driver',
    namespace='',
    remappings=[
        ('/cmd_vel', '/b2/cmd_vel'),  # hacia fuera se usa /b2/cmd_vel
    ],
)

container = ComposableNodeContainer(
    name='b2_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[composable_node],
    output='screen',
)
```

El driver ve internamente el topic `cmd_vel`, mientras que el resto del sistema trabaja con `/b2/cmd_vel` gracias al remapping.

### `twist_mux` + `joy_estop`

Cadena típica:

* `joy_node` → publica `sensor_msgs/Joy` en `/joy`.
* `joy_estop_node`:

  * Lee `/joy`.
  * Genera `/cmd_vel_teleop`.
  * Publica un topic de bloqueo `/e_stop` cuando se pulsa el botón de emergencia.
* `twist_mux`:

  * Suscribe:

    * `cmd_vel_teleop` (teleop mando)
    * `cmd_vel_dock` (dockeo, si existe)
    * `cmd_vel_estop` (parada forzada)
  * Según prioridades/timeouts, publica un único `/cmd_vel`.

Cadena completa de teleop:

```text
mando PS5 → /joy
          → joy_estop → /cmd_vel_teleop + /e_stop
          → twist_mux → /cmd_vel
          → (remap) → /b2/cmd_vel → B2Driver → SportClient.Move() → B2
```

---

## 🔨 Compilación

En el workspace `~/prueba_b2`:

```bash
cd ~/prueba_b2
colcon build
source install/setup.bash
```

Si se solapan paquetes con otros workspaces (como `unitree_api`, `unitree_go`), puede ser necesario:

```bash
colcon build --allow-overriding unitree_api unitree_go
```

---

## 🏃 Lanzar el stack y teleoperar

### 1. Lanzar bringup

```bash
cd ~/prueba_b2
source install/setup.bash
ros2 launch b2_bringup b2_bringup.launch.py
```

Comprobar:

```bash
ros2 topic list | grep cmd_vel
ros2 topic list | grep joy
ros2 node list | grep b2
```

### 2. Comprobar que el mando se ve en ROS 2

```bash
ros2 topic echo /joy
```

* Mover sticks y botones → valores cambian.

Después:

```bash
ros2 topic echo /cmd_vel_teleop
```

* Con el botón deadman pulsado + stick hacia delante → `linear.x` > 0.

Y:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /b2/cmd_vel
```

* Deberían reflejar el mismo `Twist` si no hay bloqueos por e-stop.

---

## 🤖 Poner el B2 a andar en modo “IA” con el mando

En otra terminal:

```bash
cd ~/prueba_b2
source install/setup.bash
```

1. **Desbloquear y ponerse en equilibrio (balance)**

   ```bash
   ros2 service call /mode b2_interfaces/srv/Mode "{mode: 'balance_stand'}"
   ```

2. **Activar un gait “inteligente” (controlado por Ai Motion)**

   ```bash
   ros2 service call /mode b2_interfaces/srv/Mode "{mode: 'classic_walk'}"
   # o:
   ros2 service call /mode b2_interfaces/srv/Mode "{mode: 'free_walk'}"
   ```

3. **Ajustar nivel de velocidad**

   ```bash
   ros2 service call /speed_level b2_interfaces/srv/SpeedLevel "{level: 0}"
   ```

4. **(Opcional) Activar auto-recovery**

   ```bash
   ros2 service call /autorecover std_srvs/srv/SetBool "{data: true}"
   ```

5. **Probar sin mando (solo para verificar conexión)**

   ```bash
   ros2 topic pub /b2/cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
   ```

   Si el robot anda suave, la parte driver/SDK está bien.

6. **Usar el mando**

   * Mantener pulsado el botón deadman.
   * Quitar e-stop si estuviera activado.
   * Mover los sticks para avanzar/girar.

---

## 🩺 Debugging útil

* Ver el estado de marcha del B2:

  ```bash
  ros2 topic list | grep sport
  ros2 topic echo rt/sportmodestate
  ```

  Campos interesantes:

  * `mode`

    * 0: idle (stand por defecto)
    * 1: balanceStand
    * 3: locomotion
    * 5: lieDown
    * 6: jointLock
    * 7: damping
    * 8: recoveryStand
    * 9: FreeWalk
    * 18: ClassicWalk
    * 19: FastWalk
    * 20: Euler
  * `error_code` (0 = sin error).

* Ver `cmd_vel` reales:

  ```bash
  ros2 topic echo /cmd_vel
  ros2 topic echo /b2/cmd_vel
  ```

* Añadir logs en el driver:

  En `cmdVelCallback`, usar:

  ```cpp
  RCLCPP_INFO(get_logger(), "cmd_vel: vx=%.3f vy=%.3f wz=%.3f",
              msg->linear.x, msg->linear.y, msg->angular.z);
  ```

  Y recompilar para ver en consola qué comandos están llegando al driver.

---

## 📌 Notas finales

* Este stack está pensado para ser lo más “sano” posible:

  * Usa el SDK oficial (`unitree_ros2`) y su Ai Motion Interface.
  * Encapsula las llamadas en un driver propio (`b2_driver`).
  * Expone servicios/topics claros (`b2_interfaces` + `cmd_vel`).
* A futuro se puede conectar directamente con:

  * Nav2 (remapeando `/cmd_vel` de Nav2 a `/b2/cmd_vel`).
  * Otros módulos de planificación / navegación.

Cualquier contribución o mejora (nuevos servicios, nuevos modos, conversión completa de estados a `odom` y `joint_states`, etc.) es bienvenida.
