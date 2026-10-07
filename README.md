# Cinemática directa e inversa del Kinova Gen3 6-DOF con ROS 2 Jazzy

**IMT-342 Robótica — Primer Parcial Práctico**
Universidad Católica Boliviana "San Pablo"
Docente: Carlos Daniel Aguilar Mancachi

**Grupo 09**
- Miguel Angel Torrez
- Henrry Pablo Lima Poma

## Objetivo

Modelar la cinemática del robot Kinova Gen3 de 6 grados de libertad mediante la convención Denavit–Hartenberg estándar e implementar en ROS 2:

- un nodo de **cinemática directa** (`fk_node`) que calcula la pose del efector final a partir de `/joint_states`;
- un nodo de **cinemática inversa de posición** (`ik_node`) que recibe un objetivo cartesiano en `/target` y publica la solución articular en `/joint_states`, mediante el método iterativo de la pseudoinversa del Jacobiano posicional.

## Software requerido

| Componente | Versión |
|---|---|
| Sistema operativo | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Python | 3.12 |
| Middleware | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Librerías Python | NumPy |
| Visualización | RViz2, Joint State Publisher GUI |

ROS 2 Jazzy debe estar instalado previamente en `/opt/ros/jazzy`.

## Instalación

```bash
git clone https://github.com/migueltorrezv/grupo_09_kinova_gen3_6dof_ws.git ~/grupo_09_kinova_gen3_6dof_ws
cd ~/grupo_09_kinova_gen3_6dof_ws
chmod +x instalar.sh
./instalar.sh
```

`instalar.sh` instala las dependencias del sistema, descarga el modelo oficial del robot, resuelve las dependencias con `rosdep` y compila el workspace.

### Modelo oficial del robot

El paquete `kortex_description` pertenece a Kinova y no se incluye en este repositorio. Se descarga automáticamente desde `dependencias.repos`, fijado a un commit conocido:

| Repositorio | Commit |
|---|---|
| https://github.com/Kinovarobotics/ros2_kortex | `462dab9aa4732d733be55e1846530dd920c7c7d3` |

Descarga manual equivalente:

```bash
vcs import src < dependencias.repos
```

## Compilación

```bash
cd ~/grupo_09_kinova_gen3_6dof_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source install/setup.bash
```

En cada terminal nueva basta con:

```bash
cd ~/grupo_09_kinova_gen3_6dof_ws
source entorno.sh
```

## Ejecución

### Visualización con Joint State Publisher GUI

```bash
ros2 launch grupo09_kinova_gen3_bringup display.launch.py
```

### Cinemática directa

Con el robot abierto, en otra terminal:

```bash
ros2 run grupo09_kinova_gen3_kinematics fk_node
```

Al mover los sliders del GUI, el nodo muestra `q`, la posición `(x, y, z)`, el cuaternión y la matriz de rotación del efector final, y publica la pose en `/fk_pose`.

### Cinemática inversa

El GUI y `ik_node` publican ambos en `/joint_states`, por lo que no deben ejecutarse al mismo tiempo. Para la cinemática inversa se utiliza un launch sin GUI que incluye `ik_node` y `fk_node`:

```bash
ros2 launch grupo09_kinova_gen3_bringup ik_demo.launch.py
```

En otra terminal se envía el objetivo:

```bash
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.40, y: 0.20, z: 0.50}" -t 3
```

### Ejemplo de prueba

Objetivo `(0.40, 0.20, 0.50)` partiendo de la postura home:

```
target p_d [m]   = [0.4 0.2 0.5]
q0 [rad]         = [ 0.      0.2618 -2.2689  0.      0.9599  1.5708]
q* [rad]         = [-0.4198  0.1541 -2.2085  0.1294  0.9999  1.5708]
iterations       = 13
p(q*) [m]        = [0.4    0.1999 0.5   ]
error ||e|| [m]  = 0.000064
status           = CONVERGED
```

## Modelo cinemático

Convención DH estándar, `A_{i-1}^{i} = Rz(θi) Tz(di) Tx(ai) Rx(αi)`, con una transformación fija `T_base^0 = Rx(π)` entre `base_link` y el marco 0.

| i | θi | di [m] | ai [m] | αi [rad] |
|---|---|---|---|---|
| 1 | q1 | −0.28481 | 0 | π/2 |
| 2 | q2 − π/2 | −0.005375 | 0.410 | π |
| 3 | q3 − π/2 | −0.006375 | 0 | π/2 |
| 4 | q4 | −0.31436 | 0 | −π/2 |
| 5 | q5 | −0.00035 | 0 | π/2 |
| 6 | q6 + π | −0.167455 | 0 | π |

Parámetros de la cinemática inversa:

| Parámetro | Valor |
|---|---|
| Configuración inicial | home de Kinova: (0°, 15°, −130°, 0°, 55°, 90°); luego, la última solución |
| Factor de actualización α | 0.5 |
| Tolerancia ε | 1×10⁻⁴ m |
| Iteraciones máximas | 500 |
| Paso máximo por iteración | 0.2 rad |
| Límites articulares | URDF; joints continuos 1, 4 y 6 acotados a [−π, π] |

## Tópicos

| Tópico | Tipo | Publica | Suscribe |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | GUI o `ik_node` | `robot_state_publisher`, `fk_node` |
| `/target` | `geometry_msgs/msg/Point` | usuario (terminal) | `ik_node` |
| `/fk_pose` | `geometry_msgs/msg/PoseStamped` | `fk_node` | — |
| `/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` | RViz2 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | RViz2 |

## Estructura del repositorio

```
grupo_09_kinova_gen3_6dof_ws/
├── src/
│   ├── grupo09_kinova_gen3_bringup/
│   │   ├── launch/
│   │   │   ├── display.launch.py
│   │   │   └── ik_demo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── grupo09_kinova_gen3_kinematics/
│       ├── grupo09_kinova_gen3_kinematics/
│       │   ├── kinematics.py
│       │   ├── fk_node.py
│       │   └── ik_node.py
│       ├── resource/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
├── docs/
│   ├── GRUPO_09_KINOVA_GEN3_6DOF_INFORME_PRIMER_PARCIAL.pdf
│   ├── figura_frames.pdf
│   ├── figura_frames.png
│   └── capturas/
├── dependencias.repos
├── instalar.sh
├── entorno.sh
├── requirements.txt
├── README.md
└── .gitignore
```

## Consideraciones conocidas

- **Conflicto en `/joint_states`:** si el Joint State Publisher GUI sigue activo durante la cinemática inversa, su postura sobrescribe la solución de `ik_node`. En algunos casos cerrar la ventana no termina el proceso; se puede forzar con `pkill -9 -f joint_state_publisher_gui`.
- **Publicación única:** `ros2 topic pub --once` puede enviar el mensaje antes de que se establezca la conexión con el suscriptor. Se recomienda `-t 3`.
- **Middleware:** todas las terminales deben usar CycloneDDS (`source entorno.sh`).
- **Joints continuos:** joint_1, joint_4 y joint_6 no tienen límites en el URDF; la cinemática inversa los acota a [−π, π].
- **Singularidad:** con todas las articulaciones en cero el brazo está completamente extendido y el Jacobiano pierde rango; por ello la cinemática inversa no parte de `q = 0`.
- **Objetivos fuera de alcance:** el alcance aproximado del brazo es 0.89 m desde el hombro. Si el objetivo no es alcanzable, `ik_node` informa `NOT CONVERGED` y conserva la postura anterior.
- **Joint 6:** el efector final se encuentra sobre el eje de joint_6, por lo que esta articulación no modifica la posición; su columna en el Jacobiano posicional es nula.
- **Wayland (Ubuntu 24.04):** el Joint State Publisher GUI puede no responder al mouse ni al teclado (sliders, Center, Randomize). `entorno.sh` fuerza X11 con `QT_QPA_PLATFORM=xcb`, lo que soluciona el problema.
