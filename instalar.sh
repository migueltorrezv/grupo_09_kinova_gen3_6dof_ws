#!/usr/bin/env bash
# Instalador reproducible - Grupo 09, Kinova Gen3 6-DOF (IMT-342 Robotica)
# Requisitos: Ubuntu 24.04 + ROS 2 Jazzy instalado en /opt/ros/jazzy
# Uso (desde la raiz del repositorio):
#   chmod +x instalar.sh && ./instalar.sh
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: ROS 2 Jazzy no esta instalado en /opt/ros/jazzy."
  exit 1
fi
source /opt/ros/jazzy/setup.bash

echo ">> Instalando dependencias del sistema..."
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rmw-cyclonedds-cpp

if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

cd "$WS_DIR"
mkdir -p src

echo ">> Descargando el modelo oficial de Kinova (dependencias.repos)..."
if [ ! -d src/ros2_kortex ]; then
  vcs import src < dependencias.repos
else
  echo "   src/ros2_kortex ya existe, se conserva."
fi

echo ">> Instalando dependencias ROS (solo paquetes usados)..."
rosdep install --from-paths \
  src/grupo09_kinova_gen3_bringup \
  src/grupo09_kinova_gen3_kinematics \
  src/ros2_kortex/kortex_description \
  --ignore-src -r -y --rosdistro jazzy || true

echo ">> Compilando..."
colcon build --symlink-install \
  --packages-up-to grupo09_kinova_gen3_bringup grupo09_kinova_gen3_kinematics

echo
echo "INSTALACION COMPLETA - Grupo 09, Kinova Gen3 6-DOF"
echo "Siguiente paso:"
echo "  cd $WS_DIR"
echo "  source entorno.sh"
echo "  ros2 launch grupo09_kinova_gen3_bringup display.launch.py"
