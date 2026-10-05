#!/usr/bin/env bash
# Carga ROS 2 Jazzy, CycloneDDS y este workspace.
# Uso (desde cualquier carpeta):  source entorno.sh

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi
