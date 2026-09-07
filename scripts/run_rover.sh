#!/usr/bin/env bash
set -e

export GPIOZERO_PIN_FACTORY=lgpio
export RPI_LGPIO_CHIP=4

source /opt/ros/jazzy/setup.bash
source "$HOME/rover_ws/install/setup.bash"

ros2 launch rover_bringup rover_all.launch.py
