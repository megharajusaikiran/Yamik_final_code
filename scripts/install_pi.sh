#!/usr/bin/env bash
set -e

echo "Installing YAMIK Rover dependencies..."

sudo apt update

sudo apt install -y \
  git \
  python3-pip \
  python3-venv \
  python3-serial \
  python3-gpiozero \
  python3-lgpio \
  python3-colcon-common-extensions \
  ros-jazzy-ros-base \
  ros-jazzy-rosbridge-server

echo "Adding current user to dialout group for UART/USB-TTL access..."
sudo usermod -aG dialout "$USER"

echo "Dependencies installed."
echo "IMPORTANT: Reboot or log out/login before running rover software."
