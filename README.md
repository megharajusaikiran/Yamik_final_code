# YAMIK Rover

ROS 2 Jazzy skid-steer rover on Raspberry Pi 5, Ubuntu 24.
4x 24V motors, 2x BTS7960 drivers, LiDAR, dual ultrasonic, NEO-6M GPS, BNO055 IMU.

## Hardware map
| Device | Interface | Port |
|---|---|---|
| YDLIDAR X2 | USB-TTL | /dev/ttyUSB1 |
| Rear ultrasonic (DYP) | USB-TTL | /dev/ttyUSB0 |
| GPS NEO-6M | USB-TTL | /dev/ttyUSB2 (9600 baud) |
| Front ultrasonic (DYP) | Pi UART | /dev/ttyAMA0 |
| IMU BNO055 | I2C bus 1 | address 0x28 |

## Packages
- rover_bringup - main launch (motors, sensors, rosbridge, lidar, safety)
- rover_nav - GPS node, IMU node, waypoint navigation node
- ui/index.html - web UI (v5): telemetry, manual drive, calibration, Set North, waypoint GO

## Run
ros2 launch rover_nav rover_master.launch.py
UI: http://<pi-ip>/index.html (rosbridge on port 9090)

## Topics
/scan  /ultrasonic/front  /ultrasonic/rear  /gps/fix  /imu/heading
/cmd_vel_raw (UI+nav) -> /cmd_vel (motor, via lidar safety node)

## Notes
- IMU calibration + north offset are saved in ~/bno055_calib.json and
  ~/bno055_north.json (per-rover, per-location - NOT committed to git).
- Motor node convention: linear.x = steering (+right), angular.z = inverted throttle.
- After cloning on a new Pi: enable I2C (raspi-config), add user to i2c group, reboot.
