# YAMIK Technologies Rover

ROS 2 Jazzy skid-steer rover project for Raspberry Pi 5 running Ubuntu 24.04.

## Hardware

- Raspberry Pi 5
- Ubuntu 24.04
- ROS 2 Jazzy
- Two BTS7960 motor drivers
- Four 24 V brushed DC motors in skid-steer configuration
- Front A02YYUW UART ultrasonic sensor on Pi UART
- Rear A02YYUW UART ultrasonic sensor through USB-TTL adapter
- Local PC HTML control dashboard through rosbridge WebSocket

## ROS Topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Browser and ROS rover movement command |
| `/ultrasonic/front` | `sensor_msgs/msg/Range` | Front ultrasonic distance |
| `/ultrasonic/rear` | `sensor_msgs/msg/Range` | Rear ultrasonic distance |

## Run

```bash
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_bringup rover_all.launch.py
```

## Safety

- Test with wheels lifted first.
- Use a fused 24 V motor supply and physical emergency switch.
- Use a separate regulated supply for the Raspberry Pi.
- Pi ground, motor-driver logic ground, and battery negative must share common ground.
- Never commit passwords, API keys, or private SSH keys.
