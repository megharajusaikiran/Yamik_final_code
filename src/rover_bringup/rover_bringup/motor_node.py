import os

os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
os.environ['RPI_LGPIO_CHIP'] = '4'

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from gpiozero import PWMOutputDevice, DigitalOutputDevice


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.max_linear_speed = 0.60
        self.invert_left = True
        self.invert_right = True
        self.wheel_base = 0.42
        self.safety_distance_cm = 20.0

        # LEFT BTS7960
        # RPWM: GPIO18 / physical pin 12
        # LPWM: GPIO23 / physical pin 16
        # R_EN: GPIO17 / physical pin 11
        # L_EN: GPIO27 / physical pin 13
        self.left_rpwm = PWMOutputDevice(
            18,
            frequency=1000,
            initial_value=0
        )

        self.left_lpwm = PWMOutputDevice(
            23,
            frequency=1000,
            initial_value=0
        )

        self.left_ren = DigitalOutputDevice(
            17,
            initial_value=True
        )

        self.left_len = DigitalOutputDevice(
            27,
            initial_value=True
        )

        # RIGHT BTS7960
        # RPWM: GPIO12 / physical pin 32
        # LPWM: GPIO16 / physical pin 36
        # R_EN: GPIO22 / physical pin 15
        # L_EN: GPIO24 / physical pin 18
        self.right_rpwm = PWMOutputDevice(
            12,
            frequency=1000,
            initial_value=0
        )

        self.right_lpwm = PWMOutputDevice(
            16,
            frequency=1000,
            initial_value=0
        )

        self.right_ren = DigitalOutputDevice(
            22,
            initial_value=True
        )

        self.right_len = DigitalOutputDevice(
            24,
            initial_value=True
        )

        self.front_distance_cm = 999.0
        self.rear_distance_cm = 999.0
        self.lock = threading.Lock()
        self.last_command_time = time.monotonic()

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Range,
            '/ultrasonic/front',
            self.front_callback,
            10
        )

        self.create_subscription(
            Range,
            '/ultrasonic/rear',
            self.rear_callback,
            10
        )

        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            'Motor node started: Raspberry Pi 5 lgpio chip 4.'
        )

    def front_callback(self, message):
        with self.lock:
            self.front_distance_cm = float(message.range) * 100.0

    def rear_callback(self, message):
        with self.lock:
            self.rear_distance_cm = float(message.range) * 100.0

    def set_side(self, rpwm, lpwm, speed, invert=False):
        speed = max(-1.0, min(1.0, speed))

        if invert:
            speed = -speed

        if speed > 0.0:
            lpwm.value = 0.0
            rpwm.value = speed
        elif speed < 0.0:
            rpwm.value = 0.0
            lpwm.value = abs(speed)
        else:
            rpwm.value = 0.0
            lpwm.value = 0.0

    def stop_all(self):
        self.set_side(self.left_rpwm, self.left_lpwm, 0.0, self.invert_left)
        self.set_side(self.right_rpwm, self.right_lpwm, 0.0, self.invert_right)

    def cmd_vel_callback(self, message):
        self.last_command_time = time.monotonic()

        linear_x = float(message.linear.x)
        angular_z = float(message.angular.z)

        with self.lock:
            front_cm = self.front_distance_cm
            rear_cm = self.rear_distance_cm

        if linear_x > 0.0 and front_cm < self.safety_distance_cm:
            self.get_logger().warn(
                f'FORWARD BLOCKED: front obstacle {front_cm:.1f} cm'
            )
            self.stop_all()
            return

        if linear_x < 0.0 and rear_cm < self.safety_distance_cm:
            self.get_logger().warn(
                f'REVERSE BLOCKED: rear obstacle {rear_cm:.1f} cm'
            )
            self.stop_all()
            return

        left_speed = (
            linear_x -
            angular_z * self.wheel_base / 2.0
        ) / self.max_linear_speed

        right_speed = (
            linear_x +
            angular_z * self.wheel_base / 2.0
        ) / self.max_linear_speed

        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        self.set_side(
            self.left_rpwm,
            self.left_lpwm,
            left_speed,
            self.invert_left
        )

        self.set_side(
            self.right_rpwm,
            self.right_lpwm,
            right_speed,
            self.invert_right
        )

        self.get_logger().info(
            f'CMD x={linear_x:.2f}, z={angular_z:.2f}, '
            f'L={left_speed:.2f}, R={right_speed:.2f}'
        )

    def watchdog(self):
        if time.monotonic() - self.last_command_time > 0.7:
            self.stop_all()

    def destroy_node(self):
        self.stop_all()

        for device in [
            self.left_rpwm,
            self.left_lpwm,
            self.left_ren,
            self.left_len,
            self.right_rpwm,
            self.right_lpwm,
            self.right_ren,
            self.right_len,
        ]:
            device.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
