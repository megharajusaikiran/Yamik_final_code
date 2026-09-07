import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # REPLACE these four paths with your actual /dev/serial/by-id/... paths
        self.declare_parameter('front_left_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A-if00-port0')
        self.declare_parameter('front_right_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B-if00-port0')
        self.declare_parameter('back_left_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_C-if00-port0')
        self.declare_parameter('back_right_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_D-if00-port0')
        self.serials = {}
        self.pubs = {}

        for name in ['front_left', 'front_right', 'back_left', 'back_right']:
            port = str(self.get_parameter(name + '_port').value)
            try:
                self.serials[name] = serial.Serial(port, 9600, timeout=0.02)
                self.pubs[name] = self.create_publisher(Range, f'/ultrasonic/{name}', 10)
                self.get_logger().info(f'{name}: {port}')
            except Exception as exc:
                self.get_logger().error(f'{name} unavailable: {exc}')

        self.create_timer(0.02, self.poll)

    def read_frame(self, ser):
        data = ser.read(ser.in_waiting or 1)
        for byte in data:
            if byte == 0xFF:
                rest = ser.read(3)
                if len(rest) != 3:
                    return None
                high, low, checksum = rest
                if ((0xFF + high + low) & 0xFF) == checksum:
                    return ((high << 8) | low) / 1000.0
        return None

    def poll(self):
        for name, ser in self.serials.items():
            distance = self.read_frame(ser)
            if distance is None or distance < 0.03 or distance > 4.5:
                continue
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = name
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.52
            msg.min_range = 0.03
            msg.max_range = 4.5
            msg.range = distance
            self.pubs[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ser in node.serials.values():
            ser.close()
        node.destroy_node()
        rclpy.shutdown()
