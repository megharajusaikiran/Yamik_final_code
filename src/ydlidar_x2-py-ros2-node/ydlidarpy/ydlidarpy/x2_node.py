import math
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class X2Node(Node):
    def __init__(self):
        super().__init__('ydlidar_x2')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_hz', 7.0)
        self.declare_parameter('range_min', 0.10)
        self.declare_parameter('range_max', 8.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.frame_id = self.get_parameter('frame_id').value
        self.scan_hz = float(self.get_parameter('scan_hz').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.ser = serial.Serial(port, baud, timeout=0.1)

        time.sleep(0.5)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.scan_hz,
            self.publish_scan
        )

        self.ranges = [float('inf')] * 360

        self.get_logger().info(
            f'YDLidar X2 node started on {port}'
        )

    def publish_scan(self):
        self.read_packets()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        scan = LaserScan()
        scan.header = header
        scan.angle_min = 0.0
        scan.angle_max = math.radians(359.0)
        scan.angle_increment = math.radians(1.0)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = self.ranges
        scan.intensities = []

        self.scan_pub.publish(scan)

    def read_packets(self):
        if self.ser.in_waiting < 9:
            return

        header = self.ser.read(2)

        if len(header) != 2:
            return

        # X2 uses 0xAA 0x55 as header (not 0x55 0xAA)
        if header[0] != 0xAA or header[1] != 0x55:
            self.ser.reset_input_buffer()
            return

        length_byte = self.ser.read(1)

        if len(length_byte) != 1:
            return

        length = length_byte[0]

        if length < 5:
            return

        data = self.ser.read(length)

        if len(data) != length:
            return

        number_of_points = (length - 2) // 3

        for i in range(number_of_points):
            index = i % 360

            low = data[2 + i * 3]
            high = data[2 + i * 3 + 1]

            distance_mm = (high << 8) | low
            distance_m = distance_mm / 1000.0

            if self.range_min <= distance_m <= self.range_max:
                self.ranges[index] = distance_m
            else:
                self.ranges[index] = float('inf')

def main(args=None):
    rclpy.init(args=args)

    node = X2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
