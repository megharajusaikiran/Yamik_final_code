import glob
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicTwoPi5Node(Node):
    def __init__(self):
        super().__init__('ultrasonic_two_pi5_node')

        self.baudrate = 9600

        self.front_serial = self.open_serial(
            '/dev/ttyAMA0',
            'front'
        )

        self.rear_port = self.find_rear_port()

        self.rear_serial = None

        if self.rear_port is not None:
            self.rear_serial = self.open_serial(
                self.rear_port,
                'rear'
            )
        else:
            self.get_logger().error(
                'Rear USB-TTL adapter not found. Plug it into the Pi USB port.'
            )

        self.front_pub = self.create_publisher(
            Range,
            '/ultrasonic/front',
            10
        )

        self.rear_pub = self.create_publisher(
            Range,
            '/ultrasonic/rear',
            10
        )

        self.front_buffer = bytearray()
        self.rear_buffer = bytearray()

        self.create_timer(0.01, self.poll)

    def find_rear_port(self):
        ports = []

        for pattern in [
            '/dev/serial/by-id/*',
            '/dev/ttyUSB*',
            '/dev/ttyACM*'
        ]:
            ports.extend(sorted(glob.glob(pattern)))

        unique_ports = []

        for port in ports:
            if port not in unique_ports:
                unique_ports.append(port)

        self.get_logger().info(
            f'USB serial devices found: {unique_ports}'
        )

        if len(unique_ports) == 0:
            return None

        return unique_ports[0]

    def open_serial(self, port, name):
        try:
            serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )

            self.get_logger().info(
                f'{name} sensor opened on {port}'
            )

            return serial_port

        except Exception as error:
            self.get_logger().error(
                f'{name} sensor failed on {port}: {error}'
            )

            return None

    def parse_frames(self, buffer):
        distances = []

        while len(buffer) >= 4:
            try:
                start = buffer.index(0xFF)
            except ValueError:
                buffer.clear()
                break

            if start > 0:
                del buffer[:start]

            if len(buffer) < 4:
                break

            high = buffer[1]
            low = buffer[2]
            checksum = buffer[3]

            expected_checksum = (
                0xFF + high + low
            ) & 0xFF

            if checksum != expected_checksum:
                del buffer[0]
                continue

            distance_mm = (high << 8) | low
            distance_m = distance_mm / 1000.0

            if 0.03 <= distance_m <= 4.5:
                distances.append(distance_m)

            del buffer[:4]

        return distances

    def make_range(self, frame_id, distance):
        message = Range()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.header.frame_id = frame_id
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = 0.52
        message.min_range = 0.03
        message.max_range = 4.5
        message.range = distance

        return message

    def read_sensor(self, serial_port, buffer, publisher, frame_id):
        if serial_port is None:
            return

        try:
            waiting = serial_port.in_waiting

            if waiting > 0:
                buffer.extend(serial_port.read(waiting))

            distances = self.parse_frames(buffer)

            for distance in distances:
                publisher.publish(
                    self.make_range(frame_id, distance)
                )

        except Exception as error:
            self.get_logger().error(
                f'{frame_id} sensor read error: {error}'
            )

    def poll(self):
        self.read_sensor(
            self.front_serial,
            self.front_buffer,
            self.front_pub,
            'front'
        )

        self.read_sensor(
            self.rear_serial,
            self.rear_buffer,
            self.rear_pub,
            'rear'
        )

    def destroy_node(self):
        if self.front_serial is not None:
            self.front_serial.close()

        if self.rear_serial is not None:
            self.rear_serial.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = UltrasonicTwoPi5Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
