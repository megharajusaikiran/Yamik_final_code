import glob
import os
import time

import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import String


class RoverSensorsGpsNode(Node):
    def __init__(self):
        super().__init__('rover_sensors_gps_node')

        self.baudrate = 9600

        self.front_serial = None
        self.rear_serial = None
        self.gps_serial = None

        self.front_buffer = bytearray()
        self.rear_buffer = bytearray()

        self.last_satellites = 0

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

        self.gps_fix_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )

        self.gps_status_pub = self.create_publisher(
            String,
            '/gps/status',
            10
        )

        self.open_front_uart()

        self.create_timer(
            2.0,
            self.discover_usb_devices
        )

        self.create_timer(
            0.02,
            self.poll
        )

        self.publish_gps_status(
            'GPS waiting for USB-TTL adapter and satellite fix.'
        )

    def open_front_uart(self):
        try:
            self.front_serial = serial.Serial(
                '/dev/ttyAMA0',
                self.baudrate,
                timeout=0.01
            )

            self.get_logger().info(
                'Front ultrasonic opened on /dev/ttyAMA0'
            )

        except Exception as error:
            self.get_logger().error(
                f'Front UART error: {error}'
            )

    def usb_devices(self):
        devices = []

        for pattern in [
            '/dev/serial/by-id/*',
            '/dev/ttyUSB*',
            '/dev/ttyACM*'
        ]:
            for device in sorted(glob.glob(pattern)):
                real_device = os.path.realpath(device)

                if real_device not in devices:
                    devices.append(real_device)

        return devices

    def classify_device(self, device):
        try:
            port = serial.Serial(
                device,
                self.baudrate,
                timeout=0.15
            )

            port.reset_input_buffer()

            binary_data = bytearray()
            ascii_data = ''

            deadline = time.time() + 2.0

            while time.time() < deadline:
                chunk = port.read(
                    port.in_waiting or 1
                )

                if not chunk:
                    continue

                binary_data.extend(chunk)

                ascii_data += chunk.decode(
                    'ascii',
                    errors='ignore'
                )

                if any(
                    sentence in ascii_data
                    for sentence in [
                        '$GPGGA',
                        '$GNGGA',
                        '$GPRMC',
                        '$GNRMC'
                    ]
                ):
                    return 'gps', port

                if 0xFF in binary_data and len(binary_data) >= 4:
                    return 'rear_ultrasonic', port

            port.close()

        except Exception:
            pass

        return None, None

    def discover_usb_devices(self):
        devices = self.usb_devices()

        for device in devices:
            already_used = []

            if self.rear_serial is not None:
                already_used.append(
                    os.path.realpath(self.rear_serial.port)
                )

            if self.gps_serial is not None:
                already_used.append(
                    os.path.realpath(self.gps_serial.port)
                )

            if device in already_used:
                continue

            kind, port = self.classify_device(device)

            if kind == 'rear_ultrasonic':
                if self.rear_serial is None:
                    self.rear_serial = port

                    self.get_logger().info(
                        f'Rear ultrasonic active on {device}'
                    )
                else:
                    port.close()

            elif kind == 'gps':
                if self.gps_serial is None:
                    self.gps_serial = port

                    self.get_logger().info(
                        f'GPS active on {device}'
                    )

                    self.publish_gps_status(
                        f'GPS connected on {device}. '
                        'Waiting for satellite fix.'
                    )
                else:
                    port.close()

    def parse_ultrasonic(self, buffer):
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

            expected = (
                0xFF + high + low
            ) & 0xFF

            if checksum != expected:
                del buffer[0]
                continue

            distance_m = (
                (high << 8) | low
            ) / 1000.0

            if 0.03 <= distance_m <= 4.5:
                distances.append(distance_m)

            del buffer[:4]

        return distances

    def range_message(self, frame_id, distance):
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

    def read_ultrasonic(
        self,
        port,
        buffer,
        publisher,
        frame_id
    ):
        if port is None:
            return

        try:
            count = port.in_waiting

            if count > 0:
                buffer.extend(port.read(count))

            for distance in self.parse_ultrasonic(buffer):
                publisher.publish(
                    self.range_message(frame_id, distance)
                )

        except Exception as error:
            self.get_logger().warn(
                f'{frame_id} ultrasonic error: {error}'
            )

    def nmea_lat_lon(self, value, hemisphere):
        if not value or not hemisphere:
            return None

        try:
            raw = float(value)
            degrees = int(raw / 100)
            minutes = raw - degrees * 100

            decimal = degrees + minutes / 60.0

            if hemisphere in ['S', 'W']:
                decimal = -decimal

            return decimal

        except Exception:
            return None

    def process_nmea_line(self, line):
        if not line.startswith('$'):
            return

        fields = line.split('*')[0].split(',')

        if fields[0] in ['$GPGGA', '$GNGGA']:
            self.process_gga(fields)

        elif fields[0] in ['$GPRMC', '$GNRMC']:
            self.process_rmc(fields)

    def process_gga(self, fields):
        if len(fields) < 10:
            return

        latitude = self.nmea_lat_lon(
            fields[2],
            fields[3]
        )

        longitude = self.nmea_lat_lon(
            fields[4],
            fields[5]
        )

        try:
            quality = int(fields[6] or 0)
        except Exception:
            quality = 0

        try:
            satellites = int(fields[7] or 0)
        except Exception:
            satellites = 0

        try:
            altitude = float(fields[9] or 0.0)
        except Exception:
            altitude = 0.0

        self.last_satellites = satellites

        if quality == 0 or latitude is None or longitude is None:
            self.publish_gps_status(
                f'GPS no fix | Satellites: {satellites}'
            )
            return

        self.publish_fix(
            latitude,
            longitude,
            altitude,
            satellites
        )

    def process_rmc(self, fields):
        if len(fields) < 7:
            return

        if fields[2] != 'A':
            return

        latitude = self.nmea_lat_lon(
            fields[3],
            fields[4]
        )

        longitude = self.nmea_lat_lon(
            fields[5],
            fields[6]
        )

        if latitude is None or longitude is None:
            return

        self.publish_fix(
            latitude,
            longitude,
            0.0,
            self.last_satellites
        )

    def read_gps(self):
        if self.gps_serial is None:
            return

        try:
            while self.gps_serial.in_waiting > 0:
                line = self.gps_serial.readline().decode(
                    'ascii',
                    errors='ignore'
                ).strip()

                self.process_nmea_line(line)

        except Exception as error:
            self.get_logger().warn(
                f'GPS serial error: {error}'
            )

            try:
                self.gps_serial.close()
            except Exception:
                pass

            self.gps_serial = None

    def publish_fix(
        self,
        latitude,
        longitude,
        altitude,
        satellites
    ):
        message = NavSatFix()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.header.frame_id = 'gps_link'
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = altitude

        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.gps_fix_pub.publish(message)

        self.publish_gps_status(
            f'GPS FIX | Lat: {latitude:.7f} | '
            f'Lon: {longitude:.7f} | '
            f'Altitude: {altitude:.1f} m | '
            f'Satellites: {satellites}'
        )

    def publish_gps_status(self, text):
        message = String()
        message.data = text
        self.gps_status_pub.publish(message)

    def poll(self):
        self.read_ultrasonic(
            self.front_serial,
            self.front_buffer,
            self.front_pub,
            'front'
        )

        self.read_ultrasonic(
            self.rear_serial,
            self.rear_buffer,
            self.rear_pub,
            'rear'
        )

        self.read_gps()

    def destroy_node(self):
        for port in [
            self.front_serial,
            self.rear_serial,
            self.gps_serial
        ]:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RoverSensorsGpsNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
