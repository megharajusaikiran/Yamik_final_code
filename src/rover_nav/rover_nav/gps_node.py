import threading
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(0.5)
        self.ser.reset_input_buffer()

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.status_pub = self.create_publisher(String, '/gps/status', 10)

        self.lat = None
        self.lon = None
        self.alt = float('nan')
        self.sats = 0
        self.quality = 0
        self.speed = 0.0

        self.get_logger().info(f'GPS node started on {port}')

        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                time.sleep(1.0)
                continue
            if not line.startswith('$'):
                continue
            try:
                self.parse(line)
            except Exception as e:
                self.get_logger().error(f'Parse error: {e}')

    def parse(self, line):
        fields = line.split(',')
        talker = fields[0]

        if talker[3:] == 'GGA' and len(fields) >= 12:
            try:
                self.quality = int(fields[6])
                self.sats = int(fields[7])
                self.alt = float(fields[9]) if fields[9] else float('nan')
                if self.quality > 0 and fields[2] and fields[4]:
                    self.lat = self.to_deg(fields[2], fields[3])
                    self.lon = self.to_deg(fields[4], fields[5])
            except ValueError:
                pass

        elif talker[3:] == 'RMC' and len(fields) >= 9:
            try:
                if fields[2] == 'A' and fields[3] and fields[5]:
                    self.lat = self.to_deg(fields[3], fields[4])
                    self.lon = self.to_deg(fields[5], fields[6])
                    self.speed = float(fields[7]) * 0.51444 if fields[7] else 0.0
                    if self.quality == 0:
                        self.quality = 1
            except ValueError:
                pass

        if self.lat is not None and self.lon is not None and self.quality > 0:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'
            fix.latitude = self.lat
            fix.longitude = self.lon
            fix.altitude = self.alt
            # 0=no fix, 1=GPS fix, 2=SBAS/DGPS fix
            fix.status.status = 1 if self.quality == 1 else 2
            # 1=GPS service
            fix.status.service = 1
            fix.position_covariance_type = 0  # unknown
            self.fix_pub.publish(fix)

    def to_deg(self, value, hemisphere):
        raw = float(value)
        deg = int(raw / 100.0)
        minutes = raw - deg * 100.0
        result = deg + minutes / 60.0
        if hemisphere in ('S', 'W'):
            result = -result
        return result

    def publish_status(self):
        if self.quality == 0:
            state = 'NO FIX'
        elif self.quality == 1:
            state = 'GPS FIX'
        else:
            state = 'DGPS FIX'
        msg = String()
        msg.data = f'{state} | Sats: {self.sats} | Speed: {self.speed:.2f} m/s'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
