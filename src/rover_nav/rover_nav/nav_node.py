import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool, Float32MultiArray

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        self.declare_parameter('declination_deg', 0.0)   # magnetic vs true north offset
        self.declare_parameter('goal_tolerance_m', 1.0)  # stop distance
        self.declare_parameter('turn_rate', 0.8)
        self.declare_parameter('drive_speed', 0.35)
        self.declare_parameter('angle_tolerance_deg', 20.0)

        self.declination = math.radians(float(self.get_parameter('declination_deg').value))
        self.goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.angle_tol = float(self.get_parameter('angle_tolerance_deg').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(String, '/nav/status', 10)

        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_subscription(Float32, '/imu/heading', self.on_heading, 10)
        self.create_subscription(Float32MultiArray, '/nav/goal', self.on_goal, 10)
        self.create_subscription(Bool, '/nav/enable', self.on_enable, 10)

        self.lat = None
        self.lon = None
        self.heading = None
        self.goal_lat = None
        self.goal_lon = None
        self.enabled = False
        self.state = 'IDLE'

        self.timer = self.create_timer(0.2, self.control_loop)  # 5 Hz
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info('Nav node ready. Send goal via /nav/goal and enable via /nav/enable')

    def on_fix(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def on_heading(self, msg):
        self.heading = msg.data

    def on_goal(self, msg):
        if len(msg.data) >= 2:
            self.goal_lat = msg.data[0]
            self.goal_lon = msg.data[1]
            self.get_logger().info(f'New goal: {self.goal_lat:.6f}, {self.goal_lon:.6f}')

    def on_enable(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.publish_cmd(0.0, 0.0)
            self.state = 'STOPPED'

    def distance_to_goal(self):
        R = 6371000.0
        dlat = math.radians(self.goal_lat - self.lat)
        dlon = math.radians(self.goal_lon - self.lon)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(self.lat)) * math.cos(math.radians(self.goal_lat)) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def bearing_to_goal(self):
        dlon = math.radians(self.goal_lon - self.lon)
        y = math.sin(dlon) * math.cos(math.radians(self.goal_lat))
        x = math.cos(math.radians(self.lat)) * math.sin(math.radians(self.goal_lat)) - \
            math.sin(math.radians(self.lat)) * math.cos(math.radians(self.goal_lat)) * math.cos(dlon)
        return math.degrees(math.atan2(y, x)) % 360.0

    def publish_cmd(self, linear, angular):
        t = Twist()
        t.linear.x = -angular
        t.angular.z = -linear
        self.cmd_pub.publish(t)

    def control_loop(self):
        if not self.enabled:
            if self.state not in ('IDLE', 'STOPPED'):
                self.state = 'STOPPED'
            return
        if self.goal_lat is None or self.lat is None or self.heading is None:
            self.state = 'WAITING_SENSORS'
            return

        dist = self.distance_to_goal()
        if dist < self.goal_tol:
            self.publish_cmd(0.0, 0.0)
            self.enabled = False
            self.state = 'REACHED'
            return

        bearing = self.bearing_to_goal()
        imu_true = (self.heading + math.degrees(self.declination)) % 360.0
        err = (bearing - imu_true + 540.0) % 360.0 - 180.0

        if abs(err) > self.angle_tol:
            angular = -self.turn_rate if err > 0 else self.turn_rate
            self.publish_cmd(0.0, angular)
            self.state = 'TURNING'
        else:
            angular = max(-0.4, min(0.4, -err * 0.02))
            speed = min(self.drive_speed, self.drive_speed * dist / 3.0 + 0.1)
            self.publish_cmd(speed, angular)
            self.state = 'DRIVING'

    def publish_status(self):
        msg = String()
        if self.goal_lat is not None and self.lat is not None:
            dist = self.distance_to_goal()
            bearing = self.bearing_to_goal()
            msg.data = f'{self.state} | Goal: {dist:.1f} m away | Bearing: {bearing:.0f} deg'
        else:
            msg.data = f'{self.state} | No goal set'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
