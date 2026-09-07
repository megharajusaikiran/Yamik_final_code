import os
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from flask import Flask, send_from_directory
from flask_socketio import SocketIO

class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')

        self.speed = 0.35
        self.turn = 0.8

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest = {
            'front_left': None,
            'front_right': None,
            'back_left': None,
            'back_right': None,
        }

        for name in self.latest:
            self.create_subscription(
                Range,
                f'/ultrasonic/{name}',
                lambda msg, n=name: self.range_cb(n, msg),
                10
            )

        base = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'web')

        app = Flask(__name__, static_folder=base, static_url_path='')
        socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')
        self.socketio = socketio

        @app.route('/')
        def index():
            return send_from_directory(base, 'index.html')

        @socketio.on('connect')
        def connected():
            socketio.emit('status', {'connected': True})
            socketio.emit('ultrasonic', self.latest)

        @socketio.on('command')
        def command(data):
            action = data.get('action', 'stop') if isinstance(data, dict) else 'stop'
            msg = Twist()
            if action == 'forward':
                msg.linear.x = self.speed
            elif action == 'back':
                msg.linear.x = -self.speed
            elif action == 'left':
                msg.angular.z = self.turn
            elif action == 'right':
                msg.angular.z = -self.turn
            self.pub.publish(msg)

        self.thread = threading.Thread(
            target=lambda: socketio.run(
                app,
                host='0.0.0.0',
                port=8080,
                allow_unsafe_werkzeug=True
            ),
            daemon=True
        )
        self.thread.start()

        self.get_logger().info('UI: http://<PI-IP>:8080')

    def range_cb(self, name, msg):
        self.latest[name] = round(msg.range * 100.0, 1)
        self.socketio.emit('ultrasonic', self.latest)

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
