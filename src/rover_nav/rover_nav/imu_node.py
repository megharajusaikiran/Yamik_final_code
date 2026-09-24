import json
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from smbus2 import SMBus

BNO055_ADDR = 0x28
REG_OPR_MODE = 0x3D
REG_SYS_TRIGGER = 0x3F
REG_CALIB_STAT = 0x35
REG_CALIB_DATA = 0x55

MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

CALIB_FILE = os.path.expanduser('~/bno055_calib.json')
NORTH_FILE = os.path.expanduser('~/bno055_north.json')

class ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_node')
        self.declare_parameter('address', 0x28)
        self.addr = int(self.get_parameter('address').value)

        self.bus = SMBus(1)
        self.event_pub = self.create_publisher(String, '/imu/calib_event', 10)

        self.bus.write_byte_data(self.addr, REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.05)

        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE) as f:
                    data = [int(x) for x in json.load(f)]
                self.bus.write_i2c_block_data(self.addr, REG_CALIB_DATA, data)
                self.emit('CALIB_LOADED: saved calibration restored')
            except Exception as e:
                self.emit(f'CALIB_LOAD_FAILED: {e}')

        self.bus.write_byte_data(self.addr, REG_SYS_TRIGGER, 0x00)
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, REG_OPR_MODE, MODE_NDOF)
        time.sleep(0.2)

        self.north_offset = 0.0
        if os.path.exists(NORTH_FILE):
            try:
                with open(NORTH_FILE) as f:
                    self.north_offset = float(json.load(f)['offset'])
                self.emit(f'NORTH_LOADED: saved north offset {self.north_offset:.1f} deg restored')
            except Exception as e:
                self.emit(f'NORTH_LOAD_FAILED: {e}')

        self.raw_heading = 0.0

        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)
        self.calib_pub = self.create_publisher(String, '/imu/calib', 10)
        self.create_subscription(Bool, '/imu/save_calib', self.on_save_request, 10)
        self.create_subscription(Bool, '/imu/set_north', self.on_set_north, 10)

        self.timer = self.create_timer(0.05, self.tick)
        self.calib_counter = 0
        self.get_logger().info(f'BNO055 node started at 0x{self.addr:02x} in NDOF mode')

    def emit(self, text):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        self.get_logger().info(text)

    def on_save_request(self, msg):
        if not msg.data:
            return
        try:
            c = self.bus.read_byte_data(self.addr, REG_CALIB_STAT)
            if ((c >> 6) & 0x03) < 3:
                self.emit('CALIB_NOT_READY: calibrate first (flat + still, then figure-8) until all 4 values are 3')
                return
            data = self.bus.read_i2c_block_data(self.addr, REG_CALIB_DATA, 22)
            with open(CALIB_FILE, 'w') as f:
                json.dump(data, f)
            self.emit('CALIB_SAVED: stored, will auto-load on every boot')
        except Exception as e:
            self.emit(f'CALIB_SAVE_FAILED: {e}')

    def on_set_north(self, msg):
        if not msg.data:
            return
        self.north_offset = (-self.raw_heading) % 360.0
        try:
            with open(NORTH_FILE, 'w') as f:
                json.dump({'offset': self.north_offset}, f)
            self.emit(f'NORTH_SET: aligned with phone compass (offset {self.north_offset:.1f} deg saved)')
        except Exception as e:
            self.emit(f'NORTH_SAVE_FAILED: {e}')

    def read_signed16(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return data[0] | (data[1] << 8)

    def tick(self):
        try:
            raw = (self.read_signed16(0x1A) % 5760) / 16.0
            self.raw_heading = raw
            heading = (raw + self.north_offset) % 360.0

            roll_raw = self.read_signed16(0x1C)
            if roll_raw >= 32768:
                roll_raw -= 65536
            roll = roll_raw / 16.0
            pitch_raw = self.read_signed16(0x1E)
            if pitch_raw >= 32768:
                pitch_raw -= 65536
            pitch = pitch_raw / 16.0

            h = Float32()
            h.data = heading
            self.heading_pub.publish(h)

            self.calib_counter += 1
            if self.calib_counter >= 20:
                self.calib_counter = 0
                c = self.bus.read_byte_data(self.addr, REG_CALIB_STAT)
                sys_c = (c >> 6) & 0x03
                gyro_c = (c >> 4) & 0x03
                acc_c = (c >> 2) & 0x03
                mag_c = c & 0x03
                msg = String()
                msg.data = f'SYS:{sys_c} GYR:{gyro_c} ACC:{acc_c} MAG:{mag_c} | Roll:{roll:.1f} Pitch:{pitch:.1f}'
                self.calib_pub.publish(msg)
        except OSError as e:
            self.get_logger().error(f'I2C read failed: {e}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
