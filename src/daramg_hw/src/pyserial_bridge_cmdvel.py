#!/usr/bin/env python3
import struct, serial, argparse, threading, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def make_pkt(pkt_type: int, payload: bytes) -> bytes:
    # frame: 0xAA,0x55, type(1), len(1), payload..., crc8(1)
    hdr = bytes([0xAA, 0x55, pkt_type & 0xFF, len(payload) & 0xFF])
    crc = 0
    for b in (hdr[2:] + payload):
        crc ^= b
    return hdr + payload + bytes([crc & 0xFF])

class CmdVelBridge(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('serial_bridge_cmdvel')
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.get_logger().info(f'Opened {port} @ {baud} for /cmd_vel')
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)

        # Reader thread: drain & print ASCII debug lines from ESP32
        self._stop = False
        self.reader = threading.Thread(target=self._read_serial, daemon=True)
        self.reader.start()

    def _read_serial(self):
        buf = b""
        while not self._stop:
            try:
                b = self.ser.read(128)
                if not b:
                    continue
                buf += b
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    # Print ESP32 debug as-is
                    try:
                        print(f"[ESP32] {line.decode(errors='replace')}")
                        sys.stdout.flush()
                    except Exception:
                        pass
            except Exception as e:
                print(f"[bridge] serial read error: {e}", file=sys.stderr)
                break

    def on_twist(self, msg: Twist):
        # payload: float32 v, float32 w (little-endian)
        payload = struct.pack('<ff', float(msg.linear.x), float(msg.angular.z))
        self.ser.write(make_pkt(0x01, payload))

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    parser = argparse.ArgumentParser(description="ROS2 /cmd_vel → Serial bridge (with debug reader)")
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
