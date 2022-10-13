from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pyfirmata import Arduino, util

try:
    board = Arduino("/dev/ttyACM0")
except:
    board = Arduino("/dev/ttyACM1")
it = util.Iterator(board)
it.start()
analog_0 = board.get_pin("a:0:i")
analog_1 = board.get_pin("a:1:i")
analog_2 = board.get_pin("a:2:i")


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.get_logger().info("Controller Node Started!")
        # self.ser = Serial('/dev/ttyACM0', 9600, timeout=5.0)
        # self.serialio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        self.get_logger().warn("Controller Calibrating...")
        self.calibrate = analog_0.read(), analog_1.read()
        self.get_logger().info("Controller Calibration Complete!")
        self.controller_publisher = self.create_publisher(Twist, "/uav/controller", 2)
        self.create_timer(0.1, callback=self.sendPresses)

    def _map(self, x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def sendPresses(self):
        try:
            data = [
                -1 * round(analog_0.read() - self.calibrate[0], 4) * 200,
                -1 * round(analog_1.read() - self.calibrate[1], 4) * 200,
                -1 * round(analog_2.read() - self.calibrate[1], 4) * 200,
            ]
            for i in range(0, 3):
                if data[i] > 100:
                    data[i] = 100
                elif data[i] <= -100:
                    data[i] = -100
                if data[i] >= -2 and data[i] <= 2:
                    data[i] = 0

            data[0] = self._map(data[0], -100, 100, 0, 200) / 20

            data = [round(data[0], 1), round(data[1], 1), round(data[2], 1)]
            msg = Twist()
            msg.linear.x = float(data[0])
            msg.linear.y = float(data[1])
            msg.linear.z = float(data[0])
            self.controller_publisher.publish(msg)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
