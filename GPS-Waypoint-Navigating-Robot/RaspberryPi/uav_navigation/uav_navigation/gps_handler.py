import rclpy
from rclpy.node import Node
import io
import pynmea2
import serial
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int16

gps_quality = 0


class GPSHandler(Node):
    def __init__(self):
        super().__init__(node_name="gps_handler")
        self.start_timer()

    def start_timer(self):
        self.connection_timer = self.create_timer(1, callback=self.init_connection)

    def init_connection(self):
        try:
            self.get_logger().info("Connecting to GPS Module....")
            self.ser = serial.Serial("/dev/serial0", 9600, timeout=5.0)
            self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
            self.connection_timer.cancel()
            self.get_logger().info("Connected to GPS Module!")
            self.publisher_ = self.create_publisher(
                Vector3, "/uav/gps/latlongheight", 1
            )
            self.speed_publisher = self.create_publisher(Float64, "/uav/gps/speed", 1)
            self.satnum_publisher = self.create_publisher(Int16, "/uav/gps/satnum", 1)
            self.gps_timer = self.create_timer(0.1, callback=self.gps_process)
            self.get_logger().info("Started GPS Handler Node!")
        except Exception as e:
            self.get_logger().fatal("Device error: {}".format(e))

    def gps_process(self):
        try:
            global gps_quality
            line = self.sio.readline()
            msg_data = pynmea2.parse(line)
            if msg_data.sentence_type == "GGA":
                msg = Vector3()
                int_msg = Int16()
                gps_quality = msg_data.gps_qual
                if msg_data.gps_qual != 0:
                    msg.x = float(msg_data.latitude)
                    msg.y = float(msg_data.longitude)
                    msg.z = float(msg_data.altitude)
                    int_msg.data = int(msg_data.num_sats)
                else:
                    self.get_logger().warn("No GPS Fix")
                    msg.x = msg.y = msg.z = 0.0
                    int_msg.data = 0
                self.publisher_.publish(msg)
                self.satnum_publisher.publish(int_msg)
            elif msg_data.sentence_type == "VTG" and gps_quality != 0:
                msg = Float64()
                if gps_quality != 0:
                    msg.data = float(msg_data.spd_over_grnd_kmph)
                else:
                    msg.data = 0.0
                self.speed_publisher.publish(msg)

        except serial.SerialException as e:
            self.get_logger().fatal("Device error: {}".format(e))
            self.gps_timer.cancel()
            self.ser.close()
            self.start_timer()
        except pynmea2.ParseError as e:
            pass
        except Exception as e:
            self.get_logger().error("Unknown error: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    node = GPSHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
