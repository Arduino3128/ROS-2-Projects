import json
import rclpy
from rclpy.node import Node

# import board
# import adafruit_lsm303dlh_mag
# import adafruit_lsm303_accel
import io
import serial
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np

# i2c = board.I2C()
# mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
# accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
# m_min = (-32767, -32767, -32767)
# m_max = (+32767, +32767, +32767)


class Compass(Node):
    def __init__(self):
        super().__init__("compass_handler")
        self.start_timer()

    """"def compassNormalize(self,a):
        mag = math.sqrt(np.dot(a, a))
        a[0]/=mag
        a[1]/=mag
        a[2]/=mag   
        return a     

    def compassHeading(self):
        #mag.mag_gain()
        temp_m = list(mag._raw_magnetic)
        print(temp_m)
        #temp_m[1],temp_m[2] = temp_m[2],temp_m[1]
        a = accel._raw_acceleration
        temp_m[0] -= (m_min[0] + m_max[0]) / 2
        temp_m[1] -= (m_min[1] + m_max[1]) / 2  
        temp_m[2] -= (m_min[2] + m_max[2]) / 2
        E=np.cross(temp_m, a)
        E=self.compassNormalize(E)
        N=np.cross(a, E)
        N=self.compassNormalize(N)        
        heading_64 = Float64()
        heading = math.atan(np.dot(E,self.from_)/np.dot(N,self.from_))#math.degrees()
        heading = math.degrees(heading)
        print(heading, math.degrees(math.atan(temp_m[1]/temp_m[0])))
        if (heading < 0):
            heading = 360 + heading
        heading_64.data = heading
        #print(heading)
        self.compass_publisher.publish(heading_64)"""

    def start_timer(self):
        self.connection_timer = self.create_timer(1, callback=self.init_connection)

    def init_connection(self):
        try:
            self.get_logger().info("Connecting to Compass Module....")
            self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=5.0)
            self.connection_timer.cancel()
            self.get_logger().info("Connected to Compass Module!")
            self.publisher_ = self.create_publisher(Float64, "/uav/compass_pos", 1)
            self.compass_timer = self.create_timer(0.05, callback=self.compass_process)
            self.get_logger().info("Started Compass Handler Node!")
        except Exception as e:
            self.get_logger().fatal("Device error: {}".format(e))

    def compass_process(self):
        try:
            msg = Float64()
            line = float(self.ser.readline().decode())
            # declinationAngle = math.degrees((1 + (4.0 / 60.0)))
            # line += declinationAngle
            msg.data = line
            self.publisher_.publish(msg)

        except serial.SerialException as e:
            self.get_logger().fatal("Device error: {}".format(e))
            self.compass_timer.cancel()
            self.ser.close()
            self.start_timer()
        except Exception as e:
            self.get_logger().error("Unknown error: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    node = Compass()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
