import rclpy
from rclpy.node import Node
import math
from haversine import haversine, Unit
from math import sin, cos, atan2, pi, sqrt, asin
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

latd, logd = 26.841796177986573, 75.56305479755204


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.heading_data = 0
        self.create_subscription(
            Vector3, "/uav/gps/latlongheight", callback=self.heading, qos_profile=1
        )
        self.create_subscription(
            Float64, "/uav/compass_pos", callback=self.compass, qos_profile=1
        )
        self.bearing_publisher = self.create_publisher(
            Float64, "/uav/compass_pos/bearing", qos_profile=1
        )
        self.distance_publisher = self.create_publisher(
            Float64, "/uav/mission/distance", qos_profile=1
        )
        self.steering_publisher = self.create_publisher(Twist, "/uav/controller", 1)
        self.get_logger().info("Started Mission Module!")

    def heading(self, coords: Vector3):
        msg = Float64()
        latc = coords.x
        logc = coords.y
        deltalog = logd - logc
        deltalat = latd - latc
        x = cos(latd) * sin(deltalog)
        y = (cos(latc) * sin(latd)) - (sin(latc) * cos(latd) * cos(deltalog))
        bearing = atan2(x, y)
        if bearing < 0.0:
            bearing += 2 * pi
        bearing = math.degrees(bearing)
        msg.data = bearing
        self.bearing_publisher.publish(msg)
        d = haversine((latc, logc), (latd, logd), unit=Unit.METERS)
        msg.data = d
        self.distance_publisher.publish(msg)
        self.get_logger().info(
            f"Bearing: {bearing}° | Heading: {self.heading_data}° | Distance: {d}m"
        )
        self.steering(bearing, d, latc, logc)

    def compass(self, heading: Float64):
        self.heading_data = heading.data

    def steering(self, bearing, distance, latc, logc):
        headingError = bearing - self.heading_data
        if headingError < -180:
            headingError += 360
        if headingError > 180:
            headingError -= 360
        msg = Twist()
        if distance > 1:
            speed = 100
        else:
            speed = 20
        offset = 10
        if headingError <= offset and headingError >= -1 * offset and distance > 0.5:
            msg.linear.x = float(speed)
            msg.angular.z = 0.0
        elif headingError < -1 * offset and distance > 0.5:
            msg.linear.x = float(speed - 40)
            msg.angular.z = float(speed)
        elif headingError > offset and distance > 0.5:
            msg.linear.x = float(speed - 40)
            msg.angular.z = -1 * float(speed)
        else:
            msg.linear.x = msg.angular.z = 0.0
        self.get_logger().info(f"Delta Theta {headingError}° | Distance: {distance}m")
        self.steering_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
