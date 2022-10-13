import os
import time
import csv
from threading import Thread
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int16


class Server(Node):
    def __init__(self):
        super().__init__("server_node")
        os.chdir("/home/atticus/ros2_ws/src/uav_controller/uav_controller/UAV")
        self.filename = "./data/" + str(time.time()) + ".csv"
        with open(self.filename, "a") as file:
            wrt = csv.writer(file)
            wrt.writerow(
                [
                    "timestamp",
                    "latitude",
                    "longitude",
                    "heading",
                    "bearing",
                    "speed",
                    "satnum",
                    "distance",
                ]
            )
        self.heading = self.bearing = self.latitude = self.longitude = 0
        self.height = self.speed = self.satnum = self.distance = 0
        self.create_subscription(
            Float64, "/uav/compass_pos", callback=self.compass_callback, qos_profile=1
        )
        self.create_subscription(
            Vector3, "/uav/gps/latlongheight", callback=self.gps_callback, qos_profile=1
        )

        self.create_subscription(
            Float64,
            "/uav/compass_pos/bearing",
            callback=self.bearing_callback,
            qos_profile=1,
        )

        self.create_subscription(
            Float64,
            "/uav/mission/distance",
            callback=self.distance_callback,
            qos_profile=1,
        )
        self.create_subscription(
            Float64,
            "/uav/gps/speed",
            callback=self.speed_callback,
            qos_profile=1,
        )
        self.create_subscription(
            Int16,
            "/uav/gps/satnum",
            callback=self.satnum_callback,
            qos_profile=1,
        )
        self.create_timer(0.1, self.writer)

    def writer(self):
        response = {
            "timestamp": time.time(),
            "latitude": self.latitude,
            "longitude": self.longitude,
            "heading": self.heading,
            "bearing": self.bearing,
            "speed": self.speed,
            "satnum": self.satnum,
            "distance": self.distance,
        }
        with open(self.filename, "a") as file:
            wrt = csv.writer(file)
            wrt.writerow([x for x in response.values()])
        with open("data/data.json", "w") as file:
            json.dump(response, file)

    def compass_callback(self, data: Float64):
        self.heading = data.data

    def speed_callback(self, data: Float64):
        self.speed = data.data

    def distance_callback(self, data: Float64):
        self.distance = data.data

    def satnum_callback(self, data: Int16):
        self.satnum = data.data

    def gps_callback(self, data: Vector3):
        self.latitude = data.x
        self.longitude = data.y
        self.height = data.z
        print(
            f"Latitude {self.latitude}, Longitude {self.longitude}, Height {self.height}"
        )

    def bearing_callback(self, data: Float64):
        self.bearing = data.data


def main(args=None):
    rclpy.init(args=args)
    node = Server()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
