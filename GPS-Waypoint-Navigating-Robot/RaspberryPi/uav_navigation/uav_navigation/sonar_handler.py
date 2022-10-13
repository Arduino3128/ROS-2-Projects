from threading import Thread
import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node
from geometry_msgs.msg import Twist
SONAR_SERVO = 18
ECHO_PIN = 24
TRIG_PIN = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(SONAR_SERVO, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
EN1_PWM = GPIO.PWM(SONAR_SERVO, 100)

class ServoDriver(Node):
    def __init__(self):
        super().__init__("sonar_handler")
        self.sonar_publisher = self.create_publisher(Twist,"/uav/sonar",1)
        self.angle = 0
        self.current_angle = 0
        self.create_timer(0.1,self.distance)
        self.create_timer(0.5,self.sonarControl)

    def sonarControl(self):
        msg = Twist()
        self.current_angle+=5
        self.angle+=5
        if self.current_angle>180:
            self.angle = 360-self.angle
        elif self.angle==0:
            self.angle=0
            self.current_angle = 0
        print(f"ANGLE: {self.angle} | C_ANGLE: {self.current_angle}")
        self.setAngle(self.angle)
        msg.angular.z = float(self.angle)
        #msg.linear.x = self.distance()
        self.sonar_publisher.publish(msg)

    def setAngle(self,angle):
        duty = angle / 18 + 2
        GPIO.output(SONAR_SERVO, True)
        EN1_PWM.ChangeDutyCycle(duty)

    def distance(self):
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)
 
        StartTime = time.time()
        StopTime = time.time()
    
        while GPIO.input(ECHO_PIN) == 0:
            StartTime = time.time()
    
        while GPIO.input(ECHO_PIN) == 1:
            StopTime = time.time()
    
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2

        print(distance)
        return distance

def main(args = None):
    rclpy.init(args=args)
    node = ServoDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()