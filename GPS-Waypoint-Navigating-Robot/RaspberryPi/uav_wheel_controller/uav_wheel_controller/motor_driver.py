import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

IN1A = 13
IN1B = 19
EN1 = 26
IN2A = 16
IN2B = 20
EN2 = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1A, GPIO.OUT)
GPIO.setup(IN1B, GPIO.OUT)
GPIO.setup(IN2A, GPIO.OUT)
GPIO.setup(IN2B, GPIO.OUT)
GPIO.setup(EN1, GPIO.OUT)
GPIO.setup(EN2, GPIO.OUT)
EN1_SPEED = GPIO.PWM(EN1, 100)
EN2_SPEED = GPIO.PWM(EN2, 100)

def OFF():
    EN1_SPEED.start(0)
    EN2_SPEED.start(0)
    GPIO.output(IN1A, False)
    GPIO.output(IN1B, False)
    GPIO.output(IN2A, False)
    GPIO.output(IN2B, False)
    

def FWD(speed1 = 100,speed2 = 100):
    try:
        speed1,speed2=abs(speed1),abs(speed2)
        EN1_SPEED.start(speed1)
        EN2_SPEED.start(speed2)
        GPIO.output(IN1A, True)
        GPIO.output(IN1B, False)
        GPIO.output(IN2A, True)
        GPIO.output(IN2B, False)
    except:
        print(speed1, speed2, sep=" | ")

def BCK(speed1 = 100,speed2 = 100):
    try:
        speed1,speed2=abs(speed1),abs(speed2)
        EN1_SPEED.start(speed1)
        EN2_SPEED.start(speed2)
        GPIO.output(IN1A, False)
        GPIO.output(IN1B, True)
        GPIO.output(IN2A, False)
        GPIO.output(IN2B, True)
    except:
        print(speed1, speed2, sep=" | ")

    

def CCW(speed1 = 100,speed2 = 100):
    speed1,speed2=abs(speed1),abs(speed2)
    EN1_SPEED.start(speed1)
    EN2_SPEED.start(speed2)
    GPIO.output(IN1A, True)
    GPIO.output(IN1B, False)
    GPIO.output(IN2A, False)
    GPIO.output(IN2B, True)


def CW(speed1 = 100,speed2 = 100):
    speed1,speed2=abs(speed1),abs(speed2)
    EN1_SPEED.start(speed1)
    EN2_SPEED.start(speed2)
    GPIO.output(IN1A, False)
    GPIO.output(IN1B, True)
    GPIO.output(IN2A, True)
    GPIO.output(IN2B, False)
OFF()

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.create_subscription(Twist,"/uav/controller",self.motorControl,1)

    def motorControl(self,control:Twist):
        linear = control.linear
        angular = control.angular
        if linear.x>0 and angular.z==0:
            FWD(linear.x,linear.x)
        elif linear.x<0 and angular.z==0:
            BCK(linear.x,linear.x)
        elif linear.x==0 and angular.z>0:
            CW(angular.z,angular.z)
        elif linear.x==0 and angular.z<0:
            CCW(angular.z,angular.z)
        elif linear.x>0 and angular.z>0:
            FWD(linear.x/2+linear.x/angular.z,linear.x)
        elif linear.x>0 and angular.z<0:
            FWD(linear.x,linear.x/2+linear.x/angular.z)
        elif linear.x<0 and angular.z>0:
            BCK(linear.x/2+linear.x/angular.z,linear.x)
        elif linear.x<0 and angular.z<0:
            BCK(linear.x,linear.x/2+linear.x/angular.z)            
        else:
            OFF()
        

def main(args = None):
    rclpy.init(args = args)
    node  = MotorDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()