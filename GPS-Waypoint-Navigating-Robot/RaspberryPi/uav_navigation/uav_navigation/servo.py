from gpiozero import Servo
from time import sleep

servo = Servo(12, min_pulse_width=1 / 2200, max_pulse_width=2.1 / 1000)
val = -1

try:
    while True:
        for i in range(-10, 11, 1):
            servo.value = i / 10
            sleep(0.075)
        sleep(0.2)
        for i in range(10, -11, -1):
            servo.value = i / 10
            sleep(0.075)
        sleep(0.2)
except KeyboardInterrupt:
    servo.stop()
    print("Program stopped")
