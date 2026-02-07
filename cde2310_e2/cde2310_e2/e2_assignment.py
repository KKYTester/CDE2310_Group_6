# For RPi.GPIO to be used
import time
import RPi.GPIO as GPIO

# For ROS2 Nodes
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan # for LiDAR data

SOLENOID_PIN = 16
SERVO_PIN = 12
OUT = 1
IN = 0

# set up RPI GPIOs
GPIO.setmode(GPIO.BCM) 
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

#setup PWM for the servo
servo_motor = GPIO.PWM(SERVO_PIN, 50)
servo_motor.start(0) #init starting angle to 0

# convert 0 to 360 degrees to the approximately 240 ticks used to index msg.ranges
def angle_map(degrees: int):
    ticks = int((degrees / 360) * 240)
    return ticks

def control_solenoid(solenoid_pin: int, state: int):
    if state == 0: # Out
        GPIO.output(solenoid_pin, GPIO.HIGH)
    if state != 0: # In
        GPIO.output(solenoid_pin, GPIO.LOW)

def control_servo(angle):
    # servo input range is 0.5ms to 2.5ms HIGH within 20ms period
    # corresponds to 2.5% to 12.5% PWM duty cycle
    pwm = (angle / 18) + 2.5
    servo_motor.ChangeDutyCycle(pwm)

# Scan front of TurtleBot3 to get laser distance from front ONLY
class ScanFront(Node):

    def __init__(self):
        super().__init__('scan_front')
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.get_logger ().info('Works')
        self.laser_range = 0

    def scan_callback(self, msg):
        front_idx = angle_map(180)
        if msg.ranges[front_idx] < 1.05 and msg.ranges[front_idx] > 0.95:
            control_solenoid(SOLENOID_PIN, OUT)
            control_servo(45)
        else:
            control_solenoid(SOLENOID_PIN, IN)
            control_servo(0)
            
            


def main(args=None):
    rclpy.init(args=args) # set up nodes

    scan_front = ScanFront()

    rclpy.spin(scan_front)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_front.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup() # clean up GPIO pins


if __name__ == '__main__':
    main()