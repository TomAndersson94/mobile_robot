import rospy
import math
import time
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

MAX_SPEED = 1
LEFT_PWM_PIN_FORWARD = 13
LEFT_PWM_PIN_BACKWARD = 19
RIGHT_PWM_PIN_FORWARD = 18
RIGHT_PWM_PIN_BACKWARD = 12
PWM_FREQUENCY = 100

LEFT_HALL_PIN_A = 17
LEFT_HALL_PIN_B = 27
RIGHT_HALL_PIN_A = 23
RIGHT_HALL_PIN_B = 24
HALL_RESOLUTION = 990
WHEEL_RADIUS = 0.04

class Robot:

    left_running_backward = False
    left_running_forward = False
    right_running_backward = False   
    right_running_forward = False
    left_pwm_forward = None
    left_pwm_backward = None
    right_pwm_forward = None
    right_pwm_backward = None

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        #PWM
        GPIO.setup(LEFT_PWM_PIN_FORWARD,GPIO.OUT)
        GPIO.setup(LEFT_PWM_PIN_BACKWARD,GPIO.OUT)
        GPIO.setup(RIGHT_PWM_PIN_FORWARD,GPIO.OUT)
        GPIO.setup(RIGHT_PWM_PIN_BACKWARD,GPIO.OUT)

        self.left_pwm_forward = GPIO.PWM(LEFT_PWM_PIN_FORWARD, PWM_FREQUENCY)
        self.left_pwm_backward = GPIO.PWM(LEFT_PWM_PIN_BACKWARD, PWM_FREQUENCY)
        self.right_pwm_forward = GPIO.PWM(RIGHT_PWM_PIN_FORWARD, PWM_FREQUENCY)
        self.right_pwm_backward = GPIO.PWM(RIGHT_PWM_PIN_BACKWARD, PWM_FREQUENCY)

        #Interrupts
        GPIO.setup(LEFT_HALL_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(LEFT_HALL_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(RIGHT_HALL_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(RIGHT_HALL_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(LEFT_HALL_PIN_A, GPIO.RISING, callback=self.hall_pulse_callback, bouncetime=1)
        GPIO.add_event_detect(LEFT_HALL_PIN_B, GPIO.RISING, callback=self.hall_pulse_callback, bouncetime=1)
        GPIO.add_event_detect(RIGHT_HALL_PIN_A, GPIO.RISING, callback=self.hall_pulse_callback, bouncetime=1)
        GPIO.add_event_detect(RIGHT_HALL_PIN_B, GPIO.RISING, callback=self.hall_pulse_callback, bouncetime=1)

        #ROS
        rospy.init_node("robot_node")
        rospy.Subscriber('/cmd_vel', Twist, self.set_wheel_velocity)
        rospy.spin()


    def hall_pulse_callback(self, channel):
        if channel == LEFT_HALL_PIN_A:
            if not GPIO.input(LEFT_HALL_PIN_B):
                print("left hall a, forwward")
            else:
                print("left hall a, backward")
        elif channel == LEFT_HALL_PIN_B:
            if not GPIO.input(LEFT_HALL_PIN_A):
                print("left hall b, backward")
            else:
                print("left hall b, forward")
        elif channel == RIGHT_HALL_PIN_A:
            if not GPIO.input(RIGHT_HALL_PIN_B):
                print("right hall a, backward")
            else: 
                print("right hall a, forward")
        elif channel == RIGHT_HALL_PIN_B:
            if not GPIO.input(RIGHT_HALL_PIN_A):
                print("right hall b, forward")
            else:
                print("right hall b, backward")
    

    def set_wheel_velocity(self, cmd_vel):
        joystick_x = cmd_vel.linear.x
        joystick_y = -cmd_vel.angular.z
        speed = MAX_SPEED*math.sqrt(joystick_x**2+joystick_y**2)/math.sqrt(2)
    
        angle = 0
        if joystick_y == 0 and joystick_x >= 0:
            angle = math.pi/2
        elif joystick_y == 0 and joystick_x < 0:
            angle = -math.pi/2
        else:
            angle = math.atan(joystick_x/joystick_y)
    
        if joystick_y < 0 and angle < 0:
            angle += math.pi
        elif joystick_y < 0 and angle >= 0:
            angle -= math.pi
    
        if joystick_x == 0 and joystick_y == 0:
            left_vel_setpoint = 0
            right_vel_setpoint = 0
        elif joystick_x >= 0 and joystick_y > 0:
            left_vel_setpoint = speed*1
            right_vel_setpoint = speed*self.scale(angle, [0, math.pi/2], [-1,1])
        elif joystick_x > 0 and joystick_y <= 0:
            left_vel_setpoint = speed*self.scale(angle, [math.pi/2, math.pi], [1,-1])
            right_vel_setpoint = speed*1
        elif joystick_x <= 0 and joystick_y < 0:
            left_vel_setpoint = speed*-1
            right_vel_setpoint = speed*self.scale(angle, [-math.pi/2, -math.pi], [-1,1])
        elif joystick_x< 0 and joystick_y >= 0:
            left_vel_setpoint = speed*self.scale(angle, [0, -math.pi/2], [1,-1])
            right_vel_setpoint = speed*-1

        if left_vel_setpoint > 0:
            if self.left_running_backward:
                self.left_pwm_backward.stop()
                self.left_pwm_forward.start(left_vel_setpoint*100)
                self.left_running_backward = False
                self.left_running_forward = True
            elif self.left_running_forward:
                self.left_pwm_forward.ChangeDutyCycle(left_vel_setpoint*100)
            else:
                self.left_pwm_forward.start(left_vel_setpoint*100)
                self.left_running_forward = True
        elif left_vel_setpoint < 0:
            if self.left_running_backward:
                self.left_pwm_backward.ChangeDutyCycle(abs(left_vel_setpoint)*100)
            elif self.left_running_forward:
                self.left_pwm_forward.stop()
                self.left_pwm_backward.start(abs(left_vel_setpoint)*100)
                self.left_running_forward = False
                self.left_running_backward = True
            else:
                self.left_pwm_backward.start(abs(left_vel_setpoint)*100)
                self.left_running_backward = True
        elif left_vel_setpoint == 0:
            if self.left_running_backward:
                self.left_pwm_backward.stop()
                self.left_running_backward = False
            elif self.left_running_forward:
                self.left_pwm_forward.stop()
                self.left_running_forward = False
    
        if right_vel_setpoint > 0:
            if self.right_running_backward:
                self.right_pwm_backward.stop()
                self.right_pwm_forward.start(right_vel_setpoint*100)
                self.right_running_backward = False
                self.right_running_forward = True
            elif self.right_running_forward:
                self.right_pwm_forward.ChangeDutyCycle(right_vel_setpoint*100)
            else:
                self.right_pwm_forward.start(right_vel_setpoint*100)
                self.right_running_forward = True
        elif right_vel_setpoint < 0:
            if self.right_running_backward:
                self.right_pwm_backward.ChangeDutyCycle(abs(right_vel_setpoint)*100)
            elif self.right_running_forward:
                self.right_pwm_forward.stop()
                self.right_pwm_backward.start(abs(right_vel_setpoint)*100)
                self.right_running_forward = False
                self.right_running_backward = True
            else:
                self.right_pwm_backward.start(abs(right_vel_setpoint)*100)
                self.right_running_backward = True
        elif right_vel_setpoint == 0:
            if self.right_running_backward:
                self.right_pwm_backward.stop()
                self.right_running_backward = False
            elif self.right_running_forward:
                self.right_pwm_forward.stop()
                self.right_running_forward = False


    def scale(self, val, scale_from, scale_to):
        """
        Scale the given value from the scale of scale_from to the scale of scale_to.
        """
        return ((val - scale_from[0]) / (scale_from[1]-scale_from[0])) * (scale_to[1]-scale_to[0]) + scale_to[0]
