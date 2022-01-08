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


class Robot:

    left_running_backward = False
    left_running_forward = False
    right_running_backward = False   
    right_running_forward = False

    def __init(self):

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LEFT_PWM_PIN_FORWARD,GPIO.OUT)
        GPIO.setup(LEFT_PWM_PIN_BACKWARD,GPIO.OUT)
        GPIO.setup(RIGHT_PWM_PIN_FORWARD,GPIO.OUT)
        GPIO.setup(RIGHT_PWM_PIN_BACKWARD,GPIO.OUT)

        left_pwm_forward = GPIO.PWM(LEFT_PWM_PIN_FORWARD, PWM_FREQUENCY)
        left_pwm_backward = GPIO.PWM(LEFT_PWM_PIN_BACKWARD, PWM_FREQUENCY)
        right_pwm_forward = GPIO.PWM(RIGHT_PWM_PIN_FORWARD, PWM_FREQUENCY)
        right_pwm_backward = GPIO.PWM(RIGHT_PWM_PIN_BACKWARD, PWM_FREQUENCY)

        left_pwm_forward.start(0)
        right_pwm_forward.start(0)

        rospy.init_node("robot_node")
        rospy.Subscriber('/cmd_vel', Twist, self.set_wheel_velocity)
        rospy.spin()


    def set_wheel_velocity(self, cmd_vel):
        joystick_x = cmd_vel.linear.x
        joystick_y = -cmd_vel.angular.z
        speed = MAX_SPEED*math.sqrt(joystick_x**2+joystick_y**2)
    
        angle = 0
        if joystick_y == 0 and joystick_x >= 0:
            angle = math.pi/2
        elif joystick_x == 0 and joystick_x < 0:
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
            self.left_pwm_forward.ChangeDutyCycle(left_vel_setpoint*100)
        else:
            self.left_pwm_forward.ChangeDutyCycle(0)
        
        if right_vel_setpoint > 0:
            self.right_pwm_forward.ChangeDutyCycle(right_vel_setpoint*100)
        else:
            self.right_pwm_forward.ChangeDutyCycle(0)

        #if left_vel_setpoint > 0:
        #    if self.left_running_backward:
        #        self.left_pwm_backward.stop()
        #        self.left_pwm_forward.start(left_vel_setpoint*100)
        #        self.left_running_backward = False
        #    else:
        #        self.left_pwm_forward.ChangeDutyCycle(left_vel_setpoint*100)
        #    
        #elif left_vel_setpoint < 0:
        #    if self.left_running_backward:
        #        self.left_pwm_backward.ChangeDutyCycle(abs(left_vel_setpoint)*100)
        #    else:
        #        self.left_pwm_forward.stop()
        #        self.left_pwm_backward.start(abs(left_vel_setpoint)*100)
        #        self.left_running_backward = True
        #elif left_vel_setpoint == 0:
        #    if self.left_running_backward:
        #        self.left_pwm_backward.stop()
        #    else:
        #        self.left_pwm_forward.stop()
    
    
        #if right_vel_setpoint > 0:
        #    self.right_pwm_forward.ChangeDutyCycle(right_vel_setpoint*100)
    #
        #elif right_vel_setpoint <= 0:
        #    self.right_pwm_backward.ChangeDutyCycle(right_vel_setpoint*100)
    

    def scale(val, scale_from, scale_to):
        """
        Scale the given value from the scale of scale_from to the scale of scale_to.
        """
        return ((val - scale_from[0]) / (scale_from[1]-scale_from[0])) * (scale_to[1]-scale_to[0]) + scale_to[0]
