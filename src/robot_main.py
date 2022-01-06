import rospy
import math
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

MAX_SPEED = 1
LEFT_PWM_PIN_FORWARD = 13
RIGHT_PWM_PIN_FORWARD = 12
PWM_FREQUENCY = 1000

def scale(val, scale_from, scale_to):
    """
    Scale the given value from the scale of scale_from to the scale of scale_to.
    """
    return ((val - scale_from[0]) / (scale_from[1]-scale_from[0])) * (scale_to[1]-scale_to[0]) + scale_to[0]



def set_wheel_velocity(cmd_vel):
    print("received topic")
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
        right_vel_setpoint = speed*scale(angle, [0, math.pi/2], [-1,1])
    elif joystick_x > 0 and joystick_y <= 0:
        left_vel_setpoint = speed*scale(angle, [math.pi/2, math.pi], [1,-1])
        right_vel_setpoint = speed*1
    elif joystick_x <= 0 and joystick_y < 0:
        left_vel_setpoint = speed*-1
        right_vel_setpoint = speed*scale(angle, [-math.pi/2, -math.pi], [-1,1])
    elif joystick_x< 0 and joystick_y >= 0:
        left_vel_setpoint = speed*scale(angle, [0, -math.pi/2], [1,-1])
        right_vel_setpoint = speed*-1
    

    if left_vel_setpoint > 0:
        left_pwm_forward.ChangeDutyCycle(left_vel_setpoint)
        print("left pwm: " , left_vel_setpoint)
    elif left_vel_setpoint <= 0:
        left_pwm_forward.ChangeDutyCycle(0)
        print("left pwm: " , 0)

    if right_vel_setpoint > 0:
        right_pwm_forward.ChangeDutyCycle(right_vel_setpoint)
        print("right pwm: " , right_vel_setpoint)
    elif right_vel_setpoint <= 0:
        right_pwm_forward.ChangeDutyCycle(0)
        print("left pwm: " , 0)

print("starting")
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LEFT_PWM_PIN_FORWARD,GPIO.OUT)
GPIO.setup(RIGHT_PWM_PIN_FORWARD,GPIO.OUT)

left_pwm_forward = GPIO.PWM(LEFT_PWM_PIN_FORWARD, PWM_FREQUENCY)
right_pwm_forward = GPIO.PWM(RIGHT_PWM_PIN_FORWARD, PWM_FREQUENCY)
left_pwm_forward.start(0)
right_pwm_forward.start(0)

rospy.init_node("robot_node")
rospy.Subscriber('/cmd_vel', Twist, set_wheel_velocity)
print("sub done")
rospy.spin()
