import rospy
import math
from geometry_msgs.msg import Twist

def received_cmd_vel(cmd_vel):
    left_vel_setpoint = math.sqrt(cmd_vel.linear.x**2-math.copysign(1.0, cmd_vel.angular.z)**2)
    right_vel_setpoint = math.sqrt(cmd_vel.linear.x**2+math.copysign(1.0, cmd_vel.angular.z)*cmd_vel.angular.z**2)
    print([left_vel_setpoint, right_vel_setpoint])
    
rospy.init_node("robot_node")
rospy.Subscriber('/cmd_vel', Twist, received_cmd_vel)

rospy.spin()
