#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from custom_msg_python.msg import custom
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

global error_y
global error_x
error_y=0
global error_dy
error_dy=0
screen_x=384/2
screen_y=384/2
Kpy=0.01
print("Hello")

def callback(A):
    global error_dy
    global error_y
    global error_x
    cent_x=A.coordinates[0]
    cent_y=A.coordinates[1]
   # print(cent_y,cent_x)
    error_y=screen_y-cent_y    #error_y is error in image frame and error_dz error in drone 
    error_dz=-error_y
    error_x=screen_x-cent_x
    error_dy=error_x
    print(error_y)


def imu_callback(data):
    global error_dy
    global error_x
    quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    #print(quat)
    phi,theta,psi = euler_from_quaternion(quat)
    #print(psi *(180/math.pi))
    #print("ankit")
    if error_dy>=15:
        corrected_dy=error_dy-15
    elif error_dy<=-15:
        corrected_dy=error_dy+15
    else:
        corrected_dy=0

    V_x = 0.0
    V_y =Kpy*corrected_dy
    #V_y=0.2
    V_z = 0.0
    yaw_rate = 0.0
    V_X=V_x
    if V_y>=0.3:
        V_y=0.3
    if V_y<=-0.3:
        V_y=-0.3
    
    V_X = math.cos(psi)*V_x - math.sin(psi)*V_y
    V_Y = math.sin(psi)*V_x + math.cos(psi)*V_y
    V_Z=V_z   
    print(V_x, V_y)
    msg = TwistStamped()
    msg.twist.linear.x = V_X
    msg.twist.linear.y = V_Y
    msg.twist.linear.z = V_Z
    msg.twist.angular.z = 0.0
    pub.publish(msg)
    print("corected",corrected_dy,"error",error_dy) 
    # rate = rospy.Rate(6) 
    # rate.sleep()



if __name__=='__main__':
    print(1)
    rospy.init_node('transformation', anonymous = True)
    print(2)
    sub2=rospy.Subscriber("custom_message",custom,callback)
    print(3)
    sub = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    print(4)
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    print(5)
    rospy.spin()
    
   
