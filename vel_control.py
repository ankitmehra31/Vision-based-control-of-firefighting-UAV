#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State,RCIn
from custom_msg_python.msg import custom
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
import serial


global error_y
global error_x
error_y=0
global error_dy
global error_dz
global xpid_gains
global ypid_gains
global zpid_gains
global initial_psi
global check,rc
rc=0
global spray_dist,new_dist,ranger_x
global phi,theta,psi
ranger_x=2            #+- 0.5 meters of  error allowed in x direction on fixed spray distace
theta=0
spray_dist=5.5
new_dist=spray_dist
check=0
xpid_gains=[0.0,0.0,0.0]                #pitch direction(forward)
ypid_gains=[0.1,0.0,0.1]               #roll direction(sidewards)
zpid_gains=[-0.1,0.0,0.0]              #altitude direction(up and down)
error_dy=0.0
error_dz=0.0
screen_x=384/2
screen_y=384/2
#Kpy=0.01
moving=[]
#print("Hello")
e_lx = [0]*10
e_ly = [0]*10
e_lz=[0]*10

def pidx(error,pid_gain):
    # print("x error",error)
    e_lx.append(error)
    if  len(e_lx) > 10:
        del e_lx[0]
    error_int = float((1/3)*((e_lx[-10]+e_lx[-1])+(4*(e_lx[-2]+e_lx[-4]+e_lx[-6]+e_lx[-8]))+(2*(e_lx[-3]+e_lx[-5]+e_lx[-7]+e_lx[-9]))))
    # pid[0]=p_gain          
    # pid[0]=i_gain
    # pid[0]=d_gain            
    p_value=pid_gain[0]*(error)
    i_value=pid_gain[1]*(error_int)
    d_value=pid_gain[2]*((e_lx[-2]-e_lx[-1])/0.1)
    V_x =  p_value + i_value + d_value
    # print("yyyyyy",e_ly[-1],e_ly[-2])
    return V_x , p_value ,i_value, d_value
def pidy(error,pid_gain):
    # print("y error",error)
    e_ly.append(error)
    if  len(e_ly) > 10:
        del e_ly[0]
    error_int = float((1/3)*((e_ly[-10]+e_ly[-1])+(4*(e_ly[-2]+e_ly[-4]+e_ly[-6]+e_ly[-8]))+(2*(e_ly[-3]+e_ly[-5]+e_ly[-7]+e_ly[-9]))))
    # pid[0]=p_gain          
    # pid[0]=i_gain
    # pid[0]=d_gain            
    p_value=pid_gain[0]*(error)
    i_value=pid_gain[1]*(error_int)
    d_value=pid_gain[2]*((e_ly[-2]-e_ly[-1])/0.1)
    V_y =  p_value + i_value + d_value
    # print("yyyyyy",e_ly[-1],e_ly[-2])
    return V_y , p_value ,i_value, d_value
def pidz(error,pid_gain):
    # print("z error",error)
    e_lz.append(error)
    if  len(e_lz) > 10:
        del e_lz[0]
    error_int = float((1/3)*((e_lz[-10]+e_lz[-1])+(4*(e_lz[-2]+e_lz[-4]+e_lz[-6]+e_lz[-8]))+(2*(e_lz[-3]+e_lz[-5]+e_lz[-7]+e_lz[-9]))))
    # pid[0]=p_gain          
    # pid[0]=i_gain
    # pid[0]=d_gain            
    p_value=pid_gain[0]*(error)
    i_value=pid_gain[1]*(error_int)
    d_value=pid_gain[2]*((e_lz[-2]-e_lz[-1])/0.1)
    V_z =  p_value + i_value + d_value
    # print("zzzzzzz",e_lz[-1],e_lz[-2])
    return V_z , p_value ,i_value, d_value
def callback(A):
    global error_dy
    global error_dz
    global error_y
    global error_x
    cent_x=A.coordinates[0]
    cent_y=A.coordinates[1]
   # print(cent_y,cent_x)
    error_y=screen_y-cent_y    #error_y is error in image frame and error_dz error in drone 
    error_dz=-error_y
    error_x=screen_x-cent_x
    error_dy=error_x
    #print(error_y)

def RC_callback(rc_data):
    global rc
    channel=rc_data.channels
    check_offb=channel[5]
    if 1400<check_offb<1600:
        rc=1
    else:
        rc=0

    

def lidar(event):
    global spray_dist,theta,new_dist
    # Configure the serial port
    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)  # Adjust the port if needed
    uart=[0]*9
    HEADER=89
    while True:
        # Read data from the sensor
        if ser.in_waiting:
            if ord(ser.read())==HEADER:
                uart[0]=HEADER
                if ord(ser.read())==HEADER:
                    uart[1]=HEADER

                    for i in range(2,9):
                        uart[i]=ord(ser.read())

                    check=sum(uart[0:8])
                
                    if uart[8]==(check & 255):
                        dist =uart[2]+(uart[3]*256)
                        strength=uart[4]+(uart[5]*256)

                        new_dist=(dist*(math.cos(theta)))/100.0
                        # return (dist)
def limits(lower,upper,vel):
    if vel >upper:
        vel=upper
    elif vel< lower:
        vel=lower
    else:
        vel=vel
    return vel

def imu_callback(data):
    global error_dy
    global error_dz
    global error_x,ranger_x
    global ypid_gains
    global zpid_gains
    global check,new_dist
    global initial_psi
    global phi, theta ,psi
    quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    #print(quat)
    phi,theta,psi = euler_from_quaternion(quat)
    
    if check ==0:
        initial_psi=psi
        check=1

    pid_plot=custom()
    pid_plot.header.frame_id="map"

    pid_plotz=custom()
    pid_plotz.header.frame_id="map"

    pid_plotx=custom()
    pid_plotx.header.frame_id="map"

    corrected_dy=error_dy
    corrected_dz=error_dz
    # print("corrected dy",corrected_dy)
    moving.append([corrected_dy,corrected_dz])
    if len(moving)>16:
        moving_err_y=(1.0/17.0)*(moving[0][0]+moving[1][0]+moving[2][0]+moving[3][0]+moving[4][0]+moving[5][0]+moving[6][0]+moving[7][0]+moving[8][0]+moving[9][0]+moving[10][0]+moving[11][0]+moving[12][0]+moving[13][0]+moving[14][0]+moving[15][0]+moving[16][0])
        moving_err_z=(1.0/17.0)*(moving[0][1]+moving[1][1]+moving[2][1]+moving[3][1]+moving[4][1]+moving[5][1]+moving[6][1]+moving[7][1]+moving[8][1]+moving[9][1]+moving[10][1]+moving[11][1]+moving[12][1]+moving[13][1]+moving[14][1]+moving[15][1]+moving[16][1])
        del moving[0]
    else:
        moving_err_y=corrected_dy
        moving_err_z=corrected_dz
    
    moving_err_y=moving_err_y/40
    moving_err_z=moving_err_z/40
    # print("moving error y",moving_err_y)
    # print("moving error z",moving_err_z)

    x_error=new_dist-spray_dist
    if abs(x_error) >= ranger_x: 
        x_error=0
    print("x_error",x_error,new_dist)
    V_x,px,ix,dx=pidx(x_error,xpid_gains)
    
    V_y,py,iy,dy=pidy(moving_err_y,ypid_gains)

    V_z,pz,iz,dz=pidz(moving_err_z,zpid_gains)
    
    # yaw_rate =-0.1*(initial_psi-psi)

    V_x=limits(-2,2,V_x)
    V_y=limits(-2,2,V_y)
    V_z=limits(-2,2,V_z)
    
    V_X = math.cos(psi)*V_x - math.sin(psi)*V_y
    V_Y = math.sin(psi)*V_x + math.cos(psi)*V_y
    V_Z=V_z   
    # print(V_x, V_y,V_z)
    # print("p term y",py,"i term y",iy,"d term y",dy)
    # print("p term z",pz,"i term z",iz,"d term z",dz)

    pid_plot.header.stamp = rospy.Time.now()
    pid_plot.p=py
    pid_plot.i=phi
    pid_plot.d=dy
    pid_plot.og_error=moving_err_y
    pid_plot.pid_error=V_y
    pub1.publish(pid_plot)

    pid_plotz.header.stamp = rospy.Time.now()
    pid_plotz.p=pz
    pid_plotz.i=rc
    pid_plotz.d=dz
    pid_plotz.og_error=moving_err_z
    pid_plotz.pid_error=V_z
    puby.publish(pid_plotz)

    pid_plotx.header.stamp = rospy.Time.now()
    pid_plotx.p=px
    pid_plotx.i=ix
    pid_plotx.d=dx
    pid_plotx.og_error=x_error
    pid_plotx.pid_error=V_x
    pubx.publish(pid_plotx)

    msg = TwistStamped()
    msg.twist.linear.x = V_X
    msg.twist.linear.y = V_Y
    msg.twist.linear.z = V_Z
    msg.twist.angular.z = 0.0
    pub.publish(msg)
    # print("corected  y",corrected_dy,"error y",error_dy) 
    # print("yaw rate",yaw_rate)

    # print("oooooffffboooarrrddd",rc)
    



if __name__=='__main__':
    print(1)
    rospy.init_node('transformation', anonymous = True)
    print(2)
    sub2=rospy.Subscriber("custom_message",custom,callback)
    print(3)
    #sub3=rospy.Subscriber("/lidar_topic",custom,callback1)
    sub = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    print(4)
    sub1 = rospy.Subscriber('/mavros/rc/in', RCIn, RC_callback)
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    print(5)
    pub1=rospy.Publisher("/pid_topic",custom,queue_size=10)
    puby=rospy.Publisher("/pid_topicz",custom,queue_size=10)
    pubx=rospy.Publisher("/pid_topicx",custom,queue_size=10)
    rospy.Timer(rospy.Duration(0.02),lidar)
    rospy.spin()
    
   
