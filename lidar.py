#!/usr/bin/python
import rospy
from custom_msg_python.msg import custom
import serial

# Configure the serial port
ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)  # Adjust the port if needed
uart=[0]*9
HEADER=89

def lidar():

    rospy.init_node("lidar_node", anonymous = True)
    pub= rospy.Publisher('/lidar_topic', custom, queue_size=10)
    msg=custom()

    while not rospy.is_shutdown():
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
                        msg.p=dist
                        print(dist)
                        pub.publish(msg)



if __name__=='__main__':
    try:
        lidar()
    except rospy.ROSInterruptException:
        pass
