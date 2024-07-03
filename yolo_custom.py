#!/home/firefighter/myenv/bin/python3

import cv2
import ultralytics
from ultralytics import YOLO
import sys
import time
import rospy
from custom_msg_python.msg import custom
import random
import timeit

# Initialize YOLO model
model = YOLO('/home/firefighter/catkin_ws/src/custom_msg_python/src/best_2.pt')

# Initialize video capture from the default camera (change the index if using a different camera)
cap = cv2.VideoCapture(0)
prev_frame_time=0
new_frame_time=0
global cc
global fire_detected 
fire_detected = False
cc=0

def talk():
    global fire_detected
    pub=rospy.Publisher("custom_message",custom,queue_size=10)
    rospy.init_node("custom_publisher",anonymous=True)
    #rate=rospy.Rate(6)
    msg=custom()
    #msg.header.frame_id="map"

    
    while not rospy.is_shutdown():
        # Read frame from camera
        global cc
        ret, frame = cap.read()
        frame=cv2.resize(frame,(384,384))
        #msg.header.stamp = rospy.Time.now()

        
        # Perform object detection
        tik=timeit.default_timer()
        time_up=tik*10
        time_up=int(time_up)
        #print("timeup",time_up)
        if cc==0:
            results=model.predict(frame)
            cc=5
        if time_up %3==0:   # giving every 3rd frame to model
            results = model.predict(frame)
        
        detected = False  # Initialize detection flag
        
        for result in results:
            if len(result.boxes) > 0:  # Check if any boxes are detected
                detected = True  # Set detected to True
                for bbox in result.boxes.xyxy:
                    x1, y1, x2, y2 = bbox[0].item(), bbox[1].item(), bbox[2].item(), bbox[3].item()
                    # Draw bounding box around detected object (fire)
                    color = (0, 255, 0)  # Green color
                    thickness = 2
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                    centroid = [(x1 + x2) / 2, (y1 + y2) / 2]
                    msg.coordinates=centroid
                    break  # Only consider the first detected box and centroid
            else:
                msg.coordinates=[384/2,384/2]
        if detected:
            fire_detected = True
            print("Fire Detected")
            #print("Centroid:", centroid)
            print("True")
            cmd = "1"
            # break  # Stop the code execution if fire is detected
        if not fire_detected:
            print("No fire detected")
            print("False")
            #msg.coordinates=[0,0]
            cmd = "0"
            # break
        
        # Display frame with bounding boxes and centroid
        # cv2.imshow("Fire Detection", frame)
        # cv2.waitKey(1)
        # Check for 'q' key press to exit the loop
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        #while not rospy.isshutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        #rospy.spin()
        #rate.sleep()

if __name__=="__main__":
    try:
        talk()
    except rospy.ROSInterruptException:
        pass
