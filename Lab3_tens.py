#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import TwistStamped


# Python 2/3 compatibility imports
import sys
import copy
import math
import rospy
import time
import geometry_msgs.msg
import moveit_commander

import cv2
import mediapipe as mp
import numpy as np
import threading 


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

twist = TwistStamped()
Counter_1=0
Counter_2=0
timer=-1
face_detected=True
clap_detected=False
current_frame=None


##########HAND GUESTURE DETECTION
global mp_drawing,mp_drawing_styles,mpHands,timer1,hands,actions

timer1=0
import cv2
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model

# import rospy
from std_msgs.msg import String
# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('/home/sushantshelar/catkin_ws/src/sb_master_repo/survivorbuddy_ros/src/hand-gesture-recognition-code/mp_hand_gesture')

# Load class names
f = open('/home/sushantshelar/catkin_ws/src/sb_master_repo/survivorbuddy_ros/src/hand-gesture-recognition-code/gesture.names', 'r')
classNames = f.read().split('\n')
f.close()
print(classNames)


class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        # self.pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        # self.twist_pub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
        print("generic class")
        # self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=self.audio_callback, queue_size=1)
        # print("executed audio_sub")
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.video_callback, queue_size=1)
        print("Video executed")
        rospy.loginfo("Node started.")
        self.group_name = "survivor_buddy_head"

        # Initialize ROS node and publisher
        self.gesture_pub = rospy.Publisher("/guesture", String, queue_size=10)
        self.message = String()
        self.rate=rospy.Rate(1)
        # Send a message indicating that palm was detected
        
############Face detection and movement#####################
    def video_callback(self, data):
        global timer,timer1 , Counter_1, Counter_2, hands


        global current_frame,VideoData
        VideoData=data
        global mp_drawing,mp_drawing_styles,mp_hands


        if timer1 !=0:
            timer1 -=1
            # print('skipping frame')
            return
        else:
            timer1=10

            if timer==-1:
                timer=int(time.time())

            # if data.data == None:
            #     print("Ignoring empty camera frame.")
            #     # If loading a video, use 'break' instead of 'continue'.
            #     continue

            if VideoData.data is not None:

                np_arr = np.frombuffer(VideoData.data,np.uint8)
                cv_frame = cv2.imdecode(np_arr, 1)
                image=cv2.resize(cv_frame,(120,120))
                x, y, c = image.shape

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                result = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # print('inside video')

                # print(result)
                
                className = ''

                # post process the result
                if result.multi_hand_landmarks:
                    landmarks = []
                    for handslms in result.multi_hand_landmarks:
                        for lm in handslms.landmark:
                            # print(id, lm)
                            lmx = int(lm.x * x)
                            lmy = int(lm.y * y)

                            landmarks.append([lmx, lmy])
                        timer=int(time.time())
                        # Drawing landmarks on frames
                        mpDraw.draw_landmarks(image, handslms, mpHands.HAND_CONNECTIONS)

                        # Predict gesture  13n
                        prediction = model.predict([landmarks])
                        # print(prediction)
                        classID = np.argmax(prediction)
                        if classID == 1:
                            print("Play (Victiory Peace)")
                            self.message.data='victory'
                            self.gesture_pub.publish(self.message)
                            self.rate.sleep()
                        elif classID == 2:
                            print("Increase Volume(Thumbs up)")
                            self.message.data='up'
                            self.gesture_pub.publish(self.message)
                            self.rate.sleep()                            
                            # if (twist.twist.linear.x )<= 45.0:
                            #     twist.twist.linear.x -= -2.0

                        elif classID == 3:
                            print("Decrease Volume (Thumbs Down)")
                            # if (twist.twist.linear.x )>= -45:
                            #     twist.twist.linear.x += -2.0
                            self.message.data='down'
                            self.gesture_pub.publish(self.message)
                            self.rate.sleep()

                        elif classID == 5:
                            print("Stop (Palm)")
                            self.message.data='palm'
                            self.gesture_pub.publish(self.message)
                            self.rate.sleep()
                            
                        elif classID == 8:
                            print("Pause (Fist)")
                            self.message.data='fist'
                            self.gesture_pub.publish(self.message)
                            self.rate.sleep()
                        

                        # else:
                        #     twist.twist.linear.y = 10.0
                        #     time.sleep(0.5)
                        #     twist.twist.linear.y = 0.0
                        #     time.sleep(0.5)
                        #     twist.twist.linear.y = -10.0
                        #     time.sleep(0.5)

                        className = classNames[classID]
                        print(className)               


                cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
                cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node("lab_3_node")
    moveit_commander.roscpp_initialize(sys.argv)
    GenericBehavior()

    rospy.spin()

