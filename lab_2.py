#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import TwistStamped


import audio
import globals
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
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

twist = TwistStamped()

def get_dist(a,b):
    return math.sqrt( (a.x-b.x)**2 +(a.y-b.y)**2   )


class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        self.pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        globals.initialize()
        print("early print ",globals.count)
        self.sampub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
        self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=audio.callback_1, queue_size=1)
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.callback_2, queue_size=1)
        rospy.loginfo("Node started.")
        audio.printing(6)
        print("new count is ",globals.count)
        #self.robot = moveit_commander.RobotCommander()
        #self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "survivor_buddy_head"
        #self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    # execute_behaviour

    

    def callback_2(self, data):
       # print("enter")
        global count
        global last_time
        global cur
        global an_cur
        global confused
        np_arr = np.frombuffer(data.data,np.uint8)
        image = cv2.imdecode(np_arr, 1)
        if last_time==-1:
            last_time=int(time.time())
        cv2.namedWindow("Image Window",1)

        with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detection:
            results = face_detection.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            if results.detections:
                annotated_image = image.copy()
                for detection in results.detections:
                  #  print('Nose tip:')
                    if sound==False:
                        return
                    last_time=int(time.time())
                    confused=True
                   # print(mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.NOSE_TIP))
                    nose=mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.NOSE_TIP)
                    left=mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.LEFT_EAR_TRAGION)
                    right=mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.RIGHT_EAR_TRAGION)

                   # print('Right:')
                   # print(mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.LEFT_EYE))

                   # print('LEFT:')
                   # print(mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.RIGHT_EYE))

                    left_dist=get_dist(left,nose) 
                    right_dist=get_dist(right,nose)
                    new_dist=get_dist(right,left)

                    left_ratio=left_dist/right_dist
                    #right_ratio=right_dist/left_dist

                    print("DIST is ",new_dist)
                    flag=False
                    if left_ratio>=0.75 and left_ratio <=1.25:
                        flag=True

                    if cur==0 and new_dist>=0.19 and new_dist<=0.42:
                        if self.execute_behavior2(True):
                            print("FORWARD")
                            cur=cur+1 
                    elif cur==1 and new_dist>=0.23 :
                        if self.execute_behavior2(True):
                            print("FORWARD")
                            cur=cur+1 
                    elif cur==2 and new_dist>=0.27:
                        if self.execute_behavior2(True):
                            print("FORWARD")
                            cur=cur+1 
                    elif cur==3 and new_dist>=0.3:
                        if self.execute_behavior2(True):
                            print("TOO CLOSE!!")
                            cur=0
                    elif cur==3 and new_dist<=0.26:
                        if self.execute_behavior2(False):
                            print("BACKWARD")
                            cur=cur-1
                    elif cur==2 and new_dist<=0.22:
                        if self.execute_behavior2(False):
                            print("BACKWARD")
                            cur=cur-1
                    elif cur==1 and new_dist<=0.19:
                        if self.execute_behavior2(False):
                            print("BACKWARD")
                            cur=cur-1 


                    if(an_cur==0 and nose.x<0.35):
                        self.execute_behavior3(True)
                        an_cur=1
                        print("You MOVED   RIGHT")
                    if(an_cur==0 and nose.x>0.65):
                        print("You MOVED   LEFT")
                        self.execute_behavior3(False)
                        an_cur=1
                    
                         
                    
                    if(an_cur>0):
                        an_cur+=1

                    if(an_cur==10):
                        an_cur=0


                    #mp_drawing.draw_detection(annotated_image, detection)               //COMMENTED THESE 2 lines
                #cv2.imwrite('/tmp/annotated_image'  + '.png', annotated_image)
                
            elif confused and time.time()-int(last_time)>=2:
                print("CONFUSED")
                self.execute_behavior4(1)
                last_time=int(time.time())
                confused=False
                

        cv2.imshow("Image Window",image)
        cv2.waitKey(3)
        pass

if __name__ == '__main__':
    rospy.init_node("lab_1_node")
    moveit_commander.roscpp_initialize(sys.argv)
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    GenericBehavior()
    rospy.spin()
