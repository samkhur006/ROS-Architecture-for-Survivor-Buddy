#!/usr/bin/env python3
from __future__ import annotations
from abc import ABC, abstractmethod
from random import randrange
from typing import List

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory
from threading import Thread
from geometry_msgs.msg import TwistStamped


# Python 2/3 compatibility imports
import sys
import globals
import copy
import random
import math
import rospy
import time
import geometry_msgs.msg
import moveit_commander
import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import threading
import signal
import time

motor_pos = [0, 0, 0, 0]
global_goal = [0,0,0,0]
global_speed = 1
time_rate = 0.05

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


steps1 = [[0, 0, 0, 0, 2], [10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5], [-5, -20, 0, 30, 10], [5, 0, 0, 0, 15], [0, 0, 10, 45, 10], [5, -10, -10, 0, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5], [-5, -20, 0, 30, 10],  [0, 0, -10, -45, 10], [5, 10, -10, 0, 5], [-5, -5, 10, 0, 10], [10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5], [-5, -20, 0, 30, 10], [5, 0, 0, 0, 15], [0, 0, 10, 45, 10], [5, -10, -10, 0, 5], [-5, 5, 10, 0, 10]]
steps2 = [[10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5], [-5, -20, 0, 30, 10], [5, 0, 0, 0, 15], [0, 0, 10, 45, 10], [-5, 10, -10, 0, 5], [5, -5, 10, 0, 10]]
steps3 = [[-5, 5, 10, 0, 10],  [10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5], [-5, -20, 0, 30, 10], [5, 0, 0, 0, 15], [0, 0, 10, 45, 10], [5, -10, -10, 0, 5], [-5, 5, 10, 0, 10], [10, 0, 0, 20, 10], [5, 20, 0, 30, 5]]
steps4 = [[5, 0, 0, 0, 15], [0, 0, 10, 45, 10], [-5, 10, -10, 0, 5], [5, -5, 10, 0, 10], [-10, 0, 0, 0, 15]]

steps = []
# steps.append(steps1)
# steps.append(steps2)
# steps.append(steps3)
# steps.append(steps4)
# face wave
side_w_1 = [[0, 0, 0, 0, 4], [0, -20, 0, 0, 4], [0, 0, 0, 0, 4], [0, 20, 0, 0, 4],
        [0, 0, 0, 0, 4], [0, -20, 0, 0, 4], [0, 0, 0, 0, 4], [0, 20, 0, 0, 4], [0, 0, 0, 0, 4]]
side_w_reverse = [[0, 0, 0, 0, 4], [0, 20, 0, 0, 4], [0, 0, 0, 0, 4], [0, -20, 0, 0, 4],
        [0, 0, 0, 0, 4], [0, 20, 0, 0, 4], [0, 0, 0, 0, 4], [0, -20, 0, 0, 4], [0, 0, 0, 0, 4]]

# torso twist 
side_t_1 = [[0, 0, 0, 0, 4], [0, -20, 20, 0, 4], [0, 0, 0, 0, 5], [0, 20, -20, 0, 4],
        [0, 0, 0, 0, 4], [0, -20, 20, 0, 4], [0, 0, 0, 0, 5], [0, 20, -20, 0, 4], [0, 0, 0, 0, 4]]        
side_t_reverse = [[0, 0, 0, 0, 4], [0, 20, -20, 0, 4], [0, 0, 0, 0, 5], [0, -20, 20, 0, 4],
        [0, 0, 0, 0, 4], [0, 20, -20, 0, 4], [0, 0, 0, 0, 5], [0, -20, 20, 0, 4], [0, 0, 0, 0, 4]]        

# torso bend
side_b_1 = [[0, 0, 0, 0, 4], [0, -20, -20, 0, 4], [0, 0, 0, 0, 5], [0, 20, 20, 0, 4],
        [0, 0, 0, 0, 4], [0, -20, -20, 0, 4], [0, 0, 0, 0, 5], [0, 20, 20, 0, 4], [0, 0, 0, 0, 4]]
side_b_reverse = [[0, 0, 0, 0, 4], [0, 20, 20, 0, 4], [0, 0, 0, 0, 5], [0, -20, -20, 0, 4],
        [0, 0, 0, 0, 4], [0, 20, 20, 0, 4], [0, 0, 0, 0, 5], [0, -20, -20, 0, 4], [0, 0, 0, 0, 4]]

# torso wave
body_t_1 = [[0, 0, 0, 0, 4], [-20, -20, 20, 0, 4], [0, 0, 0, 0, 5], [20, 20, -20, 0, 4],
            [0, 0, 0, 0, 4], [-20, 20, -20, 0, 4], [0, 0, 0, 0, 5], [20, -20, 20, 0, 4],
            [0, 0, 0, 0, 4], [-20, -20, 20, 0, 4], [0, 0, 0, 0, 5], [20, 20, -20, 0, 4],         
            [0, 0, 0, 0, 4], [-20, 20, -20, 0, 4], [0, 0, 0, 0, 5], [20, -20, 20, 0, 4], [0, 0, 0, 0, 4]]

body_t_reverse = [[0, 0, 0, 0, 4], [-20, 20, 20, 0, 4], [0, 0, 0, 0, 5], [20, -20, -20, 0, 4],
            [0, 0, 0, 0, 4], [-20, -20, -20, 0, 4], [0, 0, 0, 0, 5], [20, 20, 20, 0, 4],
            [0, 0, 0, 0, 4], [-20, 20, 20, 0, 4], [0, 0, 0, 0, 5], [20, -20, -20, 0, 4],         
            [0, 0, 0, 0, 4], [-20, -20, -20, 0, 4], [0, 0, 0, 0, 5], [20, 20, 20, 0, 4], [0, 0, 0, 0, 4]]

# neck wave
neck_w_1 = [[0, 0, 0, 0, 4], [0, 0, 0, -20, 4], [0, 0, 0, 0, 5], [0, 0, 0, 20, 4],
        [0, 0, 0, 0, 4], [0, 0, 0, -20, 4], [0, 0, 0, 0, 5], [0, 0, 0, 20, 4], [0, 0, 0, 0, 4]]
neck_w_reverse = [[0, 0, 0, 0, 4], [0, 0, 0, 20, 4], [0, 0, 0, 0, 5], [0, 0, 0, -20, 4],
        [0, 0, 0, 0, 4], [0, 0, 0, 20, 4], [0, 0, 0, 0, 5], [0, 0, 0, -20, 4], [0, 0, 0, 0, 4]]

# neck bend
neck_b_1 = [[0, 0, 0, 0, 4], [20, 0, 0, -20, 4], [0, 0, 0, 0, 5], [-20, 0, 0, 20, 4],
        [0, 0, 0, 0, 4], [20, 0, 0, -20, 4], [0, 0, 0, 0, 5], [-20, 0, 0, 20, 4], [0, 0, 0, 0, 4]]
neck_b_reverse = [[0, 0, 0, 0, 4], [-20, 0, 0, 20, 4], [0, 0, 0, 0, 5], [20, 0, 0, -20, 4],
        [0, 0, 0, 0, 4], [-20, 0, 0, 20, 4], [0, 0, 0, 0, 5], [20, 0, 0, -20, 4], [0, 0, 0, 0, 4]]
solo_time = 18
delta = 10
sync_time = 85

# STEP 2: Create an GestureRecognizer object.
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

images = []
results = []

CLAP_THREASHOLD = 100



class Subject(ABC):
    """
    The Subject interface declares a set of methods for managing subscribers.
    """

    @abstractmethod
    def addObserver(self, observer: Observer) -> None:
        """
        Attach an observer to the subject.
        """
        pass

    @abstractmethod
    def removeObserver(self, observer: Observer) -> None:
        """
        Detach an observer from the subject.
        """
        pass

    @abstractmethod
    def notify(self) -> None:
        """
        Notify all observers about an event.
        """
        pass


class CameraPerception(Subject):
    """
    The Subject owns some important state and notifies observers when the state
    changes.
    """

    # _state: int = None
    _state: CompressedImage = None
    """
    For the sake of simplicity, the Subject's state, essential to all
    subscribers, is stored in this variable.
    """

    _observers: List[Observer] = []
    """
    List of subscribers. In real life, the list of subscribers can be stored
    more comprehensively (categorized by event type, etc.).
    """

    def addObserver(self, observer: Observer) -> None:
        # print("Subject: Attached an observer.")
        self._observers.append(observer)

    def removeObserver(self, observer: Observer) -> None:
        self._observers.remove(observer)

    """
    The subscription management methods.
    """

    def notify(self) -> None:
        """
        Trigger an update in each subscriber.
        """

        # print("Subject: Notifying observers...")
        for observer in self._observers:
            observer.update(self)

    def setChanged(self, data) -> None:
        """
        Usually, the subscription logic is only a fraction of what a Subject can
        really do. Subjects commonly hold some important business logic, that
        triggers a notification method whenever something important is about to
        happen (or after it).
        """

        # print("\nSubject: I'm doing something important.")
        # self._state = randrange(0, 10)
        self._state = data

        # print(f"Subject: My state has just changed to: {self._state}")
        self.notify()

class AudioPerception(Subject):
    """
    The Subject owns some important state and notifies observers when the state
    changes.
    """

    # _state: int = None
    _state: Float32MultiArray = None
    """
    For the sake of simplicity, the Subject's state, essential to all
    subscribers, is stored in this variable.
    """

    _observers: List[Observer] = []
    """
    List of subscribers. In real life, the list of subscribers can be stored
    more comprehensively (categorized by event type, etc.).
    """

    def addObserver(self, observer: Observer) -> None:
        # print("Subject: Attached an observer.")
        self._observers.append(observer)

    def removeObserver(self, observer: Observer) -> None:
        self._observers.remove(observer)

    """
    The subscription management methods.
    """

    def notify(self) -> None:
        """
        Trigger an update in each subscriber.
        """

        # print("Subject: Notifying observers...")
        for observer in self._observers:
            observer.update(self)

    def setChanged(self, audioData) -> None:
        """
        Usually, the subscription logic is only a fraction of what a Subject can
        really do. Subjects commonly hold some important business logic, that
        triggers a notification method whenever something important is about to
        happen (or after it).
        """

        # print("\nSubject: I'm doing something important.")
        # self._state = randrange(0, 10)
        self._state = audioData

        # print(f"Subject: My state has just changed to: {self._state}")
        self.notify()


class Observer(ABC):
    """
    The Observer interface declares the update method, used by subjects.
    """

    @abstractmethod
    def update(self, subject: Subject) -> None:
        """
        Receive update from subject.
        """
        pass


"""
Concrete Observers react to the updates issued by the Subject they had been
attached to.
"""
class ClapCartographer(Observer, Subject):
    """
    The Subject owns some important state and notifies observers when the state
    changes.
    """
    # _state: int = None
    _state: CompressedImage = None
    previous_audio = None
    new_audio = None
    """
    For the sake of simplicity, the Subject's state, essential to all
    subscribers, is stored in this variable.
    """

    _observers: List[Observer] = []
    """
    List of subscribers. In real life, the list of subscribers can be stored
    more comprehensively (categorized by event type, etc.).
    """
    def __init__(self):
        x = threading.Thread(target=self.detectClap, args=())
        x.start()
    def addObserver(self, observer: Observer) -> None:
        print("ClapCartographer: Attached an observer.")
        self._observers.append(observer)

    def removeObserver(self, observer: Observer) -> None:
        self._observers.remove(observer)

    """
    The subscription management methods.
    """

    def notify(self) -> None:
        """
        Trigger an update in each subscriber.
        """

        # print("ClapCartographer: Notifying observers...")
        for observer in self._observers:
            observer.update(self)

    def setChanged(self, clap_data) -> None:
        """
        Usually, the subscription logic is only a fraction of what a Subject can
        really do. Subjects commonly hold some important business logic, that
        triggers a notification method whenever something important is about to
        happen (or after it).
        """

        # print("\ClapCartographer: calling setChanged")
        # self._state = randrange(0, 10)
        self._state = clap_data

        # print(f"Subject: My state has just changed to: {self._state}")
        self.notify()

    def update(self, subject: Subject) -> None:
        # print("AudioCartographer: Received audioPerception data")
        # self.detectClap(subject._state)
        self.new_audio = subject._state


    
    def isNoise(self,arr):
        thr = CLAP_THREASHOLD	#Threashold
        sum = 0
        for i in arr:
            sum+=abs(i)
        if(abs(sum)>thr):
            print(abs(sum))
            intensity = (abs(sum)-thr)/thr*100
            #print("intensity: ", intensity)
            return True
            #print(abs(sum))
        return False

    def detectClap(self):  
        while not killer.kill_now:
            if(self.previous_audio != self.new_audio):  
                self.previous_audio = self.new_audio
                audio_input_data = self.new_audio
                if(self.isNoise(audio_input_data.data)):
                    print("===========================================Clap detected from AudioPerception data: ")
                    clap_detected = True
                    self.setChanged(clap_detected) 
                else:
                    self.setChanged(False)

            time.sleep(0.1)


class CameraObserver(Observer):
    def update(self, subject: Subject) -> None:
        # print("Camera Observer: Reacted to the event", subject._state)
        pass


class FaceCartographer(Observer, Subject):
    """
    The Subject owns some important state and notifies observers when the state
    changes.
    """
    _face: bool = None
    previous_image = None
    new_image = None
    # _state: int = None
    _state: CompressedImage = None
    """
    For the sake of simplicity, the Subject's state, essential to all
    subscribers, is stored in this variable.
    """

    _observers: List[Observer] = []
    """
    List of subscribers. In real life, the list of subscribers can be stored
    more comprehensively (categorized by event type, etc.).
    """
    def __init__(self):
        x = threading.Thread(target=self.detectFace, args=())
        x.start()
    def addObserver(self, observer: Observer) -> None:
        print("Subject: Attached an observer.")
        self._observers.append(observer)

    def removeObserver(self, observer: Observer) -> None:
        self._observers.remove(observer)

    """
    The subscription management methods.
    """

    def notify(self) -> None:
        """
        Trigger an update in each subscriber.
        """

        # print("Subject: Notifying observers...")
        for observer in self._observers:
            observer.update(self)

    def setChanged(self, x_coordinate) -> None:
        """
        Usually, the subscription logic is only a fraction of what a Subject can
        really do. Subjects commonly hold some important business logic, that
        triggers a notification method whenever something important is about to
        happen (or after it).
        """

        # print("\nSubject: I'm doing something important.")
        # self._state = randrange(0, 10)
        self._state = x_coordinate

        # print(f"Subject: My state has just changed to: {self._state}")
        self.notify()

    def update(self, subject: Subject) -> None:
        # print("FaceCartographer: Received cameraPerception data")
        # self.detectFace(subject._state)
        self.new_image = subject._state

    def detectFace(self):        
        # print("Face detected from CameraPerception data: ", camera_input_data.header)
        while not killer.kill_now:
            if(self.previous_image != self.new_image):
                camera_input_data = self.new_image
                self.previous_image = self.new_image
                current_x=0
                np_arr = np.frombuffer(camera_input_data.data,np.uint8)
                image = cv2.imdecode(np_arr, 1)
                cv2.namedWindow("Image Window",1)
                with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detection:
                    results = face_detection.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    if results.detections:
                        for detection in results.detections:
                            nose=mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.NOSE_TIP)
                            current_x=nose.x
                cv2.imshow("Image Window",image)
                cv2.waitKey(1)
                self.setChanged(current_x) 
            time.sleep(0.01) 


class MirrorBehavior (Observer):
    coordinates = 0
    def __init__(self):
        print ("Mirror Behavior ",threading.current_thread())


    def update(self, subject: Subject) -> None:
        self.coordinates = subject._state
        self.motor_output()

    def motor_output(self):
        
        if self.coordinates<0.35:
            current_x=sb_motor_0.motor_pos[0]
            if current_x-6>=-12:
                sb_motor_0.move([current_x-6,sb_motor_0.motor_pos[1],sb_motor_0.motor_pos[2],sb_motor_0.motor_pos[3]] )
        elif self.coordinates>0.65:
            current_x=sb_motor_0.motor_pos[0]
            if current_x+6<=12:
                sb_motor_0.move([current_x+6,sb_motor_0.motor_pos[1],sb_motor_0.motor_pos[2],sb_motor_0.motor_pos[3]] )
        pass

        

    


class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self, *args):
    self.kill_now = True

killer = GracefulKiller()

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

twist = TwistStamped()
pause=True
my_time=time.time()
is_confused=0

dance_clock = time.time()

class Motor_Schema:
    global killer
    def __init__(self, sb_motor_publisher):
        print ("Motor Schema ",threading.current_thread())
        self.sb_motor_publisher = sb_motor_publisher
        self.motor_pos = [0,0,0,0]
        self.global_goal = [0,0,0,0]
        self.global_speed = 10
        x = threading.Thread(target=self.motor_control, args=())
        x.start()
        
        
    
    def move(self, goal, speed=globals.default_speed):
        self.global_goal = goal
        self.global_speed = speed
        # self.motor_control()
    def inProgress(self):
        return self.motor_pos != self.global_goal

    def motor_control(self):
            print ("motor control thread: ",threading.current_thread())
            while not killer.kill_now:
                
                transient_pos = self.motor_pos
                goal = self.global_goal
                speed = self.global_speed
                if(goal != self.motor_pos): 
                    if(goal[0]>self.motor_pos[0]):
                        transient_pos[0] = transient_pos[0]+speed
                        if(transient_pos[0]>goal[0]):
                            transient_pos[0] = goal[0]
                    elif(goal[0]<self.motor_pos[0]):
                        transient_pos[0] = transient_pos[0]-speed
                        if(transient_pos[0]<goal[0]):
                            transient_pos[0] = goal[0]
                    
                    if(goal[1]>self.motor_pos[1]):
                        transient_pos[1] = transient_pos[1]+speed
                        if(transient_pos[1]>goal[1]):
                            transient_pos[1] = goal[1]
                    elif(goal[1]<self.motor_pos[1]):
                        transient_pos[1] = transient_pos[1]-speed
                        if(transient_pos[1]<goal[1]):
                            transient_pos[1] = goal[1]

                    if(goal[2]>self.motor_pos[2]):
                        transient_pos[2] = transient_pos[2]+speed
                        if(transient_pos[2]>goal[2]):
                            transient_pos[2] = goal[2]
                    elif(goal[2]<self.motor_pos[2]):
                        transient_pos[2] = transient_pos[2]-speed
                        if(transient_pos[2]<goal[2]):
                            transient_pos[2] = goal[2]

                    if(goal[3]>self.motor_pos[3]):
                        transient_pos[3] = transient_pos[3]+speed
                        if(transient_pos[3]>goal[3]):
                            transient_pos[3] = goal[3]
                    elif(goal[3]<self.motor_pos[3]):
                        transient_pos[3] = transient_pos[3]-speed
                        if(transient_pos[3]<goal[3]):
                            transient_pos[3] = goal[3]
                    # print("Movement values: ",transient_pos)
                    self.motor_pos = transient_pos
                    
                    twist.twist.linear.x = self.motor_pos[0]
                    twist.twist.linear.y = self.motor_pos[1]
                    twist.twist.linear.z = self.motor_pos[2]
                    twist.twist.angular.x = self.motor_pos[3]
                    
                    self.sb_motor_publisher.publish(twist)
                time.sleep(0.1)  
                    


class DancePerformance():
    global killer, dance_clock
    
    def __init__(self, sb_motor):
        print ("Dance Performance ",threading.current_thread())
        # self.motors = motors
        self.danceSteps = []
        self.run = False
        self.freeStyle = FreeStyle(None)
        # self.sb_number = sb_number
        self.sb_motor = sb_motor
        # self.sb_motors = 0
        x = threading.Thread(target=self.execute, args=())
        x.start()
    
    

    def start_dance(self):
        global dance_clock
        dance_clock = 0
        self.run = True
        # self.execute()
    def addDanceStep(self, danceStep, time):
        self.danceSteps.append([danceStep, time])
        # self.sb_motors = danceStep.motors
    def addFreeStyle(self, freeStyleDanceStep):
        self.freeStyle = freeStyleDanceStep
        # self.sb_motors = freeStyleDanceStep.motors
    
    def execute(self):
        global dance_clock
        kill = killer.kill_now
        trigger = False
        # self.run = False
        while not self.run and not killer.kill_now:
            # time.sleep(0.1)
            pass
        if(killer.kill_now):
            return
        else:
            if(kill):
                return
        for actionStep in self.danceSteps:
            dance = actionStep[0]
            dance_release_time = actionStep[1]
            print("----------------------------------- Next step at: ",dance_release_time ," current time: ",dance_clock)
            while dance_clock < dance_release_time and not killer.kill_now:
                # random_buddy = random.randint(0,3)
                # self.freeStyle.perform()
                time.sleep(0.1)
                pass
                
            dance.perform()
        print("============================================ Thank you For Watching !!! ================================")
        print("                                                  Hope you Enjoyed                 ")
        
            

class DanceStep:
    global killer, dance_clock
    def __init__(self, motor):
        self.motor = motor
        self.steps = []
    def addMoves(self, step):
        self.steps.append(step)
    # def motion_in_progress(self):
    #     for motor in self.motors:
    #         if(motor.inProgress()):
    #             return True
    #     return False    
        
    def perform(self):
        count = 0
        for step in self.steps:
            print("..... performing step #",count, step );
            goal = step[0:4]
            speed = step[4]
            self.motor.move(goal, speed)
            count = count + 1
            while(self.motor.inProgress() and not killer.kill_now):
                time.sleep(0.1)
                pass

class FreeStyle(DanceStep):
    def perform(self):
        steps = [[10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5]]
        self.addMoves(steps)
        super().perform()
    # def perform(self, id):
        # if id>3:
        #     id = 0
        # steps = [[10, 0, 0, 20, 10], [5, 20, 0, 30, 5], [-10, 0, 0, 0, 15], [-5, 0, 0, 20, 5]]
        # for step in steps:
        #     goal = step[0:4]
        #     speed = step[4]
        #     self.motors[id].move(goal,speed)
        #     while(self.motion_in_progress() and not killer.kill_now):
        #         time.sleep(0.1)
        #         pass
        # for repeat in range(5):
        #     self.confused_random(id)
        

    def confused_random(self, id):
            rand = random.randint(0,40)
            goal = [0,0,0,0]
            goal[0] =  (-1)**rand*rand
            goal[1] =  (-1)**rand*rand
            goal[2] =  (-1)**rand*rand
            goal[3] =  (-1)**rand*rand
            
            print("random movement", goal)
            speed = random.randint(3,10)
            self.motors[id].move(goal, speed)

            goal = self.reverse_goal(goal)
            self.motors[id].move(goal, speed)

    def reverse_goal(self, goal):
        new_goal = [0,0,0,0]
        new_goal[0] = -goal[0]
        new_goal[0] = -goal[1]
        new_goal[0] = -goal[2]
        new_goal[0] = -goal[3]

        

class DanceBehavior(Observer):
    def __init__(self, sb_motor_0, sb_motor_1, sb_motor_2, sb_motor_3):
        
        self.sb0_dancePerformance = DancePerformance(sb_motor_0)
        self.sb1_dancePerformance = DancePerformance(sb_motor_1)
        self.sb2_dancePerformance = DancePerformance(sb_motor_2)
        self.sb3_dancePerformance = DancePerformance(sb_motor_3)
        
        
        rospy.loginfo("Node started.")
        self.group_name = "survivor_buddy_head"
        # x = threading.Thread(target=self.motors.motor_control(), args=())
        # x.start()
        self.dancePerformance()

    def update(self, subject: Subject) -> None:
        # print("is there a clap? ",subject._state)
        if(subject._state == True):
            print("...................................................................................DanceBehavior detected start signal")
            self.sb0_dancePerformance.start_dance()
            self.sb1_dancePerformance.start_dance()
            self.sb2_dancePerformance.start_dance()
            self.sb3_dancePerformance.start_dance()

    def dancePerformance(self):
        self.danceChoreograph0(self.sb0_dancePerformance)
        self.danceChoreograph1(self.sb1_dancePerformance)
        self.danceChoreograph2(self.sb2_dancePerformance)
        self.danceChoreograph3(self.sb3_dancePerformance)

        
    
        
    def danceChoreograph0(self, dancePerformance):
        simpleDanceSteps = [] 
        global steps1, steps2, steps3, steps4, side_b_1, side_b_reverse, side_t_1, side_t_reverse, side_w_1, side_w_reverse, body_t_1, body_t_reverse
        steps = []

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_reverse)

        steps.append(side_b_reverse)

        steps.append(side_b_reverse)
        # insert solo
        steps.append(steps2)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_1)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps1)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)
        
        #2x
        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_reverse)

        steps.append(side_b_reverse)

        steps.append(side_b_reverse)
        # insert solo
        steps.append(steps3)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_1)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps4)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)
        count = 1
        for step_set in steps:
            
            danceStep = DanceStep(dancePerformance.sb_motor)
            for eachMove in step_set:
                danceStep.addMoves(eachMove)
            simpleDanceSteps.append(danceStep)   

        for simpleStep in simpleDanceSteps:
            count = count +1
            if count == 20:
                dancePerformance.addDanceStep(simpleStep, sync_time)
            # if count == 6:
            #     dancePerformance.addDanceStep(simpleStep, solo_time)
            # if count == 7:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+4*delta)
            # else:
            dancePerformance.addDanceStep(simpleStep, 1)
        
    def danceChoreograph1(self, dancePerformance):
        simpleDanceSteps = [] 
        
        steps = []

        steps.append(side_w_1)  #1
        steps.append(side_w_1)  #2
        steps.append(side_t_reverse)    #3
        steps.append(side_t_1)    #4

        steps.append(side_b_1)          #5
        steps.append(side_b_reverse)    #6
        # insert solo
        steps.append(steps2)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_1)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps1)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        #2x
        steps.append(side_w_1)  #1
        steps.append(side_w_1)  #2
        steps.append(side_t_reverse)    #3
        steps.append(side_t_1)    #4

        steps.append(side_b_1)          #5
        steps.append(side_b_reverse)    #6
        # insert solo
        steps.append(steps3)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_1)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps4)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        count = 1
        for step_set in steps:
            
            danceStep = DanceStep(dancePerformance.sb_motor)
            for eachMove in step_set:
                danceStep.addMoves(eachMove)
            simpleDanceSteps.append(danceStep)   

        for simpleStep in simpleDanceSteps:
            count = count +1
            if count == 20:
                dancePerformance.addDanceStep(simpleStep, sync_time)
            # if count == 6:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+delta)
            # if count == 7:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+4*delta)
            # else:
            dancePerformance.addDanceStep(simpleStep, 1)

    def danceChoreograph2(self, dancePerformance):
        simpleDanceSteps = [] 
        
        steps = []

        steps.append(side_w_1)      #1
        steps.append(side_w_1)      #2
        steps.append(side_t_reverse)    #3
        steps.append(side_t_reverse)    #4

        steps.append(side_b_reverse)          #5
        steps.append(side_b_reverse)    #6
        # insert solo
        steps.append(steps2)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_reverse)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps1)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        #2x
        steps.append(side_w_1)      #1
        steps.append(side_w_1)      #2
        steps.append(side_t_reverse)    #3
        steps.append(side_t_reverse)    #4

        steps.append(side_b_reverse)          #5
        steps.append(side_b_reverse)    #6
        # insert solo
        steps.append(steps3)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_reverse)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps4)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        count = 1
        for step_set in steps:
            
            danceStep = DanceStep(dancePerformance.sb_motor)
            for eachMove in step_set:
                danceStep.addMoves(eachMove)
            simpleDanceSteps.append(danceStep)   

        for simpleStep in simpleDanceSteps:
            count = count +1
            if count == 20:
                dancePerformance.addDanceStep(simpleStep, sync_time)
            # if count == 6:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+2*delta)
            # if count == 7:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+4*delta)
            # else:
            dancePerformance.addDanceStep(simpleStep, 1)
        

    def danceChoreograph3(self, dancePerformance):
        simpleDanceSteps = [] 
        
        steps = []

        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        steps.append(side_b_reverse)
        # insert solo
        steps.append(steps2)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_reverse)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps1)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)
        
        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        # 2x
        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        steps.append(side_b_reverse)
        # insert solo
        steps.append(steps3)
        # solo end
        steps.append(body_t_1)
        steps.append(body_t_reverse)
        steps.append(neck_w_reverse)
        steps.append(neck_w_reverse)
        # solo 2
        steps.append(steps4)
        steps.append(neck_b_1)
        steps.append(neck_b_reverse)
        
        steps.append(side_w_1)
        steps.append(side_w_reverse)
        steps.append(side_t_1)
        steps.append(side_t_1)    #4
        steps.append(side_b_1)

        count = 1
        for step_set in steps:
            
            danceStep = DanceStep(dancePerformance.sb_motor)
            for eachMove in step_set:
                danceStep.addMoves(eachMove)
            simpleDanceSteps.append(danceStep)   

        for simpleStep in simpleDanceSteps:
            count = count +1
            if count == 20:
                dancePerformance.addDanceStep(simpleStep, sync_time)
            # if count == 7:
            #     dancePerformance.addDanceStep(simpleStep, solo_time+4*delta)
            # else:
            dancePerformance.addDanceStep(simpleStep, 1)


def clock_update(event):
    global dance_clock
    dance_clock = dance_clock + time_rate
    # print("                  clock updated", dance_clock)


if __name__ == '__main__':
    
    rospy.init_node("lab_1_node")
    moveit_commander.roscpp_initialize(sys.argv)
    #globals.initialize()
    sb_motor_publisher_0 = rospy.Publisher('/sb_0_cmd_state', TwistStamped, queue_size=1)
    sb_motor_publisher_1 = rospy.Publisher('/sb_1_cmd_state', TwistStamped, queue_size=1)
    sb_motor_publisher_2 = rospy.Publisher('/sb_2_cmd_state', TwistStamped, queue_size=1)
    sb_motor_publisher_3 = rospy.Publisher('/sb_3_cmd_state', TwistStamped, queue_size=1)

    sb_motor_0 = Motor_Schema(sb_motor_publisher_0);
    sb_motor_1 = Motor_Schema(sb_motor_publisher_1);
    sb_motor_2 = Motor_Schema(sb_motor_publisher_2);
    sb_motor_3 = Motor_Schema(sb_motor_publisher_3);

    
    
    talk = rospy.Publisher('/talker', String, queue_size=1)


    danceBehavior = DanceBehavior(sb_motor_0, sb_motor_1, sb_motor_2, sb_motor_3)
    clapCartographer = ClapCartographer()
    clapCartographer.addObserver(danceBehavior)

    cameraPerception = CameraPerception()
    cameraObserver = CameraObserver()
    cameraPerception.addObserver(cameraObserver)

    audioPerception = AudioPerception()
    audioPerception.addObserver(clapCartographer)

    

    # cameraPerception.setChanged(12345)

    # camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=buddy.callback_2, queue_size=1)
    camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=cameraPerception.setChanged, queue_size=1)
    audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback= audioPerception.setChanged, queue_size=1) 

    # globals.sb_motor_publisher_0 = rospy.Publisher('/sb_0_cmd_state', TwistStamped, queue_size=1)
    # globals.sb_motor_0 = Motor_Schema(globals.sb_motor_publisher_0);
    #Mirror Behavior Main Code
    mirrorBehavior = MirrorBehavior()
    faceCartographer = FaceCartographer()

    cameraPerception.addObserver(faceCartographer)
    faceCartographer.addObserver(mirrorBehavior)


    print ("main ",threading.current_thread())
    # init_pos = [0,0,0,0]
    # x = threading.Thread(target=self.motor_control, args=(global_goal, 1))
    # x.start()

    # motor_control(buddy, global_goal, 1);
    # killer = GracefulKiller()
    # motors = Motor_schema(buddy)
    # x = threading.Thread(target=motors.move, args=())
    # x.start()
    
    rospy.Timer(rospy.Duration(time_rate), clock_update)

    rospy.spin()
    # rospy.MultiThreadedSpinner spinner(4); # Use 4 threads
    # spinner.spin(); # spin() will not return until the node has been shutdown
