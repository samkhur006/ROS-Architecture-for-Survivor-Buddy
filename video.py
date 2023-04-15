import cv2
import mediapipe as mp
import numpy as np
import math
import time

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

def get_dist(a,b):
    return math.sqrt( (a.x-b.x)**2 +(a.y-b.y)**2   )

def execute_behavior4(a):
    bool=False
    if a==1:
        globals.twist.twist.linear.y=-35
    elif a==2:
        globals.twist.twist.linear.y=35
    else:
        globals.twist.twist.linear.x= 0
        globals.twist.twist.linear.y = 0
        globals.twist.twist.linear.z = 0
        globals.twist.twist.angular.x = 0
    self.sampub.publish(globals.twist)
    time.sleep(0.5)
    if a==1:
        execute_behavior4(2)
    elif a==2:
        execute_behavior4(3)


def execute_behavior3(a):
    #print("ENTER with  ",a)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    # group_name = "survivor_buddy_head"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    # planning_frame = move_group.get_planning_frame()
        

        # joint value planning
    bool=False
    # joint_goal = move_group.get_current_joint_values()
# print("cur ",joint_goal[1])


    if a==False:                                                    # CHECK
        if globals.twist.twist.linear.y + 14 <=45 :
            globals.twist.twist.linear.y=globals.twist.twist.linear.y + 14
            bool=True
            print("MOVING ")
    else:
        if globals.twist.twist.linear.y - 14 >=-45 :
            globals.twist.twist.linear.y=globals.twist.twist.linear.y - 14
            bool=True
            print("MOVING ")

    self.sampub.publish(globals.twist)

#     move_group.go(joint_goal, wait=True)
#     plan = move_group.plan()
#     move_group.stop()

#     display_trajectory = DisplayTrajectory()
#     display_trajectory.trajectory_start = robot.get_current_state()
#     pub.publish(display_trajectory)
#      execute plan
#     move_group.execute(plan[1], wait=True)
    return bool




def execute_behavior2(a):
    #print("ENTER with  ",a)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    bool =False
    # group_name = "survivor_buddy_head"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    # planning_frame = move_group.get_planning_frame()
        

        # joint value planning
    # joint_goal = move_group.get_current_joint_values()
    

    if a:                   #Check
        if globals.twist.twist.linear.x - 6 >=-18 :
            globals.twist.twist.linear.x-=6
            bool = True
        else:
            globals.twist.twist.linear.x=0
            cur=0
    else:
        if globals.twist.twist.linear.x + 6 <=0 :
            globals.twist.twist.linear.x += 6
            bool=True
        else:
            print("Can't move back ",twist.twist.linear.x + 14 )

    
    self.sampub.publish(globals.twist)
    


#     move_group.go(joint_goal, wait=True)
#     plan = move_group.plan()
#     move_group.stop()

#   #  display_trajectory = DisplayTrajectory()
#    # display_trajectory.trajectory_start = robot.get_current_state()
#     #pub.publish(display_trajectory)
#     # execute plan
#     move_group.execute(plan[1], wait=True)
    return bool

def execute_behavior1(a):
    #print("ENTER with  ",a)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    # group_name = "survivor_buddy_head"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    # planning_frame = move_group.get_planning_frame()
        

        # joint value planning
    if( a ):
        globals.twist.twist.linear.x = 5
        globals.twist.twist.linear.y = 5
        #twist.twist.linear.z = 6
        #twist.twist.angular.x = -10
    else:
        globals.twist.twist.linear.x= 0
        globals.twist.twist.linear.y = 0
        globals.twist.twist.linear.z = 0
        globals.twist.twist.angular.x = 0

    self.sampub.publish(globals.twist)
    time.sleep(0.5)

    #move_group.go(joint_goal, wait=True)
    #plan = move_group.plan()
    #move_group.stop()

    # display_trajectory = DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    #pub.publish(display_trajectory)
    # execute plan
    #move_group.execute(plan[1], wait=True)
    #print("a is ",a)
    if a:
        execute_behavior1(False)


def callback_2( data):
    print("entered camera")
    np_arr = np.frombuffer(data.data,np.uint8)
    image = cv2.imdecode(np_arr, 1)
    if globals.last_time==-1:
        globals.last_time=int(time.time())
    cv2.namedWindow("Image Window",1)

    with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detection:
        results = face_detection.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if results.detections:
            annotated_image = image.copy()
            for detection in results.detections:
                #  print('Nose tip:')
                if globals.sound==False:
                    return
                globals.last_time=int(time.time())
                globals.confused=True
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

                if globals.cur==0 and new_dist>=0.19 and new_dist<=0.42:
                    if execute_behavior2(True):
                        print("FORWARD")
                        globals.cur=globals.cur+1 
                elif globals.cur==1 and new_dist>=0.23 :
                    if execute_behavior2(True):
                        print("FORWARD")
                        globals.cur=globals.cur+1 
                elif globals.cur==2 and new_dist>=0.27:
                    if execute_behavior2(True):
                        print("FORWARD")
                        globals.cur=globals.cur+1 
                elif globals.cur==3 and new_dist>=0.3:
                    if execute_behavior2(True):
                        print("TOO CLOSE!!")
                        globals.cur=0
                elif globals.cur==3 and new_dist<=0.26:
                    if execute_behavior2(False):
                        print("BACKWARD")
                        globals.cur=globals.cur-1
                elif globals.cur==2 and new_dist<=0.22:
                    if execute_behavior2(False):
                        print("BACKWARD")
                        globals.cur=globals.cur-1
                elif globals.cur==1 and new_dist<=0.19:
                    if execute_behavior2(False):
                        print("BACKWARD")
                        globals.cur=globals.cur-1 


                if(globals.an_cur==0 and nose.x<0.35):
                    execute_behavior3(True)
                    globals.an_cur=1
                    print("You MOVED   RIGHT")
                if(globals.an_cur==0 and nose.x>0.65):
                    print("You MOVED   LEFT")
                    execute_behavior3(False)
                    globals.an_cur=1
                
                        
                
                if(globals.an_cur>0):
                    globals.an_cur+=1

                if(globals.an_cur==10):
                    globals.an_cur=0


                #mp_drawing.draw_detection(annotated_image, detection)               //COMMENTED THESE 2 lines
            #cv2.imwrite('/tmp/annotated_image'  + '.png', annotated_image)
            
        elif globals.confused and time.time()-int(globals.last_time)>=2:
            print("CONFUSED")
            execute_behavior4(1)
            globals.last_time=int(time.time())
            globals.confused=False
            

    cv2.imshow("Image Window",image)
    cv2.waitKey(3)
    pass
