#!/usr/bin/env python3

from pynput import keyboard
import roslaunch
import rospy
import actionlib
import tf
from manos.msg import gesture
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from guizero import App, Text, PushButton
import time
import re
import cv2
# print(cv2.__version__)
import numpy as np
import mediapipe as mp

width=1280
height=720
option = False

class mpHands:
    def __init__(self,maxHands=2,tol1=.5,tol2=.5):
       self.hands=mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=maxHands,min_detection_confidence=tol1,min_tracking_confidence=tol2)
    def Marks(self,frame):
        myHands=[]
        frameRGB=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        results=self.hands.process(frameRGB)
        if results.multi_hand_landmarks != None:
            for handLandMarks in results.multi_hand_landmarks:
                myHand=[]
                for landMark in handLandMarks.landmark:
                    myHand.append((int(landMark.x*width),int(landMark.y*height)))
                myHands.append(myHand)
        return myHands
def findDistances(handData):
    distMatrix=np.zeros([len(handData),len(handData)],dtype='float')
    palmSize=((handData[0][0]-handData[9][0])**2+(handData[0][1]-handData[9][1])**2)**(1./2.)
    for row in range(0,len(handData)):
        for column in range(0,len(handData)):
            distMatrix[row][column]=(((handData[row][0]-handData[column][0])**2+(handData[row][1]-handData[column][1])**2)**(1./2.))/palmSize
    return distMatrix

def findError(gestureMatrix,unknownMatrix,keyPoints):
    error=0
    for row in keyPoints:
        for column in keyPoints:
            error=error+abs(gestureMatrix[row][column]-unknownMatrix[row][column])
    print(error)
    return error
def findGesture(unknownGesture,knownGestures,keyPoints,gestNames,tol):
    errorArray=[]
    for i in range(0,len(gestNames),1):
        error=findError(knownGestures[i],unknownGesture,keyPoints)
        errorArray.append(error)
    errorMin=errorArray[0]
    minIndex=0
    for i in range(0,len(errorArray),1):
        if errorArray[i]<errorMin:
            errorMin=errorArray[i]
            minIndex=i
    if errorMin<tol:
        gesture=gestNames[minIndex]
    if errorMin>=tol:
        gesture='Unknown'
    return gesture

class modes:
    def __init__(self):
        self.arucos = []
    def manual(self):
        print(f'Modo manual seleccionado.')

        rospy.init_node('manual_mode', anonymous=True)
        pub = rospy.Publisher('/type_of_gesture', gesture, queue_size=5)
        message = gesture() 

        cam=cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        cam.set(cv2.CAP_PROP_FPS, 30)
        cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        findHands= mpHands()
        time.sleep(5)
        keyPoints=[0,4,5,9,13,17,8,12,16,20]
        train=True
        tol=10
        trainCnt=0
        knownGestures=[]

        # numGest=int(input('How Many Gestures Do You Want? '))
        numGest = 5
        gestNames=['Delante', 'Izquierda', 'Derecha', 'Coger', 'Dejar']

        # for i in range(0,numGest,1):
        #     prompt='Name of Gesture #'+str(i+1)+' '
        #     name=input(prompt)
        #     gestNames.append(name)
        # print(gestNames)

        while (not rospy.is_shutdown()):
            ignore,  frame = cam.read()
            frame=cv2.resize(frame,(width,height))
            frame = cv2.flip(frame, 2)
            handData=findHands.Marks(frame)
            if train==True:
                if handData!=[]:
                    print('Please Show Gesture ',gestNames[trainCnt],': Press t when Ready')
                    if cv2.waitKey(1) & 0xff==ord('t'):
                        knownGesture=findDistances(handData[0])
                        knownGestures.append(knownGesture)
                        trainCnt=trainCnt+1
                        if trainCnt==numGest:
                            train=False
            if train == False:
                if handData!=[]:
                    unknownGesture=findDistances(handData[0])
                    myGesture=findGesture(unknownGesture,knownGestures,keyPoints,gestNames,tol)
                    message.type_gesture = myGesture
                    pub.publish(message) 
                    #error=findError(knownGesture,unknownGesture,keyPoints)
                    cv2.putText(frame,myGesture,(100,175),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),8)
            for hand in handData:
                for ind in keyPoints:
                    cv2.circle(frame,hand[ind],25,(255,0,255),3)
            cv2.imshow('my WEBcam', frame)
            cv2.moveWindow('my WEBcam',0,0)
            if cv2.waitKey(1) & 0xff ==ord('q'):
                break
        cam.release()

    def add_aruco(self, aruco):
        change = False
        if(len(self.arucos)==0):
            self.arucos.append(aruco)
            print("ARUCO ADDED")
        else:
            for j in range(len(self.arucos)):
                if(self.arucos[j]['id'] == aruco['id']):
                    # self.arucos[j]['pos_x'] = aruco['pos_x']
                    # self.arucos[j]['pos_y'] = aruco['pos_y']
                    # self.arucos[j]['pos_z'] = aruco['pos_z']
                    change = True
                        
            if(change == False):
                self.arucos.append(aruco)
                print("ARUCO ADDED")

    def get_aruco_data(self, data):
        if(len(data.transforms)>0):
            for i in range(len(data.transforms)):
                #print(f"id: {data.transforms[0].child_frame_id}; posicion en x: {data.transforms[0].transform.translation.x}; posicion en y: {data.transforms[0].transform.translation.y}; posicion en z {data.transforms[0].transform.translation.z}")
                txt = str(data.transforms[0].child_frame_id)
                id = [int(s) for s in re.findall('[0-9]+', txt)]
                # ea = self.listener.canTransform('map', txt, rospy.Time().now())
                # print(f'{ea}')
                # if(ea==0):
                # self.listener.waitForTransform('map', txt, rospy.Time(0), rospy.Duration(0.5))
                # (trans,rot) = self.listener.lookupTransform('map', txt, rospy.Time(0))

                new_aruco = dict({'id': id[0], 'pos_x': self.robot_pos_x + data.transforms[0].transform.translation.z, 'pos_y': self.robot_pos_y - data.transforms[0].transform.translation.x, 'pos_z': self.robot_pos_z - data.transforms[0].transform.translation.y, 'orient_x': 0.0, 'orient_y': 0.0, 'orient_z': 0.0, 'orient_w': 1.0})
                
                self.add_aruco(new_aruco)

    def save_aruco_data(self,data):
        if(len(data.transforms)>0):
            for i in range(len(data.transforms)):
                #print(f"id: {data.transforms[0].child_frame_id}; posicion en x: {data.transforms[0].transform.translation.x}; posicion en y: {data.transforms[0].transform.translation.y}; posicion en z {data.transforms[0].transform.translation.z}")
                txt = str(data.transforms[0].child_frame_id)
                id = [int(s) for s in re.findall('[0-9]+', txt)]

                self.new_aruco = dict({'id': id[0], 'pos_x': self.robot_pos_x, 'pos_y': self.robot_pos_y, 'pos_z': self.robot_pos_z, 'orient_x': self.robot_orient_x, 'orient_y': self.robot_orient_y, 'orient_z': self.robot_orient_z, 'orient_w': self.robot_orient_w})
                

    def movebase_client(self,aruco):

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the axis of the "map" coordinate frame 
        goal.target_pose.pose.position.x = aruco['pos_x']
        goal.target_pose.pose.position.y = aruco['pos_y']

        goal.target_pose.pose.orientation.x = aruco['orient_x']
        goal.target_pose.pose.orientation.y = aruco['orient_y']
        goal.target_pose.pose.orientation.z = aruco['orient_z']
        goal.target_pose.pose.orientation.w = aruco['orient_w']
    # No rotation of the mobile base frame w.r.t. map frame
        # goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
        client.send_goal(goal)
    # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return client.get_result()   
        return
    def robot_position(self,data):

        self.robot_pos_x = data.pose.pose.position.x
        self.robot_pos_y = data.pose.pose.position.y    
        self.robot_pos_z = 0 
        
        self.robot_orient_x = data.pose.pose.orientation.x
        self.robot_orient_y = data.pose.pose.orientation.y    
        self.robot_orient_z = data.pose.pose.orientation.z
        self.robot_orient_w = data.pose.pose.orientation.w  

    def myFunc(self,e):
        return e['id']

    def auto(self):
        
        print(f'Modo automático eleccionado.')
        
        rospy.init_node('auto_mode', anonymous=True)
        # self.listener = tf.TransformListener()
        # rospy.sleep(5)
        rospy.Subscriber('odom',Odometry, self.robot_position)
        rospy.Subscriber("/tf_list", TFMessage, self.get_aruco_data)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        file_path="/home/zuleikarg/ros/src/frontier_exploration/launch/explore_costmap.launch"
        launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
        launch.start()

        # The event listener will be running in this block
        with keyboard.Events() as events:
            for event in events:
                if event.key == keyboard.Key.esc:
                    launch.shutdown()
                    break

        print(f"FINISHED: {self.arucos}")

        base_found = False
        for i in range(len(self.arucos)):
            if(self.arucos[i]['id']==0):
                base_found = True

        if(base_found == True):
            self.arucos.sort(key=self.myFunc)
            print(f"SORTED: {self.arucos}")

            new_order = []

            for j in range (len(self.arucos)-1):
                new_order.append(self.arucos[j+1])
                new_order.append(self.arucos[0])

            print(f"FINAL ORDER: {new_order}")
            for z in range(len(new_order)):
                try:
                    result = self.movebase_client(new_order[z])
                    if result:
                        rospy.loginfo("Goal execution done!")
                        rospy.sleep(2)

                except rospy.ROSInterruptException:
                    rospy.loginfo("Navigation test finished.")
        else:
            rospy.loginfo("Base Aruco was not found.")

    def teleop(self):
        print(f'Modo mapeo con Teleoperación.')
        
        rospy.init_node('teleop_mode', anonymous=True)
        # self.listener = tf.TransformListener()
        # rospy.sleep(5)
        rospy.Subscriber('odom',Odometry, self.robot_position)
        rospy.Subscriber("/tf_list", TFMessage, self.save_aruco_data)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        file_path="/home/zuleikarg/ros/src/turtlebot3/turtlebot3_teleop/launch/turtlebot3_teleop_key.launch"
        launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
        launch.start()

        # The event listener will be running in this block
        with keyboard.Events() as events:
            for event in events:
                if (event.key == keyboard.Key.esc):
                    launch.shutdown()
                    break
                elif (event.key == keyboard.Key.space):
                    self.add_aruco(self.new_aruco)
                    

        print(f"FINISHED: {self.arucos}")

        base_found = False
        for i in range(len(self.arucos)):
            if(self.arucos[i]['id']==0):
                base_found = True

        if(base_found == True):
            self.arucos.sort(key=self.myFunc)
            print(f"SORTED: {self.arucos}")

            new_order = []

            for j in range (len(self.arucos)-1):
                new_order.append(self.arucos[j+1])
                new_order.append(self.arucos[0])

            print(f"FINAL ORDER: {new_order}")
            for z in range(len(new_order)):
                try:
                    result = self.movebase_client(new_order[z])
                    if result:
                        rospy.loginfo("Goal execution done!")
                        rospy.sleep(2)

                except rospy.ROSInterruptException:
                    rospy.loginfo("Navigation test finished.")
        else:
            rospy.loginfo("Base Aruco was not found.")

def choose_manual():
    global option
    option = 0
    app.destroy()

def choose_auto():
    global option
    option = 1
    app.destroy()

def choose_teleop():
    global option
    option = 2
    app.destroy()

if __name__ == '__main__':

    app = App(title="wall-e",width=500,height=500)

    modelo = modes()
    welcome_message = Text(app, text="Elige el modo de funcionamiento:")
    update_text = PushButton(app, command=choose_teleop, text="Modo automático con mapeo Teleoperado",pady=30,padx=500,align="bottom")
    update_text = PushButton(app, command=choose_manual, text="Modo Manual",pady=200,padx=80,align="left")
    update_text = PushButton(app, command=choose_auto, text="Modo Automático",pady=200,padx=80,align="right")
    app.display()

    if(option == 0):
        modelo.manual()
    elif(option == 1):
        modelo.auto()
    else:
        modelo.teleop()
    
    print("")
    print("PROGRAM FINISHED")