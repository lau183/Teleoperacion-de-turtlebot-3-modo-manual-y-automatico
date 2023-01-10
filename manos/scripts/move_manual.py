#!/usr/bin/env python3

import rospy
from manos.msg import gesture
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState

#MoveNode class
class MoveNode:
    def __init__(self):
        rospy.init_node('manual_movement')                                                                                                      #Node registry
        rospy.Subscriber('/type_of_gesture', gesture, self.callback)                                                                            #Suscribe to the distance topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)                                                                           ###Create the publisher in "/cmd_vel" topic for simulation
        # self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)                                                     ###Create the publisher in "/mobile_base/commands/velocity" topic for real turtlebot
        self.pub_pinza = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=5)                                ###Create the publisher in "/move_group/fake_controller_joint_states" topic for simulation 
        # self.pub_pinza = rospy.Publisher('/phantomx_reactor_controller/joint_command', JointState, queue_size=5)                              ###Create the publisher in "/phantomx_reactor_controller/joint_command" topic for real turtlebot 

        #Mesagges for movements of the turtlebot and the arm
        self.message = Twist()                                                                                                                  #Create the message
        self.message_pinza = JointState()

        rospy.spin()                                                                                                                            #Repeat

    #When a gesture is recived
    def callback(self,data):
        #Initialize the message
        self.message.linear.x = 0
        self.message.linear.y = 0
        self.message.linear.z = 0

        self.message.angular.x = 0
        self.message.angular.y = 0
        self.message.angular.z = 0

        self.message_pinza.header.frame_id = 'base_link'
        self.message_pinza.name = ['']
        self.message_pinza.position = [0]
        self.message_pinza.velocity = [0]
        self.message_pinza.effort = [0]

        #Set the value of the movement based on the gesture recived
        if(data.type_gesture=="Delante"):
            self.message.linear.x = 0.2
        elif(data.type_gesture=="Izquierda"):
            self.message.angular.z = 0.5
        elif(data.type_gesture=="Derecha"):
            self.message.angular.z = -0.5
        elif(data.type_gesture=="Coger"):
            #Repeat in order to set the values for all the joints
            for i in range(5):
                if(i==0):
                    self.message_pinza.name = ['shoulder_yaw_joint']
                    self.message_pinza.position = [0]
                if(i==1):
                    self.message_pinza.name = ['shoulder_pitch_joint']
                    self.message_pinza.position = [-0.5]
                if(i==2):
                    self.message_pinza.name = ['elbow_pitch_joint']
                    self.message_pinza.position = [0]
                if(i==3):
                    self.message_pinza.name = ['wrist_roll_joint']
                    self.message_pinza.position = [0]
                if(i==4):
                    self.message_pinza.name = ['wrist_pitch_joint']
                    self.message_pinza.position = [0.5]
                self.pub_pinza.publish(self.message_pinza) 
        elif(data.type_gesture=="Dejar"):
            #Repeat in order to set the values for all the joints
            for i in range(5):
                if(i==0):
                    self.message_pinza.name = ['shoulder_yaw_joint']
                    self.message_pinza.position = [0]
                if(i==1):
                    self.message_pinza.name = ['shoulder_pitch_joint']
                    self.message_pinza.position = [-1.57]
                if(i==2):
                    self.message_pinza.name = ['elbow_pitch_joint']
                    self.message_pinza.position = [1.57]
                if(i==3):
                    self.message_pinza.name = ['wrist_roll_joint']
                    self.message_pinza.position = [0]
                if(i==4):
                    self.message_pinza.name = ['wrist_pitch_joint']
                    self.message_pinza.position = [0.7]
                self.pub_pinza.publish(self.message_pinza) 

        self.pub.publish(self.message)                                                                                                                      #Publish message

if __name__ == '__main__':
    MoveNode()                                                                                                                                              #Call the Move node
    


