#!/usr/bin/env python3

import rospy
from manos.msg import gesture
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState


class MoveNode:                                                                                             #Move class
    def __init__(self):
        rospy.init_node('manual_movement')                                                                       #Node registry
        rospy.Subscriber('/type_of_gesture', gesture, self.callback)                                     #Suscribe to the distance topic
        # self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)                                         ###Create the publisher in "/cmd_vel" topic for real turtlebot '/mobile_base/commands/velocity'
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)                                         ###Create the publisher in "/cmd_vel" topic for real turtlebot '/mobile_base/commands/velocity'
        self.pub_pinza = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=5)                                         ###Create the publisher in "/cmd_vel" topic for real turtlebot '/mobile_base/commands/velocity'

        self.message = Twist()                                                                              #Create the message
        self.message_pinza = JointState()

        rospy.spin()                                                                                        #Repeat

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

        if(data.type_gesture=="Delante"):
            self.message.linear.x = 0.2
        elif(data.type_gesture=="Izquierda"):
            self.message.angular.z = 0.5
        elif(data.type_gesture=="Derecha"):
            self.message.angular.z = -0.5
        elif(data.type_gesture=="Coger"):
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

        self.pub.publish(self.message)                                                                      #Publish message

if __name__ == '__main__':
    MoveNode()                                                                                              #Call the Move node
    


