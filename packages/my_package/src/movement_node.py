#!/usr/bin/env python3

import os
import rospy 
from std_msgs.msg import String # type: ignore
from duckietown.dtros import DTROS, NodeType # type: ignore
from duckietown_msgs.msg import WheelsCmdStamped # type: ignore

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.pub_wheels_cmd = rospy.Publisher('/duckiegpt/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
#Where you see the value duckiegpt you will need to put your duckiebot name

    def moveSimple(self):
        # Example sequence of actions using Duckietown's robot control
        self.publish_message("I'm ready to hit the road!")
        rospy.sleep(2) # These values can be changed depending on how long you want it to publish for...
        self.publish_message("Moving forward...")
        self.move_forward()
        rospy.sleep(10)  # This will have it run of 10 seconds before publishing the next message
        self.publish_message("Moving backwards...")
        self.move_backwards()
        rospy.sleep(10)  # Example of pusblishing for 100 seconds 
        self.publish_message("Turning right...")
        self.turn_right()
        rospy.sleep(0.5)
        self.publish_message("Stopping...")
        self.stop_robot()

    def move_forward(self):
        # Example of publishing a command to move forward
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.3 #left wheel velocity 
        cmd.vel_right = 0.3 #right wheel velocity 
        self.pub_wheels_cmd.publish(cmd)

    def move_backwards(self):
        # Example of publishing a command to move forward
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = -0.3 #The velocities are now negtive because it is moving backwards 
        cmd.vel_right = -0.3
        self.pub_wheels_cmd.publish(cmd)

    def turn_right(self):
        # Example of publishing a command to turn right
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.05 #When you are turning one value needs to be postive and the other needs to be negtive 
        cmd.vel_right = -0.05 #This will need to be adjusted based on floor texture. 
        self.pub_wheels_cmd.publish(cmd)

    def stop_robot(self):
        # Example of publishing a command to stop the robot
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.0 #velocity of zero because we are stopped
        cmd.vel_right = 0.0
        self.pub_wheels_cmd.publish(cmd)

    def publish_message(self, message):
        # Helper method to publish messages
        rospy.loginfo(message)
        msg = String(data=message)
        self.pub.publish(msg)

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.moveSimple()
    rospy.spin()
