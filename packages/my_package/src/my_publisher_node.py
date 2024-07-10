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

    def moveSimple(self):
        # Example sequence of actions using Duckietown's robot control
        self.publish_message("I'm ready to hit the road!")
        rospy.sleep(2)
        self.publish_message("Moving forward...")
        self.move_forward()
        rospy.sleep(10)  # Example of waiting for 2 seconds
        self.publish_message("Moving backwards...")
        self.move_backwards()
        rospy.sleep(10)  # Example of waiting for 2 seconds
        self.publish_message("Turning right...")
        self.turn_right()
        rospy.sleep(0.5)
        self.publish_message("Stopping...")
        self.stop_robot()

    def move_forward(self):
        # Example of publishing a command to move forward
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.3
        cmd.vel_right = 0.3
        self.pub_wheels_cmd.publish(cmd)

    def move_backwards(self):
        # Example of publishing a command to move forward
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = -0.3
        cmd.vel_right = -0.3
        self.pub_wheels_cmd.publish(cmd)

    def turn_right(self):
        # Example of publishing a command to turn right
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.05
        cmd.vel_right = -0.05
        self.pub_wheels_cmd.publish(cmd)

    def stop_robot(self):
        # Example of publishing a command to stop the robot
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.0
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
