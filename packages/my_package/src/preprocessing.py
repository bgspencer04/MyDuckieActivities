#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Image, CompressedImage

# Define HSV range for object detection (example values)
lower_hsv = np.array([0, 0, 70])
upper_hsv = np.array([50, 155, 255])

class RobotController:
    def __init__(self):
        rospy.init_node("object_detection_node")
        self.pub_wheels_cmd = rospy.Publisher('/duckiegpt/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.sub_camera = rospy.Subscriber("/duckiegpt/camera_node/image/compressed", CompressedImage, self.callback)
        self.move = WheelsCmdStamped()

    def preprocess(self, image_rgb):
        """Preprocesses the RGB image to apply object detection"""
        hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        return mask

    def callback(self, image_msg):
        try:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            image_rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            mask = self.preprocess(image_rgb)
            
            # Compute the sum of pixels in different regions of the mask
            left_sum = np.sum(mask[:, :mask.shape[1] // 2])
            right_sum = np.sum(mask[:, mask.shape[1] // 2:])
            front_sum = np.sum(mask[mask.shape[0] // 3 : 2 * mask.shape[0] // 3, :])

            # Define threshold values based on mask pixel sums
            threshold = 5000  # Adjust as needed based on your specific environment

            if left_sum > threshold and left_sum > right_sum and left_sum > front_sum:
                rospy.loginfo(rospy.get_caller_id() + " Object detected on the left!")
                self.turn_right()
            elif right_sum > threshold and right_sum > left_sum and right_sum > front_sum:
                rospy.loginfo(rospy.get_caller_id() + " Object detected on the right!")
                self.turn_left()
            elif front_sum > threshold:
                rospy.loginfo(rospy.get_caller_id() + " Object detected in front!")
                self.stop()
            else:
                rospy.loginfo(rospy.get_caller_id() + " No object detected.")
                self.move_forward()

        except Exception as e:
            rospy.logerr("Error processing image: {}".format(str(e)))

    def move_forward(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.3  # Adjust velocities as needed
        msg.vel_right = 0.3
        self.pub_wheels_cmd.publish(msg)

    def stop(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheels_cmd.publish(msg)

    def turn_left(self):
        msg = WheelsCmdStamped()
        msg.vel_left = -0.3  # Adjust velocities as needed for turning left
        msg.vel_right = 0.3
        self.pub_wheels_cmd.publish(msg)

    def turn_right(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.3
        msg.vel_right = -0.3  # Adjust velocities as needed for turning right
        self.pub_wheels_cmd.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    controller = RobotController()
    controller.spin()
