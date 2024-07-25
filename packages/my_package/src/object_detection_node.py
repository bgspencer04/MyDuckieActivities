#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Define HSV range for object detection 
# These values can be changed based on what color you what to detect
# These values are set for bright yellow
lower_hsv = np.array([20, 100, 150])
upper_hsv = np.array([40, 255, 255])

class ObjectDetectionNode(DTROS):

    def __init__(self, node_name):

        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.bridge = CvBridge()
        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.image_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
    
        self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)
        self.cmd_pub = rospy.Publisher(self.twist_topic, Twist2DStamped, queue_size=10)

        self.twist_msg = Twist2DStamped()

        rospy.loginfo(f"{node_name} initialized")

    def preprocess(self, image_rgb):
        """Preprocesses the RGB image to apply object detection"""
        hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
        return hsv

    def move_forward(self):
        self.twist_msg.v = 0.2  # Adjust forward velocity as needed
        self.twist_msg.omega = 0.0
        self.cmd_pub.publish(self.twist_msg)

    def stop(self):
        self.twist_msg.v = 0.0  # Stop the robot
        self.twist_msg.omega = 0.0
        self.cmd_pub.publish(self.twist_msg)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        hsv_image = self.preprocess(cv_image)
        height, width, _ = hsv_image.shape

        # Divide the image into four quadrants
        # This allows you to detect in one or all of the quadrants 
        upper_left = hsv_image[:height//2, :width//2]
        upper_right = hsv_image[:height//2, width//2:]
        lower_left = hsv_image[height//2:, :width//2]
        lower_right = hsv_image[height//2:, width//2:]

        # Apply color filtering to each quadrant
        # this is filtering everything out of the image other than the color you defined 
        mask_ul = cv2.inRange(upper_left, lower_hsv, upper_hsv)
        mask_ur = cv2.inRange(upper_right, lower_hsv, upper_hsv)
        mask_ll = cv2.inRange(lower_left, lower_hsv, upper_hsv)
        mask_lr = cv2.inRange(lower_right, lower_hsv, upper_hsv)

        # Compute sums of pixels in each quadrant
        sum_ul = np.sum(mask_ul)
        sum_ur = np.sum(mask_ur)
        sum_ll = np.sum(mask_ll)
        sum_lr = np.sum(mask_lr)

        # Define threshold value for detecting objects in any quadrant
        threshold = 2000  # Adjust as needed based on your specific environment
        
        if sum_ul > threshold or sum_ur > threshold or sum_ll > threshold or sum_lr > threshold:
            rospy.loginfo(rospy.get_caller_id() + " Object detected!")
            self.stop()  # Stop the duckiebot 
        else:
            rospy.loginfo(rospy.get_caller_id() + " No object detected.. continuing straight")
            self.move_forward()  # Move forward

    def on_shutdown(self):
        rospy.loginfo("Shutting down node")
        self.stop()  # Stop the robot explicitly
        rospy.sleep(0.5)  # Wait to ensure the stop command is received
        rospy.loginfo("Node shutdown complete")


if __name__ == '__main__':
    node = ObjectDetectionNode(node_name='object_detection_node')
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
