#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Import LEDPattern and ColorRGBA messages
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

# Define HSV range for object detection (example values)
lower_hsv = np.array([20, 100, 150])
upper_hsv = np.array([40, 255, 255])

class ObjectDetectionNode(DTROS):

    def __init__(self, node_name):

        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.bridge = CvBridge()
        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.image_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self.led_topic = f"/{self.vehicle_name}/led_emitter_node/led_pattern"  # Adjust based on your vehicle
        
        # Initialize LED control
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)
        self.led_segment = LedPatternSegment()

        # Subscribe to camera image topic
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

        # Change LED color to indicate object detection
        self.led_segment.color = (0, 255, 255)  # Set color to cyan (RGB: 0, 255,255)
        self.led_segment.publish_led_pattern(self.led_pub)

        # Sleep for 10 seconds
        rospy.loginfo("Stopping for 10 seconds due to object detection...")
        rospy.sleep(10.0)

        # Restore LED to default color after stopping
        self.led_segment.color = (255, 255, 255)  # Set back to default (white)
        self.led_segment.publish_led_pattern(self.led_pub)

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
        upper_left = hsv_image[:height//2, :width//2]
        upper_right = hsv_image[:height//2, width//2:]
        lower_left = hsv_image[height//2:, :width//2]
        lower_right = hsv_image[height//2:, width//2:]

        # Apply color filtering to each quadrant
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
            rospy.loginfo(rospy.get_caller_id() + " Object detected in one of the quadrants!")
            self.stop()  # Stop the robot
        else:
            rospy.loginfo(rospy.get_caller_id() + " No object detected in any quadrant.")
            self.move_forward()  # Move forward

    def on_shutdown(self):
        rospy.loginfo("Shutting down node")
        self.stop()  # Stop the robot explicitly
        rospy.sleep(0.5)  # Wait to ensure the stop command is received
        rospy.loginfo("Node shutdown complete")


class LedPatternSegment:
    def __init__(self, color=(255, 255, 255), brightness=100):
        self.color = color  # RGB color tuple (default: white)
        self.brightness = brightness  # Brightness level (default: 100%)

    def publish_led_pattern(self, pub):
        # Create LEDPattern message
        led_msg = LEDPattern()

        # Set RGB color values (assuming ColorRGBA format)
        rgba = ColorRGBA()
        rgba.r = self.color[0] / 255.0
        rgba.g = self.color[1] / 255.0
        rgba.b = self.color[2] / 255.0
        rgba.a = self.brightness / 100.0

        # Add the same color to all LEDs 
        for _ in range(5):
            led_msg.rgb_vals.append(rgba)

        # Publish LEDPattern message
        pub.publish(led_msg)
        rospy.loginfo(f"Published LED pattern with color {self.color} and brightness {self.brightness}%")


if __name__ == '__main__':
    node = ObjectDetectionNode(node_name='object_detection_node')
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
