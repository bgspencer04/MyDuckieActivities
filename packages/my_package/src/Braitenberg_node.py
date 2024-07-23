#!/usr/bin/env python3

import os
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError

@dataclass
class BraitenbergAgentConfig:
    gain: float = 0.9
    const: float = 0.0

class BraitenbergAgent:
    def __init__(self):
        self.config = BraitenbergAgentConfig()
        self.left = None
        self.right = None
        self.rgb = None
        self.l_max = -math.inf
        self.r_max = -math.inf
        self.l_min = math.inf
        self.r_min = math.inf
        self.bridge = CvBridge()
        self.vehicle_name = os.environ['VEHICLE_NAME']
        self.image_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
        self.twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"

        # Initialize ROS node
        rospy.init_node('braitenberg_agent')
        rospy.loginfo("Braitenberg Agent node initialized.")

        rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)
        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.encoder_callback)
        self.cmd_pub = rospy.Publisher(self.twist_topic, Twist2DStamped, queue_size=10)

        # Initialize movement
        self.init_movement()

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Subscribers and Publisher setup complete. Starting ROS spin.")
        rospy.spin()

    def init_movement(self):
        rospy.loginfo("Initializing movement (e.g., moving forward).")
        # Example: Publish a Twist command to move forward
        twist_msg = Twist2DStamped(v=0.5, omega=0.0)  # Adjust speed as needed
        self.cmd_pub.publish(twist_msg)
        rospy.sleep(2.0)  # Allow time for movement initialization

    def image_callback(self, msg):
        rospy.loginfo("Received image callback.")
        try:
            self.rgb = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def encoder_callback(self, msg):
        rospy.loginfo("Received encoder callback.")
        # Log encoder data for now
        if msg.header.frame_id == f"{self.vehicle_name}/left_wheel":
            rospy.loginfo(f"Left encoder: {msg.data}")
        elif msg.header.frame_id == f"{self.vehicle_name}/right_wheel":
            rospy.loginfo(f"Right encoder: {msg.data}")

    def preprocess(self, image_rgb: np.ndarray) -> np.ndarray:
        rospy.loginfo("Preprocessing image.")
        hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
        lower_hsv = np.array([0, 0, 255])
        upper_hsv = np.array([50, 155, 255])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        return mask

    def compute_commands(self) -> Tuple[float, float]:
        rospy.loginfo("Computing motor commands.")
        if self.rgb is None:
            return 0.0, 0.0

        if self.left is None:
            rospy.loginfo("Initializing left and right matrices.")
            shape = self.rgb.shape[0], self.rgb.shape[1]
            self.left = self.get_motor_left_matrix(shape)
            self.right = self.get_motor_right_matrix(shape)

        P = self.preprocess(self.rgb)
        l = float(np.sum(P * self.left))
        r = float(np.sum(P * self.right))

        self.l_max = max(l, self.l_max)
        self.r_max = max(r, self.r_max)
        self.l_min = min(l, self.l_min)
        self.r_min = min(r, self.r_min)

        ls = self.rescale(l, self.l_min, self.l_max)
        rs = self.rescale(r, self.r_min, self.r_max)

        pwm_left = self.config.const + ls * self.config.gain
        pwm_right = self.config.const + rs * self.config.gain

        return pwm_left, pwm_right

    def get_motor_left_matrix(self, shape: Tuple[int, int]) -> np.ndarray:
        rospy.loginfo("Generating left motor matrix.")
        res = np.zeros(shape=shape, dtype="float32")
        res[100:150, 100:150] = 1
        res[300:, 200:] = 1
        return res

    def get_motor_right_matrix(self, shape: Tuple[int, int]) -> np.ndarray:
        rospy.loginfo("Generating right motor matrix.")
        res = np.zeros(shape=shape, dtype="float32")
        res[100:150, 100:300] = -1
        return res

    def rescale(self, a: float, L: float, U: float) -> float:
        rospy.loginfo("Rescaling value.")
        if np.allclose(L, U):
            return 0.0
        return (a - L) / (U - L)

    def publish_commands(self):
        rospy.loginfo("Starting command publication loop.")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Inside publish_commands loop.")
            pwm_left, pwm_right = self.compute_commands()
            twist_msg = Twist2DStamped(v=pwm_left, omega=pwm_right)
            self.cmd_pub.publish(twist_msg)
            rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("Shutting down Braitenberg Agent node.")
        self.cmd_pub.publish(Twist2DStamped(v=0.0, omega=0.0))
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        agent = BraitenbergAgent()
        agent.init_movement()
        agent.publish_commands()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
