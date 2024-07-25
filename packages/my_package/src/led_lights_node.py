#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
import time

class LedPatternSegment:
    def __init__(self, color=(255, 255, 255), brightness=100):
        self.color = color  # RGB color tuple (default: white)
        self.brightness = brightness  # Brightness level (default: 100%)

    def publish_led_pattern(self, pub):
        # Create LEDPattern message
        led_msg = LEDPattern()

        # Set RGB color values (assuming ColorRGBA format)
        for i in range(5):  # Assuming 5 LED segments
            rgba = ColorRGBA()
            rgba.r = self.color[0] / 255.0
            rgba.g = self.color[1] / 255.0
            rgba.b = self.color[2] / 255.0
            rgba.a = self.brightness / 100.0
            led_msg.rgb_vals.append(rgba)  # Append ColorRGBA to rgb_vals array
       
        # Publish LEDPattern message
        pub.publish(led_msg)
        rospy.loginfo(f"Published LED pattern with color {self.color} and brightness {self.brightness}%")

def main():
    rospy.init_node('led_controller_node', anonymous=True)
    pub = rospy.Publisher('/duckiegpt/led_emitter_node/led_pattern', LEDPattern, queue_size=1)
   
    led_segment = LedPatternSegment(color=(255, 0, 255))  # Set color to purple (RGB: 255, 0, 255)

    try:
        # Publish LED pattern continuously at 1 Hz for 5 seconds
        end_time = time.time() + 5.0
        rate = rospy.Rate(1)  # 1 Hz (1 message per second)
        while not rospy.is_shutdown() and time.time() < end_time:
            led_segment.publish_led_pattern(pub)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        # After 5 seconds, terminate the node
        rospy.loginfo("Node shutting down...")
        rospy.signal_shutdown("Finished operation")

if __name__ == '__main__':
    main()