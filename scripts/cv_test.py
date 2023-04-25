#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

depth_data = None
color_image = None
bridge = CvBridge()

def depth_callback(data):
    global depth_data
    depth_data = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    rospy.loginfo("Depth data received")

def color_callback(data):
    global color_image
    color_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    rospy.loginfo("Color data received")

def tag_callback(data):
    global depth_data, color_image

    if data.detections and depth_data is not None and color_image is not None:
        rospy.loginfo("Tag detected")
        tag = data.detections[0] # Assuming only one tag is being tracked
        tag_x = tag.pose.pose.position.x
        tag_y = tag.pose.pose.position.y

        # Get depth at the tag's position
        u = int(tag_x * depth_data.shape[1])
        v = int(tag_y * depth_data.shape[0])

        depth = depth_data[v, u] / 1000.0

        dog_position = PointStamped()
        dog_position.header.stamp = rospy.Time.now()
        dog_position.point.x = tag_x
        dog_position.point.y = tag_y
        dog_position.point.z = depth

        position_pub.publish(dog_position)
        rospy.loginfo("Published dog position")

        # Draw XYZ coordinates on the image
        img = color_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_color = (0, 255, 0)
        text_scale = 0.5
        text_thickness = 1

        text = "X: {:.3f} Y: {:.3f} Z: {:.3f}".format(tag_x, tag_y, depth)
        cv2.putText(img, text, (u, v), font, text_scale, text_color, text_thickness)

        # Show the image
        cv2.imshow("Tracking", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('dog_position_tracker', anonymous=True)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, color_callback)

    position_pub = rospy.Publisher("/dog/position", PointStamped, queue_size=1)

    rospy.loginfo("Dog position tracker node initialized")
    rospy.spin()