#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import threading

depth_data = None
color_image = None
bridge = CvBridge()
tag_detected = False
resize_factor = 0.5
bbox_color = (0, 255, 0)  # Green
bbox_thickness = 2
tag_detect_fail_count = 0

def depth_callback(data):
    global depth_data
    depth_data = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def color_callback(data):
    global color_image
    color_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

def tag_callback(data):
    global depth_data, color_image, tag_detected, tag_detect_fail_count

    if data.detections and depth_data is not None and color_image is not None:
        tag_detected = True
        tag = data.detections[0] # Assuming only one tag is being tracked
        tag_x = tag.pose.pose.pose.position.x
        tag_y = tag.pose.pose.pose.position.y

        # Get depth at the tag's position
        u = int(tag_x * depth_data.shape[1])
        v = int(tag_y * depth_data.shape[0])

        depth = depth_data[v, u] / 1000.0

        dog_position = Point()
        dog_position.x = tag_x
        dog_position.y = tag_y
        dog_position.z = depth

        position_pub.publish(dog_position)
    else:
        tag_detected = False
        tag_detect_fail_count += 1
        if tag_detect_fail_count >= 100:
            dog_position = Point()
            dog_position.x = 50
            dog_position.y = 50
            dog_position.z = 50
            position_pub.publish(dog_position)
            tag_detect_fail_count = 0


def show_image():
    global color_image, tag_detected

    while not rospy.is_shutdown():
        if color_image is not None:
            img = color_image.copy()

            # Resize the image
            img = cv2.resize(img, None, fx=resize_factor, fy=resize_factor, interpolation=cv2.INTER_AREA)

            cv2.imshow("Tracking", img)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('dog_position_tracker', anonymous=True)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, color_callback)

    position_pub = rospy.Publisher("/dog/position", Point, queue_size=1)

    rospy.loginfo("Dog position tracker node initialized")

    # Start image display thread
    #image_display_thread = threading.Thread(target=show_image)
    #image_display_thread.start()

    rospy.spin()

    # Clean up the OpenCV window after exiting the node
    cv2.destroyAllWindows()