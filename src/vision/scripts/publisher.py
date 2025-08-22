#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import pyrealsense2 as rs
import cv2
import numpy as np
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

def publish_realsense_data():
    # Initialize the ROS node
    rospy.init_node('realsense_publisher', anonymous=True)

    # Set up the CvBridge for converting images
    bridge = CvBridge()

    # Create the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the color, depth, and infrared streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start the pipeline
    profile = pipeline.start(config)
    sensor = profile.get_device().query_sensors()[0]
    sensor.set_option(rs.option.exposure, 15317)

    # Create publishers for the color, depth, and pointcloud data
    color_pub = rospy.Publisher("/nakit_vision/color/image_raw", Image, queue_size=10)
    depth_pub = rospy.Publisher("/nakit_vision/depth/image_raw", Image, queue_size=10)

    # Wait for a few frames to allow the camera to initialize
    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            # Wait for the frames
            frames = pipeline.wait_for_frames()
            
            # Get color and depth frames
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            # Convert color frame to a ROS Image message
            color_image = bridge.cv2_to_imgmsg(np.asanyarray(color_frame.get_data()), encoding="bgr8")

            # Convert depth frame to a ROS Image message
            depth_image = bridge.cv2_to_imgmsg(np.asanyarray(depth_frame.get_data()), encoding="16UC1")

            # Publish color and depth images
            color_pub.publish(color_image)
            depth_pub.publish(depth_image)


            rospy.sleep(0.033)  # Sleep to maintain a rate of 30 Hz

    finally:
        # Stop the pipeline when exiting the loop
        pipeline.stop()

if __name__ == '__main__':
    try:
        publish_realsense_data()
    except rospy.ROSInterruptException:
        pass

