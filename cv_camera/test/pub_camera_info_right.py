#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

image_msg = Image()
camera_info_msg = CameraInfo()

def yaml_to_CameraInfo(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "right"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def callback_image(msg):
    global image_msg
    image_msg = msg
    camera_info_msg.header = image_msg.header
    # image_msg.header = rospy.get_rostime()


if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    # import argparse
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument("filename", default="/home/usrg/catkin_ws/src/cv_camera/test" +\
    #                                          "sample_left.yaml")
    # args = arg_parser.parse_args()
    # filename = args.filename
    filename = "/home/usrg/catkin_ws/src/cv_camera/test/sample_right.yaml"

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher_right", anonymous=True)
    publisher = rospy.Publisher("/stereo/right/camera_info", CameraInfo, queue_size=10)
    pub_image = rospy.Publisher("/stereo/right/image_raw", Image, queue_size=10)
    sub_image = rospy.Subscriber("/right/raw",Image,callback_image)
    rate = rospy.Rate(30)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        pub_image.publish(image_msg)
        rate.sleep()

