#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

image_msg_right = Image()
image_msg_left = Image()
image_msg_low_res = Image()
camera_info_msg_right = CameraInfo()
camera_info_msg_left = CameraInfo()
bridge = CvBridge()

header = Header()

def yaml_to_CameraInfo_right(yaml_fname):
    
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

def yaml_to_CameraInfo_left(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "left"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def callback_image_right(msg):
    global image_msg_right
    image_msg_right = msg
    camera_info_msg_right.header = image_msg_right.header
    camera_info_msg_right.header.frame_id = "camera_right"

    # global bridge
    # cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # down_width = 512
    # down_height = 320
    # down_points = (down_width, down_height)
    # resized_down = cv2.resize(cv_image, down_points, interpolation= cv2.INTER_LINEAR)

    # global image_msg_low_res
    # image_msg_low_res = bridge.cv2_to_imgmsg(resized_down, encoding="passthrough")
    # image_msg_low_res.header = image_msg_right.header
    # image_msg_low_res.header.frame_id = "camera_right"


def callback_image_left(msg):
    global image_msg_left
    image_msg_left = msg
    camera_info_msg_left.header = image_msg_left.header
    camera_info_msg_left.header.frame_id = "camera_left"


if __name__ == "__main__":
    
    filename_right = "/home/etri/catkin_ws/src/cv_camera/test/sample_right.yaml"
    filename_left = "/home/etri/catkin_ws/src/cv_camera/test/sample_left.yaml"
    # Parse yaml file
    camera_info_msg_right = yaml_to_CameraInfo_right(filename_right)
    camera_info_msg_left = yaml_to_CameraInfo_left(filename_left)

    # Initialize publisher node
    rospy.init_node("camera_publisher", anonymous=True)
    publisher_right = rospy.Publisher("/right/camera_info", CameraInfo, queue_size=10)
    publisher_left = rospy.Publisher("/left/camera_info", CameraInfo, queue_size=10)
    pub_image_right = rospy.Publisher("/right/image_raw", Image, queue_size=10)
    sub_image_right = rospy.Subscriber("/right/raw",Image,callback_image_right)
    pub_image_left = rospy.Publisher("/left/image_raw", Image, queue_size=10)
    sub_image_left = rospy.Subscriber("/left/raw",Image,callback_image_left)
    # pub_image_low_res = rospy.Publisher("/image_low_resolution", Image, queue_size=10)
    rate = rospy.Rate(30)

    # Run publisher
    while not rospy.is_shutdown():
        publisher_right.publish(camera_info_msg_right)
        pub_image_right.publish(image_msg_right)
        publisher_left.publish(camera_info_msg_left)
        pub_image_left.publish(image_msg_left)
        # pub_image_low_res.publish(image_msg_low_res)
        
        rate.sleep()

