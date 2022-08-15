#!/usr/bin/env python
import rospy
import yaml
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

image_msg_front_right = Image()
image_msg_rear_right = Image()
camera_info_msg_front_right = CameraInfo()
camera_info_msg_rear_right = CameraInfo()

image_msg_front_left = Image()
image_msg_rear_left = Image()
camera_info_msg_front_left = CameraInfo()
camera_info_msg_rear_left = CameraInfo()


header = Header()

def yaml_to_CameraInfo_front_right(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "front_right"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def yaml_to_CameraInfo_front_left(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "front_left"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def yaml_to_CameraInfo_rear_right(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "rear_right"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def yaml_to_CameraInfo_rear_left(yaml_fname):
    
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "rear_left"
    # camera_info_msg.header.stamp = rospy.get_rostime()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def callback_image_front_right(msg):
    global image_msg_front_right
    # image_msg_front_right = msg
    # camera_info_msg_front_right.header = image_msg_front_right.header
    front_right_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    resize_front_right = cv2.resize(front_right_image, (512, 400))
    image_msg_front_right = CvBridge().cv2_to_imgmsg(resize_front_right, "bgr8")
    camera_info_msg_front_right.header = image_msg_front_right.header

def callback_image_front_left(msg):
    global image_msg_front_left
    # image_msg_front_left = msg
    # camera_info_msg_front_left.header = image_msg_front_left.header
    front_left_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    resize_front_left = cv2.resize(front_left_image, (512, 400))
    image_msg_front_left = CvBridge().cv2_to_imgmsg(resize_front_left, "bgr8")
    camera_info_msg_front_left.header = image_msg_front_left.header


def callback_image_rear_right(msg):
    global image_msg_rear_right
    # image_msg_rear_right = msg
    # camera_info_msg_rear_right.header = image_msg_rear_right.header
    rear_right_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    resize_rear_right = cv2.resize(rear_right_image, (512, 400))
    image_msg_rear_right = CvBridge().cv2_to_imgmsg(resize_rear_right, "bgr8")
    camera_info_msg_rear_right.header = image_msg_rear_right.header
    
def callback_image_rear_left(msg):
    global image_msg_rear_left
    # image_msg_rear_left = msg
    # camera_info_msg_rear_left.header = image_msg_rear_left.header
    rear_left_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    resize_rear_left = cv2.resize(rear_left_image, (512, 400))
    image_msg_rear_left = CvBridge().cv2_to_imgmsg(resize_rear_left, "bgr8")
    camera_info_msg_rear_left.header = image_msg_rear_left.header


if __name__ == "__main__":
    
    filename_front_right = "/home/etri/catkin_ws/src/cv_camera/test/right.yaml"
    filename_front_left = "/home/etri/catkin_ws/src/cv_camera/test/left.yaml"

    filename_rear_right = "/home/etri/catkin_ws/src/cv_camera/test/down.yaml"
    filename_rear_left = "/home/etri/catkin_ws/src/cv_camera/test/down.yaml"


    # Parse yaml file
    camera_info_msg_front_right = yaml_to_CameraInfo_front_right(filename_front_right)
    camera_info_msg_rear_right = yaml_to_CameraInfo_rear_right(filename_rear_right)

    camera_info_msg_front_left = yaml_to_CameraInfo_front_left(filename_front_left)
    camera_info_msg_rear_left = yaml_to_CameraInfo_rear_left(filename_rear_left)

    # Initialize publisher node
    rospy.init_node("camera_publisher", anonymous=True)
    publisher_front_right = rospy.Publisher("/front_right/camera_info", CameraInfo, queue_size=10)
    publisher_rear_right = rospy.Publisher("/rear_right/camera_info", CameraInfo, queue_size=10)
    pub_image_front_right = rospy.Publisher("/front_right/image_raw", Image, queue_size=10)
    sub_image_front_right = rospy.Subscriber("/front_right/raw",Image,callback_image_front_right)
    pub_image_rear_right = rospy.Publisher("/rear_right/image_raw", Image, queue_size=10)
    sub_image_rear_right = rospy.Subscriber("/rear_right/raw",Image,callback_image_rear_right)

    publisher_front_left = rospy.Publisher("/front_left/camera_info", CameraInfo, queue_size=10)
    publisher_rear_left = rospy.Publisher("/rear_left/camera_info", CameraInfo, queue_size=10)
    pub_image_front_left = rospy.Publisher("/front_left/image_raw", Image, queue_size=10)
    sub_image_front_left = rospy.Subscriber("/front_left/raw",Image,callback_image_front_left)
    pub_image_rear_left = rospy.Publisher("/rear_left/image_raw", Image, queue_size=10)
    sub_image_rear_left = rospy.Subscriber("/rear_left/raw",Image,callback_image_rear_left)

    rate = rospy.Rate(30)

    # Run publisher
    while not rospy.is_shutdown():
        publisher_front_right.publish(camera_info_msg_front_right)
        pub_image_front_right.publish(image_msg_front_right)

        publisher_rear_right.publish(camera_info_msg_rear_right)
        pub_image_rear_right.publish(image_msg_rear_right)

        publisher_front_left.publish(camera_info_msg_front_left)
        pub_image_front_left.publish(image_msg_front_left)

        publisher_rear_left.publish(camera_info_msg_rear_left)
        pub_image_rear_left.publish(image_msg_rear_left)


        rate.sleep()

