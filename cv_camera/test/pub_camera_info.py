#!/usr/bin/env python
import rospy
import yaml
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class CameraSynchronizer:
    def __init__(self, filename_front_right, filename_front_left, filename_rear_right, filename_rear_left): 
        rospy.init_node("camera_publisher", anonymous=True)
        self.image_msg_front_right = Image()
        self.image_msg_rear_right = Image()
        # self.camera_info_msg_front_right = CameraInfo()
        # self.camera_info_msg_rear_right = CameraInfo()

        self.image_msg_front_left = Image()
        self.image_msg_rear_left = Image()
        # self.camera_info_msg_front_left = CameraInfo()
        # self.camera_info_msg_rear_left = CameraInfo()

        # Parse yaml file
        self.camera_info_msg_front_right = self.yaml_to_CameraInfo(filename_front_right, "front_right")
        self.camera_info_msg_rear_right = self.yaml_to_CameraInfo(filename_rear_right, "rear_right")
        self.camera_info_msg_front_left = self.yaml_to_CameraInfo(filename_front_left, "front_left")
        self.camera_info_msg_rear_left = self.yaml_to_CameraInfo(filename_rear_left, "rear_left")

        # Initialize publisher node
        self.publisher_front_right = rospy.Publisher("/front_right/camera_info", CameraInfo, queue_size=10)
        self.publisher_rear_right = rospy.Publisher("/rear_right/camera_info", CameraInfo, queue_size=10)
        self.pub_image_front_right = rospy.Publisher("/front_right/image_raw", Image, queue_size=10)
        self.pub_image_rear_right = rospy.Publisher("/rear_right/image_raw", Image, queue_size=10)
        self.publisher_front_left = rospy.Publisher("/front_left/camera_info", CameraInfo, queue_size=10)
        self.publisher_rear_left = rospy.Publisher("/rear_left/camera_info", CameraInfo, queue_size=10)
        self.pub_image_front_left = rospy.Publisher("/front_left/image_raw", Image, queue_size=10)
        self.pub_image_rear_left = rospy.Publisher("/rear_left/image_raw", Image, queue_size=10)

        self.sub_image_front_right = rospy.Subscriber("/front_right/raw",Image,self.callback_image_front_right)
        self.sub_image_rear_right = rospy.Subscriber("/rear_right/raw",Image,self.callback_image_rear_right)
        self.sub_image_front_left = rospy.Subscriber("/front_left/raw",Image,self.callback_image_front_left)
        self.sub_image_rear_left = rospy.Subscriber("/rear_left/raw",Image,self.callback_image_rear_left)


        rospy.spin()


    def yaml_to_CameraInfo(self, yaml_fname, frame_id):
        
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = frame_id
        # camera_info_msg.header.stamp = rospy.get_rostime()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg


    def callback_image_front_right(self, msg):
        # self.camera_info_msg_front_right.header.stamp = image_msg_front_right.header.stamp 
        front_right_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        # resize_front_right = cv2.resize(front_right_image, (512, 400))
        self.image_msg_front_right = CvBridge().cv2_to_imgmsg(front_right_image, "bgr8")
        self.image_msg_front_right.header.frame_id = self.camera_info_msg_front_right.header.frame_id

        self.publisher_front_right.publish(self.camera_info_msg_front_right)
        self.pub_image_front_right.publish(self.image_msg_front_right)

    def callback_image_front_left(self, msg):
        # self.camera_info_msg_front_left.header.stamp = image_msg_front_left.header.stamp
        front_left_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        resize_front_left = cv2.resize(front_left_image, (512, 400))
        self.image_msg_front_left = CvBridge().cv2_to_imgmsg(resize_front_left, "bgr8")
        self.image_msg_front_left.header.frame_id = self.camera_info_msg_front_left.header.frame_id

        self.publisher_front_left.publish(self.camera_info_msg_front_left)
        self.pub_image_front_left.publish(self.image_msg_front_left)

    def callback_image_rear_right(self, msg):
        # self.camera_info_msg_rear_right.header.stamp = image_msg_rear_right.header.stamp
        rear_right_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        resize_rear_right = cv2.resize(rear_right_image, (512, 400))
        self.image_msg_rear_right = CvBridge().cv2_to_imgmsg(resize_rear_right, "bgr8")
        self.image_msg_rear_right.header.frame_id = self.camera_info_msg_rear_right.header.frame_id 

        self.publisher_rear_right.publish(self.camera_info_msg_rear_right)
        self.pub_image_rear_right.publish(self.image_msg_rear_right)


    def callback_image_rear_left(self, msg):
        # self.camera_info_msg_rear_left.header.stamp = image_msg_rear_left.header.stamp
        rear_left_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        resize_rear_left = cv2.resize(rear_left_image, (512, 400))
        self.image_msg_rear_left = CvBridge().cv2_to_imgmsg(resize_rear_left, "bgr8")
        self.image_msg_rear_left.header.frame_id = self.camera_info_msg_rear_left.header.frame_id 

        self.publisher_rear_left.publish(self.camera_info_msg_rear_left)
        self.pub_image_rear_left.publish(self.image_msg_rear_left)


if __name__ == "__main__":

    filename_front_right = "/home/etri/catkin_ws/src/cv_camera/test/sample_right.yaml"
    filename_front_left = "/home/etri/catkin_ws/src/cv_camera/test/left.yaml"
    filename_rear_right = "/home/etri/catkin_ws/src/cv_camera/test/down.yaml"
    filename_rear_left = "/home/etri/catkin_ws/src/cv_camera/test/down.yaml"

    camera_pub = CameraSynchronizer(filename_front_right, filename_front_left, filename_rear_right, filename_rear_left)
    # rate = rospy.Rate(30)

    # # Run publisher
    # while not rospy.is_shutdown():
    #     publisher_front_right.publish(camera_info_msg_front_right)
    #     pub_image_front_right.publish(image_msg_front_right)

    #     publisher_rear_right.publish(camera_info_msg_rear_right)
    #     pub_image_rear_right.publish(image_msg_rear_right)

    #     publisher_front_left.publish(camera_info_msg_front_left)
    #     pub_image_front_left.publish(image_msg_front_left)

    #     publisher_rear_left.publish(camera_info_msg_rear_left)
    #     pub_image_rear_left.publish(image_msg_rear_left)


    #     rate.sleep()

