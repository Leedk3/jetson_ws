#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher image_right_pub;
ros::Publisher info_right_pub;
ros::Publisher image_left_pub;
ros::Publisher info_left_pub;


void callback(const ImageConstPtr& image_right, const CameraInfoConstPtr& info_right, const ImageConstPtr& image_left, const CameraInfoConstPtr& info_left)
{
  // Solve all of perception here...
  ROS_INFO("Sync_Callback");
  image_right_pub.publish(image_right);
  info_right_pub.publish(info_right);
  image_left_pub.publish(image_left);
  info_left_pub.publish(info_left);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_synchronizer_node");

//   std::string zed1_img_topic = "/zed_rear_right/left/image_raw_color";
//   std::string zed2_img_topic = "/zed_rear_right/right/image_raw_color";
//   std::string zed3_img_topic = "/zed_rear_right/left/image_raw_color";
//   std::string zed4_img_topic = "/zed_rear_right/right/image_raw_color";


  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image_right_sub(nh,"/front_right/image_raw", 100);
  message_filters::Subscriber<CameraInfo> info_right_sub(nh, "/front_right/camera_info", 100);
  message_filters::Subscriber<Image> image_left_sub(nh, "/front_left/image_raw", 100);
  message_filters::Subscriber<CameraInfo> info_left_sub(nh,"/front_left/camera_info", 100);

  image_right_pub = nh.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1000);
  info_right_pub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1000);
  image_left_pub = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1000);
  info_left_pub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1000);



  typedef sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), image_right_sub, info_right_sub, image_left_sub, info_left_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));


  ros::spin();

  return 0;
}