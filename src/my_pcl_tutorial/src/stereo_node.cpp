//This node will subscribe to the topics -
///airsim_node/PhysXCar/StereoRight/Scene/camera_info
///airsim_node/PhysXCar/StereoLeft/Scene/camera_info
///airsim_node/PhysXCar/StereoRight/Scene
///airsim_node/PhysXCar/StereoLeft/Scene


#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

sensor_msgs::CameraInfo global_camera_info_left;
bool got_cam_info_left_once = 0;
sensor_msgs::CameraInfo global_camera_info_right;
bool got_cam_info_right_once = 0;

sensor_msgs::Image most_recent_left_img;
sensor_msgs::Image most_recent_right_img;

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

//bool who_is_first = 0;//0 for left first, 1 for right first

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  most_recent_left_img = *left_img;

  if(num_left_imgs == num_right_imgs)
    pub_left_img.publish(left_img_to_be_pub);
  else if(num_left_imgs < num_right_imgs){
    left_img_to_be_pub.header.stamp.sec = most_recent_right_img.header.stamp.sec;
    left_img_to_be_pub.header.stamp.nsec = most_recent_right_img.header.stamp.nsec;
    pub_left_img.publish(left_img_to_be_pub);
  }
  else
    ROS_INFO("The left camera just gave two images, without a simgle image coming from the right camera");
  num_left_imgs++;
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  most_recent_right_img = *right_img;
  
  if(num_left_imgs == num_right_imgs)
    pub_right_img.publish(right_img_to_be_pub);
  else if(num_right_imgs < num_left_imgs){
    right_img_to_be_pub.header.stamp.sec = most_recent_left_img.header.stamp.sec;
    right_img_to_be_pub.header.stamp.nsec = most_recent_left_img.header.stamp.nsec;
    pub_right_img.publish(right_img_to_be_pub);
  }
  else
    ROS_INFO("The right camera just gave two images, without a simgle image coming from the left camera");    
  num_right_imgs++;
}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  /*ROS_INFO("Got info about the left stereo cam\n");
  if(got_cam_info_left == 0){
    sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
    global_camera_info_left_once = *left_cam;
  }
  else{
    sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  }
  pub_left_cam.pub(left_cam_to_be_pub);*/
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  pub_left_cam.publish(left_cam_to_be_pub);
}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;
  pub_right_cam.publish(right_cam_to_be_pub);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

/*
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
*/

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

/*  ros::Publisher pub_left_img = nh.advertise<sensor_msgs::Image> ("/stereo/left/image_raw", 1);
  ros::Publisher pub_right_img = nh.advertise<sensor_msgs::Image> ("/stereo/right/image_raw",1);*/

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);
/*
  ros::Subscriber sub_left_image = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  ros::Subscriber sub_right_image = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);*/

//  image_transport::CameraSubscriber sub_left_image = 

  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);

  ros::spin();
  return 0;
}