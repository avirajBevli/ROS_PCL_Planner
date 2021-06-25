//Whichever of the two cameras(left and right) gives of the image related to a particular time stamp faster, 
//when the other camera recieves image of the same time stamp, it will copy the header of the first camera
//in its field header, so that for each pair of images with the same sequence ID, the time stamps are same 

//This node will subscribe to the topics -
///airsim_node/PhysXCar/StereoRight/Scene/camera_info
///airsim_node/PhysXCar/StereoLeft/Scene/camera_info
///airsim_node/PhysXCar/StereoRight/Scene
///airsim_node/PhysXCar/StereoLeft/Scene
/*
#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

std::vector<std_msgs::Header> left_arr;
std::vector<std_msgs::Header> right_arr;

int isElem(uint32_t seq_id, std::vector<std_msgs::Header> arr){
  int n = arr.size();
  for(int i=0;i<n;i++){
    if(arr[i].seq == seq_id){
      ROS_INFO("ELEMENT\n");
      return (i+1);
    }
  }
  return 0;
}

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  if( !isElem(left_img_to_be_pub.header.seq, left_arr) )
    left_arr.push_back(left_img_to_be_pub.header);
  else{
    left_img_to_be_pub.header.stamp = left_arr[isElem(left_img_to_be_pub.header.seq, left_arr) - 1].stamp;
    pub_left_img.publish(left_img_to_be_pub);
    return;
  }

  if(isElem(left_img_to_be_pub.header.seq, right_arr))
    left_img_to_be_pub.header.stamp = right_arr[isElem(left_img_to_be_pub.header.seq, right_arr) - 1].stamp;
  pub_left_img.publish(left_img_to_be_pub);
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  if( !isElem(right_img_to_be_pub.header.seq, right_arr) )
    right_arr.push_back(right_img_to_be_pub.header);
  else{
    right_img_to_be_pub.header.stamp = right_arr[isElem(right_img_to_be_pub.header.seq, right_arr) - 1].stamp;
    pub_right_img.publish(right_img_to_be_pub);
    return;
  }

  if(isElem(right_img_to_be_pub.header.seq, left_arr))
    right_img_to_be_pub.header.stamp = left_arr[isElem(right_img_to_be_pub.header.seq, left_arr) - 1].stamp;
  pub_right_img.publish(right_img_to_be_pub);
}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  if( !isElem(left_cam_to_be_pub.header.seq, left_arr) )
    left_arr.push_back(left_cam_to_be_pub.header);
  else{
    left_cam_to_be_pub.header.stamp = left_arr[isElem(left_cam_to_be_pub.header.seq, left_arr) - 1].stamp;
    pub_left_cam.publish(left_cam_to_be_pub);
    return;
  }

  if(isElem(left_cam_to_be_pub.header.seq, right_arr))
    left_cam_to_be_pub.header.stamp = right_arr[isElem(left_cam_to_be_pub.header.seq, right_arr) - 1].stamp;
  pub_left_cam.publish(left_cam_to_be_pub);
}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;

  if( !isElem(right_cam_to_be_pub.header.seq, right_arr) )
    right_arr.push_back(right_cam_to_be_pub.header);
  else{
    right_cam_to_be_pub.header.stamp = right_arr[isElem(right_cam_to_be_pub.header.seq, right_arr) - 1].stamp;
    pub_right_cam.publish(right_cam_to_be_pub);
    return;
  }

  if(isElem(right_cam_to_be_pub.header.seq, left_arr))
    right_cam_to_be_pub.header.stamp = left_arr[isElem(right_cam_to_be_pub.header.seq, left_arr) - 1].stamp;
  pub_right_cam.publish(right_cam_to_be_pub);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);

  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);

  ros::spin();
  return 0;
}
*/
/*
header: 
  seq: 14
  stamp: 
    secs: 1590159599
    nsecs: 391111114
  frame_id: "StereoLeft/Scene_optical"
height: 600
width: 960
distortion_model: ''
D: []
K: [480.0, 0.0, 480.0, 0.0, 480.0, 300.0, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [480.0, 0.0, 480.0, 0.0, 0.0, 480.0, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
*/
#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

std::vector<std_msgs::Header> arr;

int isElem(uint32_t seq_id, std::vector<std_msgs::Header> arr){
  int n = arr.size();
  for(int i=0;i<n;i++){
    if(arr[i].seq == seq_id){
      //ROS_INFO("ELEMENT\n");
      return (i+1);
    }
  }
  return 0;
}

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  //uncomment it later
  /*if( !isElem(left_img_to_be_pub.header.seq, arr) )
    arr.push_back(left_img_to_be_pub.header);
  else
    left_img_to_be_pub.header.stamp = arr[isElem(left_img_to_be_pub.header.seq, arr) - 1].stamp;*/
  pub_left_img.publish(left_img_to_be_pub);

  sensor_msgs::CameraInfo cam_info_message;
  //frame_id is particularly useful because it says which frame this data is assosiated with 
  cam_info_message.header.frame_id = "StereoLeft_optical";
  cam_info_message.height = 600;
  cam_info_message.width = 960;
  cam_info_message.distortion_model = "plumb_bob";
  cam_info_message.D = {0, 0, 0, 0, 0};
  cam_info_message.K = {480.0, 0.000, 480.0, 0.000, 480.0, 300.0, 0.000000, 0.000000, 1.000000};
  cam_info_message.R = {1.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000, 1.000};
  cam_info_message.P = {480.0, 0.000, 480.0, 0.000, 0.000, 480.0, 300.0, 0.000, 0.000000, 0.000000, 1.000, 0.000};
  cam_info_message.binning_x = 0;
  cam_info_message.binning_y = 0;
  cam_info_message.roi.x_offset = 0;
  cam_info_message.roi.y_offset = 0;
  cam_info_message.roi.height = 0;
  cam_info_message.roi.width = 0;
  cam_info_message.roi.do_rectify = false;
  
  cam_info_message.header.stamp = left_img_to_be_pub.header.stamp;
  cam_info_message.header.seq = left_img_to_be_pub.header.seq;  
  pub_left_cam.publish(cam_info_message);
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  //uncomment it later
/*  if( !isElem(right_img_to_be_pub.header.seq, arr) )
    arr.push_back(right_img_to_be_pub.header);
  else
    right_img_to_be_pub.header.stamp = arr[isElem(right_img_to_be_pub.header.seq, arr) - 1].stamp;
 */ pub_right_img.publish(right_img_to_be_pub);

  sensor_msgs::CameraInfo cam_info_message;
  cam_info_message.header.frame_id = "StereoRight_optical";
  cam_info_message.height = 600;
  cam_info_message.width = 960;
  cam_info_message.distortion_model = "plumb_bob";
  cam_info_message.D = {0, 0, 0, 0, 0};
  cam_info_message.K = {480.0, 0.000, 480.0, 0.000, 480.0, 300.0, 0.000000, 0.000000, 1.000000};
  cam_info_message.R = {1.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000, 1.000};
  cam_info_message.P = {480.0, 0.000, 480.0, -57.6, 0.000, 480.0, 300.0, 0.000, 0.000000, 0.000000, 1.000, 0.000};
  cam_info_message.binning_x = 0;
  cam_info_message.binning_y = 0;
  cam_info_message.roi.x_offset = 0;
  cam_info_message.roi.y_offset = 0;
  cam_info_message.roi.height = 0;
  cam_info_message.roi.width = 0;
  cam_info_message.roi.do_rectify = false;

  cam_info_message.header.stamp = right_img_to_be_pub.header.stamp;
  cam_info_message.header.seq = right_img_to_be_pub.header.seq;
  pub_right_cam.publish(cam_info_message);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1); 
  /*
  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);
  */

  ros::spin();
  return 0;
}


/*

#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

std::vector<std_msgs::Header> arr;

int isElem(uint32_t seq_id, std::vector<std_msgs::Header> arr){
  int n = arr.size();
  for(int i=0;i<n;i++){
    if(arr[i].seq == seq_id){
      //ROS_INFO("ELEMENT\n");
      return (i+1);
    }
  }
  return 0;
}

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  if( !isElem(left_img_to_be_pub.header.seq, arr) )
    arr.push_back(left_img_to_be_pub.header);
  else
    left_img_to_be_pub.header.stamp = arr[isElem(left_img_to_be_pub.header.seq, arr) - 1].stamp;
  pub_left_img.publish(left_img_to_be_pub);
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  if( !isElem(right_img_to_be_pub.header.seq, arr) )
    arr.push_back(right_img_to_be_pub.header);
  else
    right_img_to_be_pub.header.stamp = arr[isElem(right_img_to_be_pub.header.seq, arr) - 1].stamp;
  pub_right_img.publish(right_img_to_be_pub);
}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  if( !isElem(left_cam_to_be_pub.header.seq, arr) )
    arr.push_back(left_cam_to_be_pub.header);
  else
    left_cam_to_be_pub.header.stamp = arr[isElem(left_cam_to_be_pub.header.seq, arr) - 1].stamp;
  pub_left_cam.publish(left_cam_to_be_pub);
}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;

  if( !isElem(right_cam_to_be_pub.header.seq, arr) )
    arr.push_back(right_cam_to_be_pub.header);
  else
    right_cam_to_be_pub.header.stamp = arr[isElem(right_cam_to_be_pub.header.seq, arr) - 1].stamp;
  pub_right_cam.publish(right_cam_to_be_pub);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);

  ros::spin();
  return 0;
}

*/

//I used to think that stereo matching is done accoeding to the sequence_ID. Since this is wrong, this commented code is trash
/*
#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

uint32_t left_img_index = 0;
uint32_t right_img_index = 0;
uint32_t left_cam_index = 0;
uint32_t right_cam_index = 0;

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo img\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;

  left_img_to_be_pub.header.seq = left_img_index;
  pub_left_img.publish(left_img_to_be_pub);
  left_img_index = left_img_index + 1;
  ROS_INFO("left_img_index: %d\n",left_img_index);
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo img\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;

  right_img_to_be_pub.header.seq = right_img_index;
  pub_right_img.publish(right_img_to_be_pub);
  right_img_index = right_img_index + 1;
  ROS_INFO("right_img_index: %d\n",right_img_index);
}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;

  left_cam_to_be_pub.header.seq = left_cam_index;
  pub_left_cam.publish(left_cam_to_be_pub);
  left_cam_index = left_cam_index + 1;
  ROS_INFO("left_cam_index: %d\n",left_cam_index);
}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;

  right_cam_to_be_pub.header.seq = right_cam_index;
  pub_right_cam.publish(right_cam_to_be_pub);
  right_cam_index = right_cam_index + 1;
  ROS_INFO("right_cam_index: %d\n",right_cam_index);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);

  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);

  ros::spin();
  return 0;
}
*/



/*
#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

std::vector<std_msgs::Header> arr;
std::vector<bool> has_been_received;

int isElem(uint32_t seq_id, std::vector<std_msgs::Header> arr){
  int n = arr.size();
  for(int i=0;i<n;i++){
    if(arr[i].seq == seq_id){
      ROS_INFO("ELEMENT\n");
      return (i+1);
    }
  }
  return 0;
}

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  std_msgs::Header sample_header;
  sample_header.seq = 0;
  sample_header.stamp.sec = 1590159599;
  sample_header.stamp.nsec = 391111114;
  sample_header.frame_id = "StereoLeft/Scene_optical";

  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  has_been_received.push_back(0);
  arr.push_back(sample_header);
  
  if(has_been_received[left_img_to_be_pub.header.seq])
    left_img_to_be_pub.header.stamp = arr[left_img_to_be_pub.header.seq].stamp;
  else{
    has_been_received[left_img_to_be_pub.header.seq] = 1;
    arr[(int)left_img_to_be_pub.header.seq] = left_img_to_be_pub.header;
  }
  pub_left_img.publish(left_img_to_be_pub);
  ROS_INFO("Exiting the left stereo cam\n");

}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  std_msgs::Header sample_header;
  sample_header.seq = 0;
  sample_header.stamp.sec = 1590159599;
  sample_header.stamp.nsec = 391111114;
  sample_header.frame_id = "StereoLeft/Scene_optical";
  
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  has_been_received.push_back(0);
  arr.push_back(sample_header);
  
  if(has_been_received[right_img_to_be_pub.header.seq])
    right_img_to_be_pub.header.stamp = arr[right_img_to_be_pub.header.seq].stamp;
  else{
    has_been_received[right_img_to_be_pub.header.seq] = 1;
    arr[(int)right_img_to_be_pub.header.seq] = right_img_to_be_pub.header;
  }
  pub_right_img.publish(right_img_to_be_pub);
  ROS_INFO("Exiting the right stereo cam \n");

}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  std_msgs::Header sample_header;
  sample_header.seq = 0;
  sample_header.stamp.sec = 1590159599;
  sample_header.stamp.nsec = 391111114;
  sample_header.frame_id = "StereoLeft/Scene_optical";
  
  ROS_INFO("Got info from the left stereo cam info\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  has_been_received.push_back(0);
  arr.push_back(sample_header);
  ROS_INFO("HI\n");
  
  if(has_been_received[left_cam_to_be_pub.header.seq]){
    ROS_INFO("if HI\n");
    left_cam_to_be_pub.header.stamp = arr[left_cam_to_be_pub.header.seq].stamp;
    ROS_INFO("if HI\n");
  }
  else{
    ROS_INFO("else HI\n");
    has_been_received[left_cam_to_be_pub.header.seq] = 1;
    ROS_INFO("else HI\n");
    arr[(int)left_cam_to_be_pub.header.seq] = left_cam_to_be_pub.header;
    ROS_INFO("else HI\n");
  }
  pub_left_cam.publish(left_cam_to_be_pub);
  ROS_INFO("Exiting the left stereo cam info\n");

}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  std_msgs::Header sample_header;
  sample_header.seq = 0;
  sample_header.stamp.sec = 1590159599;
  sample_header.stamp.nsec = 391111114;
  sample_header.frame_id = "StereoLeft/Scene_optical";
  
  ROS_INFO("Got info from the right stereo cam info\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;
  has_been_received.push_back(0);
  arr.push_back(sample_header);
  ROS_INFO("HI\n");

  if(has_been_received[right_cam_to_be_pub.header.seq]){
    ROS_INFO("if HI\n");
    right_cam_to_be_pub.header.stamp = arr[right_cam_to_be_pub.header.seq].stamp;
     ROS_INFO("if HI\n");
  }
  else{
    ROS_INFO("else HI\n");
    has_been_received[right_cam_to_be_pub.header.seq] = 1;
    ROS_INFO("else HI\n");
    arr[(int)right_cam_to_be_pub.header.seq] = right_cam_to_be_pub.header;
     ROS_INFO("else HI\n");

  }

  ROS_INFO("HI\n");
  pub_right_cam.publish(right_cam_to_be_pub);

  ROS_INFO("Exiting the right stereo cam info\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);

  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);

  ros::spin();
  return 0;
}
*/
/*
#include "ros/ros.h"
#include <ros/spinner.h>
//#include <glog/logging.h>
#include <image_transport/image_transport.h>//to work with images
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

int num_left_imgs = 0;
int num_right_imgs = 0;

image_transport::Publisher pub_left_img;
image_transport::Publisher pub_right_img;
ros::Publisher pub_left_cam;
ros::Publisher pub_right_cam;

std::vector<std_msgs::Header> left_arr;
std::vector<std_msgs::Header> right_arr;

int isElem(uint32_t seq_id, std::vector<std_msgs::Header> arr){
  int n = arr.size();
  for(int i=0;i<n;i++){
    if(arr[i].seq == seq_id){
      ROS_INFO("ELEMENT\n");
      return (i+1);
    }
  }
  return 0;
}

void StereoLeftCB(const sensor_msgs::ImageConstPtr& left_img)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::Image left_img_to_be_pub = *left_img;
  if( !isElem(left_img_to_be_pub.header.seq, left_arr) )
    left_arr.push_back(left_img_to_be_pub.header);
  else{
    left_img_to_be_pub.header = left_arr[isElem(left_img_to_be_pub.header.seq, left_arr) - 1];
    pub_left_img.publish(left_img_to_be_pub);
    return;
  }

  if(isElem(left_img_to_be_pub.header.seq, right_arr))
    left_img_to_be_pub.header = right_arr[isElem(left_img_to_be_pub.header.seq, right_arr) - 1];
  pub_left_img.publish(left_img_to_be_pub);
}

void StereoRightCB(const sensor_msgs::ImageConstPtr& right_img)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::Image right_img_to_be_pub = *right_img;
  if( !isElem(right_img_to_be_pub.header.seq, right_arr) )
    right_arr.push_back(right_img_to_be_pub.header);
  else{
    right_img_to_be_pub.header = right_arr[isElem(right_img_to_be_pub.header.seq, right_arr) - 1];
    pub_right_img.publish(right_img_to_be_pub);
    return;
  }

  if(isElem(right_img_to_be_pub.header.seq, left_arr))
    right_img_to_be_pub.header = left_arr[isElem(right_img_to_be_pub.header.seq, left_arr) - 1];
  pub_right_img.publish(right_img_to_be_pub);
}

void StereoLeftCamCB(const sensor_msgs::CameraInfoPtr& left_cam)
{
  ROS_INFO("Got info from the left stereo cam\n");
  sensor_msgs::CameraInfo left_cam_to_be_pub = *left_cam;
  if( !isElem(left_cam_to_be_pub.header.seq, left_arr) )
    left_arr.push_back(left_cam_to_be_pub.header);
  else{
    left_cam_to_be_pub.header = left_arr[isElem(left_cam_to_be_pub.header.seq, left_arr) - 1];
    pub_left_cam.publish(left_cam_to_be_pub);
    return;
  }

  if(isElem(left_cam_to_be_pub.header.seq, right_arr))
    left_cam_to_be_pub.header = right_arr[isElem(left_cam_to_be_pub.header.seq, right_arr) - 1];
  pub_left_cam.publish(left_cam_to_be_pub);
}

void StereoRightCamCB(const sensor_msgs::CameraInfoPtr& right_cam)
{
  ROS_INFO("Got info from the right stereo cam\n");
  sensor_msgs::CameraInfo right_cam_to_be_pub = *right_cam;

  if( !isElem(right_cam_to_be_pub.header.seq, right_arr) )
    right_arr.push_back(right_cam_to_be_pub.header);
  else{
    right_cam_to_be_pub.header = right_arr[isElem(right_cam_to_be_pub.header.seq, right_arr) - 1];
    pub_right_cam.publish(right_cam_to_be_pub);
    return;
  }

  if(isElem(right_cam_to_be_pub.header.seq, left_arr))
    right_cam_to_be_pub.header = left_arr[isElem(right_cam_to_be_pub.header.seq, left_arr) - 1];
  pub_right_cam.publish(right_cam_to_be_pub);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left_image = it.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene", 1000, StereoLeftCB);
  image_transport::Subscriber sub_right_image = it.subscribe("/airsim_node/PhysXCar/StereoRight/Scene", 1000, StereoRightCB);
  
  pub_left_img = it.advertise("/stereo/left/image_raw", 1);
  pub_right_img = it.advertise("/stereo/right/image_raw", 1);

  pub_left_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info",1);
  pub_right_cam = nh.advertise<sensor_msgs::CameraInfo> ("/stereo/right/camera_info", 1);

  ros::Subscriber sub_left_camera = nh.subscribe("/airsim_node/PhysXCar/StereoLeft/Scene/camera_info", 1000, StereoLeftCamCB);
  ros::Subscriber sub_right_camera = nh.subscribe("/airsim_node/PhysXCar/StereoRight/Scene/camera_info", 1000, StereoRightCamCB);

  ros::spin();
  return 0;
}*/