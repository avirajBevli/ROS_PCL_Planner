#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub_point_cloud;

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("Entered cloudCB function\n");
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);
  for (std::size_t i = 0; i < cloud.size(); ++i){
    cloud.points[i].y = -1*cloud.points[i].y; 
  	cloud.points[i].x = -1*cloud.points[i].x;
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  pub_point_cloud.publish(output);
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "invert_pc");
  ros::NodeHandle nh;
  pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2_upright", 1);
  ros::Subscriber sub = nh.subscribe("/stereo/points2", 1, cloudCB);
  ros::spin();
  return 0;
}
