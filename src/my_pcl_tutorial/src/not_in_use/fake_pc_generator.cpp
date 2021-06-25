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
  int n = cloud.size();
  for (std::size_t i = 0; i < cloud.size(); ++i)
    cloud.points[i].z = 10;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  pub_point_cloud.publish(output);
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_pc_generator");
  ros::NodeHandle nh;
  pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/fake_pc", 1);
  ros::Subscriber sub = nh.subscribe("/stereo/points2", 1, cloudCB);
  ros::spin();
  return 0;
}
