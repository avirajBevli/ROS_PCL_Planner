//We want the odometry of bot at times when it is unsafe

#include <ros/ros.h>
#include <cmath>//for sqrt and pow
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>//for pcl::removeNaNFromPointCloud

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

ros::Publisher pub_bool;
ros::Publisher pub_vel;
ros::Publisher pub_nav_odom;

#define safety_distance 3
//3.5 was reasonable for Lidar3D pointcloud
#define ground_threshold 1.0
bool traj_straight = 0;
bool traj_right = 0;
bool traj_left = 0;

bool coordinate_frame_used = 0;
bool is_safe = 1;
bool is_fixed = 0;//have we already fixed nav_odom_msg's velocities(to be publlished) to be zero once it is known that it is now unsafe

nav_msgs::Odometry nav_odom_msg_global;

//call-back function to update nav_odom_msg_global
void navOdomCB(const nav_msgs::OdometryConstPtr& nav_odom_msg)
{
    nav_odom_msg_global = *nav_odom_msg;
    return; 
} 

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("Entered cloudCB function\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud_in);

  std::vector<int> ind;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, ind);

  std_msgs::Bool decision;
  decision.data = true;
  geometry_msgs::Twist vel_msg;
  nav_msgs::Odometry nav_odom;

  is_safe = 1;
  if(!(traj_straight || traj_left || traj_right))
    traj_straight = 1;

  decision.data = true;//0 implies not safe, 1 implies it is safe
  for (std::size_t i = 0; i < cloud_out->size(); ++i)
  {
    if(traj_straight){
      if( ( (cloud_out->points[i].x < 15) && (cloud_out->points[i].y < 1) && (cloud_out->points[i].y > -1)  ) && (cloud_out->points[i].z > ground_threshold) ){
        decision.data = false;
        is_safe=0;
      }
    }

    else if(traj_left){
      if( ( (cloud_out->points[i].y < 5) && (cloud_out->points[i].y > 0) && (cloud_out->points[i].x < 5)  ) && (cloud_out->points[i].z > ground_threshold) ){
        decision.data = false;
        is_safe=0;
      }
    }

    else if(traj_right){
      if( ( (cloud_out->points[i].y > -5) && (cloud_out->points[i].y < 0) && (cloud_out->points[i].x < 5)  ) && (cloud_out->points[i].z > ground_threshold) ){
        decision.data = false;
        is_safe=0;
      }
    }
  }

  pub_bool.publish(decision);
  if(!is_safe){
    ROS_INFO("UnSafe!!!\n");
    pub_nav_odom.publish(nav_odom_msg_global);
  }
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_stereo_exp");
  ros::NodeHandle nh;
  pub_bool = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("/airsim_node/PhysXCar/cmd_vel",1);
  pub_nav_odom = nh.advertise<nav_msgs::Odometry> ("/airsim_node/decide_vel/odom_local_ned",1);

  //ros::Subscriber sub = nh.subscribe("/stereo/points2", 1, cloudCB);
  ros::Subscriber sub = nh.subscribe("transformed_points", 1, cloudCB);

  ros::Subscriber sub2 = nh.subscribe("/airsim_node/PhysXCar/odom_local_ned", 1, navOdomCB);
  
  ros::spin();
  return 0;
}
