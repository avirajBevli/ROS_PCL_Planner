#include <ros/ros.h>
#include <cmath>//for sqrt and pow
#include <iostream>
#include <pcl/io/pcd_io.h>

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

#define length_ahead_of_sensor 1
#define dist_motion_command_car 0.5
#define length_ahead_buffer_reqd 0.1
#define breadth_right_of_sensor 0.4
#define breadth_right_buffer_reqd 0.1
#define breadth_left_of_sensor (-0.4)
#define breadth_left_buffer_reqd (-0.1)

#define safety_distance 30

bool coordinate_frame_used = 0;
bool is_safe = 1;
bool is_fixed = 0;

nav_msgs::Odometry nav_odom_msg_global;

//call-back function to update nav_odom_msg_global
void navOdomCB(const nav_msgs::OdometryConstPtr& nav_odom_msg)
{
  if(is_safe){
    nav_odom_msg_global = *nav_odom_msg;
    return;
  }

  if(is_fixed)
    return;
  else
  {
    nav_odom_msg_global = *nav_odom_msg;
    nav_odom_msg_global.twist.twist.linear.x = 0;
    nav_odom_msg_global.twist.twist.linear.y = 0;
    nav_odom_msg_global.twist.twist.linear.z = 0;
    nav_odom_msg_global.twist.twist.angular.x = 0;
    nav_odom_msg_global.twist.twist.angular.y = 0;
    nav_odom_msg_global.twist.twist.angular.z = 0;
    is_fixed=1;
  }

} 

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);
  std_msgs::Bool decision;
  decision.data = true;
  geometry_msgs::Twist vel_msg;
  nav_msgs::Odometry nav_odom;

  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;

  if(is_safe==0)
  {
    decision.data = false;
    pub_bool.publish(decision);
    pub_vel.publish(vel_msg);
    pub_nav_odom.publish(nav_odom_msg_global);
    return;
  }

  decision.data = true;//0 implies not safe, 1 implies it is safe
  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    if( sqrt(pow(cloud.points[i].x,2) + pow(cloud.points[i].y,2)) < safety_distance)
    {
      std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << "       UNSAFE   UNSAFE   UNSAFE !!!" <<std::endl;
      std::cerr << " Changed to false "<<std::endl; 
      decision.data = false;
      is_safe=0;
    }
    else
      std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " " <<std::endl;
  }

  if(is_safe==0)
  {
    decision.data = false;
    pub_bool.publish(decision);
    pub_vel.publish(vel_msg);
    pub_nav_odom.publish(nav_odom_msg_global);
    return;
  }

  vel_msg.linear.x = 15;
  pub_bool.publish(decision);
  pub_vel.publish(vel_msg);
  pub_nav_odom.publish(nav_odom_msg_global);
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_node");
  ros::NodeHandle nh;
  pub_bool = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("/airsim_node/PhysXCar/cmd_vel",1);
  pub_nav_odom = nh.advertise<nav_msgs::Odometry> ("/airsim_node/decide_vel/odom_local_ned",1);

  ros::Subscriber sub = nh.subscribe("airsim_node/PhysXCar/lidar/Lidar3D", 1, cloudCB);
  ros::Subscriber sub2 = nh.subscribe("/airsim_node/PhysXCar/odom_local_ned", 1, navOdomCB);
  
  ros::spin();
  return 0;
}


/*
///airsim_node/PhysXCar/lidar/Lidar3D is the topic we wanna subscribe to
///We will publish the velocit commands to /airsim_node/PhysXCar/cmd_vel topic which is of the type "geometry_msgs/Twist"

//Used for all ROS nodes
#include <ros/ros.h>

#include <cmath>//for sqrt and pow
//pcd_write.cpp used headers
#include <iostream>
#include <pcl/io/pcd_io.h>

// PCL specific includes
// We are dealing with these two ros topic type in this code 
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#include <pcl_conversions/pcl_conversions.h>
//to convert msg types from pcl to ros and ros to pcl

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ros/conversions.h>

ros::Publisher pub;

#define length_ahead_of_sensor 1

#define dist_motion_command_car 0.5
#define length_ahead_buffer_reqd 0.1
#define breadth_right_of_sensor 0.4
#define breadth_right_buffer_reqd 0.1
#define breadth_left_of_sensor (-0.4)
#define breadth_left_buffer_reqd (-0.1)

#define safety_distance 3

bool coordinate_frame_used = 0;
//0 for NED coordinate system, 1 for normal Coordinate system(the one which ROS uses)

bool isColliding(pcl::PointXYZ pt)
{
  double threshold_x = length_ahead_of_sensor + dist_motion_command_car + length_ahead_buffer_reqd;
  double threshold_y_positive = breadth_right_buffer_reqd + breadth_right_of_sensor;
  double threshold_y_negative = breadth_left_buffer_reqd + breadth_left_of_sensor;
  if(coordinate_frame_used == 0)
  {
    if( ( (pt.x < threshold_x)&&(pt.y < threshold_y_positive) ) || (pt.x < threshold_x)&&(pt.y > threshold_y_negative) )
      return 1;
  }

  else if(coordinate_frame_used == 1)
  {
    if( ( (pt.x < threshold_x)&&(pt.y > -1*threshold_y_positive) ) || (pt.x < threshold_x)&&(pt.y < -1*threshold_y_negative) )
      return 1;
  }

  return 0;
  
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std_msgs::Bool decision;
  decision.data = true;//0 implies not safe, 1 implies it is safe
  std::cerr << " Changed to true "<<std::endl; 

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);
  //std::vector<pcl::PointXYZ> data = cloud.points;
  //This for some reason was giving a seg fault

  //code copied from FilterPointCloudByDistance in depth2pointcloud.cpp file
  
  sensor_msgs::PointCloud2 filtered_ptcloud = *cloud_msg;
  sensor_msgs::PointCloud2Iterator<float> out_x(filtered_ptcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(filtered_ptcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(filtered_ptcloud, "z");

  for (sensor_msgs::PointCloud2Iterator<float> it(filtered_ptcloud, "x"); it != it.end(); ++it) 
  {
    float x = it[0];//x coordinate of the point
    float y = it[1];
    float distance = sqrt(x * x + y * y);
    //if (distance < min_distance || distance > max_distance) 
    //{
      //it[0] = std::numeric_limits<float>::quiet_NaN();
      //it[1] = std::numeric_limits<float>::quiet_NaN();
      //it[2] = std::numeric_limits<float>::quiet_NaN();
    //}
    if(distance < safety_distance)
      decision.data = false;
  }

  //////////////////////////////////////////////////////
  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    //std::cerr was just a sanity check that pcl is doing a fine job
    //std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    if( isColliding(cloud.points[i]) )
        decision.data = false;
      if( sqrt(pow(cloud.points[i].x,2) + pow(cloud.points[i].y,2)) < safety_distance)
      {
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << "       UNSAFE   UNSAFE   UNSAFE !!!" <<std::endl;
        std::cerr << " Changed to false "<<std::endl; 
        decision.data = false;
      }

      else
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " " <<std::endl;

  }

  pub.publish(decision);
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("airsim_node/PhysXCar/lidar/Lidar3D", 1, cloudCB);
  pub = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  
  ros::spin();
  return 0;
}
*/