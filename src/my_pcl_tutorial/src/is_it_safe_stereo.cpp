//This is the final code that we want

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

#define length_ahead_of_sensor 1
#define dist_motion_command_car 0.5
#define length_ahead_buffer_reqd 0.1
#define breadth_right_of_sensor 0.4
#define breadth_right_buffer_reqd 0.1
#define breadth_left_of_sensor (-0.4)
#define breadth_left_buffer_reqd (-0.1)

#define safety_distance 3.5
//3.5 was reasonable for Lidar3D pointcloud

bool coordinate_frame_used = 0;
bool is_safe = 1;
bool is_fixed = 0;//have we already fixed nav_odom_msg's velocities(to be publlished) to be zero once it is known that it is now unsafe

nav_msgs::Odometry nav_odom_msg_global;

//call-back function to update nav_odom_msg_global
void navOdomCB(const nav_msgs::OdometryConstPtr& nav_odom_msg)
{
  if(is_safe){
    nav_odom_msg_global = *nav_odom_msg;
    return;
  }

  if(is_fixed){
    /////to be commented
    //nav_odom_msg_global = *nav_odom_msg;
    return;
  }
  else
  {
    nav_odom_msg_global = *nav_odom_msg;
/////to be uncommented
    
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
  ROS_INFO("Entered cloudCB function\n");
  //typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  /*pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud_in);

  std::vector<int> ind;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, ind);

/*  std::cout << "no. of pts in input=" << cloud_in->size() << std::endl;
  std::cout << "no. of pts in output="<< cloud_out->size() << std::endl;
*/
/*  for (int i=0;i< cloud_out->size();i++ )
  {
    std::cout <<"point["<<i<<"]=" << cloud_out->points[i].x << ","
    << cloud_out->points[i].y << ","
    << cloud_out->points[i].z << std::endl;
  }*/

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

/////UNCOMMENT THIS 

  if(is_safe==0)
  {
    //ROS_INFO("UNSAFE\n");
    decision.data = false;
    pub_bool.publish(decision);
    pub_vel.publish(vel_msg);
    //ROS_INFO("Publishing velocity command\n");
    pub_nav_odom.publish(nav_odom_msg_global);
    return;
  }

  decision.data = true;//0 implies not safe, 1 implies it is safe
  for (std::size_t i = 0; i < cloud_out->size(); ++i)
  {
    if( ( sqrt(pow(cloud_out->points[i].x,2) + pow(cloud_out->points[i].y,2)) < safety_distance )
           && ( (cloud_out->points[i].z > 13) || (cloud_out->points[i].z < 7) ) && (cloud_out->points[i].y < 0.1 && cloud_out->points[i].y > -0.1) && (cloud_out->points[i].x > 0) )
    {
      std::cerr << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << "       UNSAFE   UNSAFE   UNSAFE !!!" <<std::endl;
      //std::cerr << " Changed to false "<<std::endl; 
      decision.data = false;
      is_safe=0;
    }
    //else
      //std::cerr << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << " " <<std::endl;
  }

//UNCOMMENT THIS

  if(is_safe==0)
  {
    decision.data = false;
    pub_bool.publish(decision);
    pub_vel.publish(vel_msg);
    //ROS_INFO("Publishing velocity command\n");
    pub_nav_odom.publish(nav_odom_msg_global);
    return;
  }

  vel_msg.linear.x = 15;
  pub_bool.publish(decision);
  pub_vel.publish(vel_msg);
  //ROS_INFO("Publishing velocity command\n");
  pub_nav_odom.publish(nav_odom_msg_global);
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_stereo");
  ros::NodeHandle nh;
  pub_bool = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("/airsim_node/PhysXCar/cmd_vel",1);
  pub_nav_odom = nh.advertise<nav_msgs::Odometry> ("/airsim_node/decide_vel/odom_local_ned",1);

  //ros::Subscriber sub = nh.subscribe("/stereo/points2", 1, cloudCB);
  ros::Subscriber sub = nh.subscribe("/stereo/points2_upright", 1, cloudCB);
  ros::Subscriber sub2 = nh.subscribe("/airsim_node/PhysXCar/odom_local_ned", 1, navOdomCB);
  
  ros::spin();
  return 0;
}
