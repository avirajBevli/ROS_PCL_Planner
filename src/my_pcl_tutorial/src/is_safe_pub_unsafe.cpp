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

ros::Publisher pub_obstacles;
ros::Publisher pub_bool;
ros::Publisher pub_vel;
ros::Publisher pub_nav_odom;

#define safety_distance 3
//3.5 was reasonable for Lidar3D pointcloud
#define ground_threshold 1.0
#define threshold_number_of_points_for_obstacle 1000
bool traj_straight = 0;
bool traj_right = 1;
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

typedef struct triplet
{
  float x;
  float y;
  float z;
}triplet;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//pcl::toROSMsg function changes into the ros pointcloud form

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //ROS_INFO("Entered cloudCB function\n");
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

  PointCloud::Ptr msg (new PointCloud);
  std::vector<triplet> unsafe_points;
  decision.data = true;//0 implies not safe, 1 implies it is safe
  int num_unsafe_points = 0;
  for (std::size_t i = 0; i < cloud_out->size(); ++i)
  {
    if(traj_straight){
      if( ( (cloud_out->points[i].x < 25) && (cloud_out->points[i].y < 1) && (cloud_out->points[i].y > -1)  ) && (cloud_out->points[i].z > ground_threshold) ){
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
        num_unsafe_points++;
      }
    }

    else if(traj_left){
      if( ( (cloud_out->points[i].y < 12) && (cloud_out->points[i].y > 0) && (cloud_out->points[i].x < 8)  ) && (cloud_out->points[i].z > ground_threshold) ){
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
        num_unsafe_points++;
      }
    }

    else if(traj_right){
      if( ( (cloud_out->points[i].y > -10) && (cloud_out->points[i].y < 0) && (cloud_out->points[i].x < 8)  ) && (cloud_out->points[i].z > ground_threshold) ){
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
        num_unsafe_points++;
      }
    }
  }

  if(num_unsafe_points > threshold_number_of_points_for_obstacle)
  {
  	for(int i=0;i<num_unsafe_points;i++){
  		msg->points.push_back (pcl::PointXYZ(unsafe_points[i].x, unsafe_points[i].y, unsafe_points[i].z));
  		//std::cerr<<unsafe_points[i].x<<" "<<unsafe_points[i].y<<" "<<unsafe_points[i].z<<std::endl;
  	}
  	decision.data = false;
    is_safe=0;

  	ROS_INFO("UnSafe!!! %d unsafe points found!!!\n", num_unsafe_points);
  	sensor_msgs::PointCloud2 cloud_publish;
	pcl::toROSMsg(*msg,cloud_publish);
	cloud_publish.header = cloud_msg->header;
	cloud_publish.height = 1;
	cloud_publish.width = num_unsafe_points;
	
	pub_obstacles.publish(cloud_publish);
  	return;
  }

  sensor_msgs::PointCloud2 cloud_publish;
  pcl::toROSMsg(*msg,cloud_publish);
  cloud_publish.header = cloud_msg->header;
  cloud_publish.height = 1;
  cloud_publish.width = 0;
  pub_obstacles.publish(cloud_publish);

  pub_nav_odom.publish(nav_odom_msg_global);

  pub_bool.publish(decision);
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_stereo_exp");
  ros::NodeHandle nh;
  
  pub_obstacles = nh.advertise<sensor_msgs::PointCloud2> ("obstacles", 1);
  pub_bool = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("/airsim_node/PhysXCar/cmd_vel",1);
  pub_nav_odom = nh.advertise<nav_msgs::Odometry> ("/airsim_node/decide_vel/odom_local_ned",1);

  //ros::Subscriber sub = nh.subscribe("/stereo/points2", 1, cloudCB);
  ros::Subscriber sub = nh.subscribe("transformed_points", 1, cloudCB);
  ros::Subscriber sub2 = nh.subscribe("/airsim_node/PhysXCar/odom_local_ned", 1, navOdomCB);
  
  ros::spin();
  return 0;
}
