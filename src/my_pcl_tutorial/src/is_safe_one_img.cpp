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

#include <vector>
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit function

#define ground_threshold 1.0
bool traj_straight = 0;
bool traj_right = 0;
bool traj_left = 0;

typedef struct triplet
{
  float x;
  float y;
  float z;
}triplet;

void PrintVec(std::vector<triplet> unsafe_points)
{
  if(!unsafe_points.size())
    return;
  std::cerr<<"The unsafe points are: "<<std::endl;
  for(int i=0;i<unsafe_points.size();i++)
    std::cerr<<unsafe_points[i].x<<" "<<unsafe_points[i].y<<" "<<unsafe_points[i].z<<std::endl;
  return;
}

bool is_seen = 0;
std::vector<triplet> unsafe_points;

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if(is_seen)
    return;
  
  is_seen = 1;
  ROS_INFO("Entered cloudCB function\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud_in);

  std::vector<int> ind;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, ind);

  bool is_safe =1;

  if(!(traj_straight || traj_left || traj_right))
    traj_straight = 1;

  for (std::size_t i = 0; i < cloud_out->size(); ++i)
  {
    if(traj_straight){
      if( ( (cloud_out->points[i].x < 20) && (cloud_out->points[i].y < 1) && (cloud_out->points[i].y > -1)  ) && (cloud_out->points[i].z > ground_threshold) ){
        is_safe=0;
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
      }
    }

    else if(traj_left){
      if( ( (cloud_out->points[i].y < 15) && (cloud_out->points[i].y > 0) && (cloud_out->points[i].x < 12)  ) && (cloud_out->points[i].z > ground_threshold) ){
        is_safe=0;
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
      }
    }

    else if(traj_right){
      if( ( (cloud_out->points[i].y > -5) && (cloud_out->points[i].y < 0) && (cloud_out->points[i].x < 8)  ) && (cloud_out->points[i].z > ground_threshold) ){
        is_safe=0;
        triplet temp_triplet;
        temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
        unsafe_points.push_back(temp_triplet);
      }
    }
  }

  if(is_safe==0)
    ROS_INFO("Unsafe!!!");

  ofstream outdata;
  PrintVec(unsafe_points);
  outdata.open("unsafe_points.txt"); // opens the file
  if( !outdata ) { // file couldn't be opened
    std::cerr << "Error: file could not be opened" << std::endl;
    exit(1);
  }

  if(!unsafe_points.size())
    std::cerr << "Safe config" << std::endl;
  for (int i=0; i< unsafe_points.size(); ++i)
    outdata << unsafe_points[i].x << " "<< unsafe_points[i].y << " "<< unsafe_points[i].z << std::endl;
  outdata.close();
}	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_it_safe_stereo");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/transformed_points", 1, cloudCB);
  ros::spin();
  return 0;
}
