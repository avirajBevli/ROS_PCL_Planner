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

#include <tf/transform_broadcaster.h>

ros::Publisher pub_obstacles;
ros::Publisher pub_bool;
ros::Publisher pub_vel;
ros::Publisher pub_nav_odom;
ros::Publisher pub_traj;

float v_tangential = 3;//metres per second
float v_angular = 0.0745329;//4 radians per second

//parameters to set the trajectory vector, std::vector<triplet> car_trajectory;
float time_between_readings = 10.00;
int num_time_divisions = 10;

double vx = 2;
double vy = 1;
double vtheeta = -0.15;//+ve means a left turn, -ve means a right turn

#define safety_threshold_distance 3
//3.5 was reasonable for Lidar3D pointcloud
#define ground_threshold 1.0
#define threshold_number_of_points_for_obstacle 1000

bool is_safe_global = 1;
bool is_fixed_global = 0;
//have we already fixed nav_odom_msg's velocities(to be publlished) to be zero once it is known that it is now unsafe

nav_msgs::Odometry nav_odom_msg_global;

void navOdomCB(const nav_msgs::OdometryConstPtr& nav_odom_msg)
{
    if(is_safe_global){
      nav_odom_msg_global = *nav_odom_msg;
      return;
    }

    if(is_fixed_global)
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
      is_fixed_global=1;
    }
} 

typedef struct triplet
{
  float x;
  float y;
  float z;
}triplet;

std::vector<triplet> car_trajectory;

void Publish_car_traj()
{
  //ROS_INFO("Entered Publish_car_traj function\n");
  //compute odometry in a typical way given the velocities of the robot 
  tf2_ros::TransformBroadcaster odom_broadcaster;
  for(int i=0;i<num_time_divisions;i++)
  {
    //ROS_INFO("Inside loop number %d\n",i);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(car_trajectory[i].z);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "PhysXCar/odom_local_nwu";
    odom_trans.child_frame_id = "PhysXCar/traj_wrto_odom_local_nwu";

    odom_trans.transform.translation.x = car_trajectory[i].x;
    odom_trans.transform.translation.y = car_trajectory[i].y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //ROS_INFO("Sent transforation\n");

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "PhysXCar/odom_local_nwu";

    //set the position
    odom.pose.pose.position.x = car_trajectory[i].x;
    odom.pose.pose.position.y = car_trajectory[i].y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "PhysXCar/traj_wrto_odom_local_nwu";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vtheeta;

    //ROS_INFO("Before publishing\n");
    //publish the message
    pub_traj.publish(odom);
    //ROS_INFO("Aftere publishing\n");
  }
}

void findCarTraj()
{
  ROS_INFO("Entered findCarTraj function\n");
  float x_coord=0;
  float y_coord=0;
  float theeta=0;

  float time_inderval = time_between_readings/num_time_divisions;
  double dt = (double)time_inderval;
  triplet temp_triplet;//z represents theeta

  for(int i=0;i<num_time_divisions;i++)
  {
    temp_triplet.x = x_coord; temp_triplet.y = y_coord; temp_triplet.z = theeta;
    car_trajectory.push_back(temp_triplet);

    double delta_x = (vx * cos(theeta) - vy * sin(theeta)) * dt;
    double delta_y = (vx * sin(theeta) + vy * cos(theeta)) * dt;
    double delta_th = vtheeta * dt;

    x_coord += delta_x;
    y_coord += delta_y;
    theeta += delta_th;
  }
}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//pcl::toROSMsg function changes into the ros pointcloud form

bool isDangerous(triplet temp_triplet){
  for(int i=0;i<car_trajectory.size();i++){
    if( sqrt( pow(car_trajectory[i].x - temp_triplet.x, 2) + pow(car_trajectory[i].y - temp_triplet.y, 2) ) < safety_threshold_distance ){
      if( (temp_triplet.z > 1) && (temp_triplet.z < 5) )
        return 1;
    }
  }
  return 0;
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //ROS_INFO("Entered cloudCB function\n");
  Publish_car_traj();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud_in);

  std::vector<int> ind;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, ind);

  std_msgs::Bool decision;
  decision.data = true;
  geometry_msgs::Twist vel_msg;
  nav_msgs::Odometry nav_odom;

  PointCloud::Ptr msg (new PointCloud);
  std::vector<triplet> unsafe_points;
  decision.data = true;//0 implies not safe, 1 implies it is safe
  int num_unsafe_points = 0;
  for (std::size_t i = 0; i < cloud_out->size(); ++i)
  {
    triplet temp_triplet;
    temp_triplet.x = cloud_out->points[i].x; temp_triplet.y = cloud_out->points[i].y; temp_triplet.z = cloud_out->points[i].z; 
    if( isDangerous(temp_triplet) )
    {
      unsafe_points.push_back(temp_triplet);
      num_unsafe_points++;
    }
  }

  if(num_unsafe_points > threshold_number_of_points_for_obstacle)
  {
    for(int i=0;i<num_unsafe_points;i++){
      msg->points.push_back (pcl::PointXYZ(unsafe_points[i].x, unsafe_points[i].y, unsafe_points[i].z));
      //std::cerr<<unsafe_points[i].x<<" "<<unsafe_points[i].y<<" "<<unsafe_points[i].z<<std::endl;
    }
    decision.data = false;
    is_safe_global=0;

    ROS_INFO("UnSafe!!! %d unsafe points found!!!\n", num_unsafe_points);
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*msg,cloud_publish);
    cloud_publish.header = cloud_msg->header;  
    cloud_publish.height = 1;
    cloud_publish.width = num_unsafe_points;
    
    pub_obstacles.publish(cloud_publish);
    pub_nav_odom.publish(nav_odom_msg_global);
    pub_bool.publish(decision); 
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
  ROS_INFO("Started node\n");

  pub_traj = nh.advertise<nav_msgs::Odometry> ("hypothetical_trajectory", 50);
  pub_obstacles = nh.advertise<sensor_msgs::PointCloud2> ("obstacles", 1);
  pub_bool = nh.advertise<std_msgs::Bool> ("is_it_safe", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("/airsim_node/PhysXCar/cmd_vel",1);
  pub_nav_odom = nh.advertise<nav_msgs::Odometry> ("/airsim_node/decide_vel/odom_local_ned",1);
  findCarTraj();

  ros::Subscriber sub = nh.subscribe("transformed_points", 1, cloudCB);
  ros::Subscriber sub2 = nh.subscribe("/airsim_node/PhysXCar/odom_local_mod", 1, navOdomCB);
  
  ros::spin();
  return 0;
}