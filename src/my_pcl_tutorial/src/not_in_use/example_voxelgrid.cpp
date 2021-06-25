//Used for all ROS nodes
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
//we are dealing with this ros topic type in this code 
#include <pcl_conversions/pcl_conversions.h>
//to convert msg types from pcl to ros and ros to pcl

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//specific header for this example
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert a ROS topic type to PCL data type
  ////////////////////////////////////////////
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  ////////////////////////////////////////////

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  /////////////////////////////////////////////////////
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  /////////////////////////////////////////////////////

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // Subscribes to the input topic, which is of sensor_msgs::PointCloud2 type
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  // Published to the output topic, which is of sensor_msgs::PointCloud2 type, after doing some computation by the callback function, cloud_cb
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
