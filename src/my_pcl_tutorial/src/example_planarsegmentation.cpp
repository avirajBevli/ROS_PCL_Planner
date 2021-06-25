//Used for all ROS nodes
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
//we are dealing with this ros topic type in this code 
#include <pcl_conversions/pcl_conversions.h>
//to convert msg types from pcl to ros and ros to pcl

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//specific headers required from this particular example
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//THis is the difference from the voxel_grip wala example
/////////////////////////////////
#include <pcl/ros/conversions.h>
/////////////////////////////////

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

  ///////////////////////////////////////////////////////////////
  ///////////Very useful: convert a topic from the ros point 
  ///cloud message format(sensor_msgs/PointCloud2)
  ///to the pcl/PointCloud format
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  ///////////////////////////////////////////////////////////////

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  // Observe that ROS can publish a pcl_message type, which is not a standard ROS message type
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}