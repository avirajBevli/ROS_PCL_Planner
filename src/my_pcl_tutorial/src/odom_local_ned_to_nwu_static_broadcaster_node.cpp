//Running this code means rosrun tf tf_echo optical_frame odom_local_nwu gives values
#if 1
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "odom_local_ned_to_nwu_static_broadcaster_node");
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "PhysXCar/odom_local_ned";
  static_transformStamped.child_frame_id = "PhysXCar/odom_local_nwu";//define a new static frame
  static_transformStamped.transform.translation.x = 0.0;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(3.14, 0, 0);// Create this quaternion from roll/pitch/yaw (in radians)
  static_transformStamped.transform.rotation.x = q.x();
  static_transformStamped.transform.rotation.y = q.y();
  static_transformStamped.transform.rotation.z = q.z();
  static_transformStamped.transform.rotation.w = q.w();

  //to broadcast the static transform over ros tf
  static_broadcaster.sendTransform(static_transformStamped);
  
  ROS_INFO("Spinning until killed publishing PhysXCar/odom_local_ned to PhysXCar/odom_local_nwu");
  ros::spin();
  return 0;
};
#endif


//Running this code means rosrun tf tf_echo optical_frame odom_local_nwu does not give values
#if 0
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "adding_frame");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "PhysXCar/odom_local_ned";
  transformStamped.child_frame_id = "PhysXCar/odom_local_nwu";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(1.5708, 0, 0);// Create this quaternion from roll/pitch/yaw (in radians)
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  //publish the added frame at 10Hz in tf ROS
  ros::Rate rate(100.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};
#endif