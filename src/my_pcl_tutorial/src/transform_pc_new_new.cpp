#if 1
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//geometry_msgs::TransformStamped

class PoseDrawer
{
public:
  PoseDrawer() :
    tf2_(buffer_),  target_frame_("PhysXCar/odom_local_nwu"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "/stereo/points2", 10);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor/points2/transformed", 3);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::TransformStampedConstPtr& point_ptr) 
  {
    geometry_msgs::TransformStamped point_out;
    try 
    {
      buffer_.transform(*point_ptr, point_out, target_frame_);
      
      ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
             point_out.point.x,
             point_out.point.y,
             point_out.point.z);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::TransformStamped> tf2_filter_;
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};
#endif

#if 0
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <transform_point_cloud/LookupTransformConfig.h>

class TransformPointCloud
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string target_frame_;
  double timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

/*  void printPointCloud(const sensor_msgs::PointCloud2& pc, const size_t max_count = 8)
  {
    // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc, "z");
    size_t count = 0;
    for (; (iter_x != iter_x.end()) && (iter_y != iter_y.end()) && (iter_z != iter_z.end()) &&
         (count < max_count);
         ++iter_x, ++iter_y, ++iter_z, ++count)
    {
      ROS_DEBUG_STREAM(count << " " << *iter_x << " " << *iter_y << " " << *iter_z);
    }
  }*/
/*
  void scalePointCloud(sensor_msgs::PointCloud2& pc,
      const double sx, const double sy, const double sz)
  {
    if ((sx == 1.0) && (sy == 1.0) && (sz == 1.0))
      return;
    // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
    size_t count = 0;
    for (; (iter_x != iter_x.end()) && (iter_y != iter_y.end()) && (iter_z != iter_z.end());
         ++iter_x, ++iter_y, ++iter_z, ++count)
    {
      *iter_x *= sx;
      *iter_y *= sy;
      *iter_z *= sz;
    }
  }
*/
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // TODO(lucasw) actually subscribe and unsubscribe in config callback?
    if (!config_.enable)
      return;

    // ROS_INFO_STREAM(msg->header.frame_id << " " << target_frame_);
    ROS_DEBUG_STREAM("input:");
    printPointCloud(*msg, 8);
    geometry_msgs::TransformStamped transform;
    try
    {
      // (target_frame, pc frame) preserves the world coordinates of the point cloud but shifts
      // the parent to target_frame_
      // (pc_frame, target_frame) shifts the point cloud to be relative to the target_frame
      // by the same amount it used to be relative to msg->header.frame_id,
      // but the frame will still be msg->header.frame_id when done.
      const std::string target_frame = (config_.target_frame == "") ? msg->header.frame_id : config_.target_frame;
      const std::string source_frame = (config_.source_frame == "") ? msg->header.frame_id : config_.source_frame;
      transform = tf_buffer_.lookupTransform(
          target_frame,
          source_frame,
          msg->header.stamp + ros::Duration(config_.offset_lookup_time),
          ros::Duration(config_.timeout));
      sensor_msgs::PointCloud2 cloud_out;
      tf2::doTransform(*msg, cloud_out, transform);
      scalePointCloud(cloud_out, config_.scale_x, config_.scale_y, config_.scale_z);
      ROS_DEBUG_STREAM("output:");
      // TODO(lwalter) should the scaling be done on the output?
      printPointCloud(cloud_out, 8);
      pub_.publish(cloud_out);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

public:
  TransformPointCloud() :
    // nh_("~"),
    tf_listener_(tf_buffer_)
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor/points2/transformed", 3);
    sub_ = nh_.subscribe("/sensor/points2", 1, &TransformPointCloud::pointCloudCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_point_cloud");
  TransformPointCloud transform_point_cloud;
  ros::spin();
}
#endif