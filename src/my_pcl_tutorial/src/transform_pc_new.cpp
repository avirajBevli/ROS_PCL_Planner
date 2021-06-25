#if 0
#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

ros::Publisher pub;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{   
    std::string target_frame = "PhysXCar/odom_local_nwu"; //input->header.frame_id
    sensor_msgs::PointCloud2 transformed_cloud;
   
    //pcl_ros::transformPointCloud(target_frame, *input, transformed_cloud, transformStamped);
    tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
    std::string target_link_name = "/PhysXCar/odom_local_nwu";
    
	const auto scan_tf = tfBuffer.lookupTransform(target_link_name, input->header.frame_id, ros::Time::now(), ros::Duration(3.0));//turtle2 is the target frame, turtle1- source frame
/*	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
	}*/

	transformed_cloud.header.frame_id = target_link_name;
	pcl_ros::transformPointCloud(tf2::transformToEigen(scan_tf.transform).matrix(), *input, transformed_cloud);

    transformed_cloud.header = input->header;
    transformed_cloud.header.frame_id = target_link_name;
    pub.publish(transformed_cloud);
}

#if 0
const auto scan_tf = tf_buffer.lookupTransform(robot_frame, scan.header.frame_id, scan.header.stamp, ros::Duration(1));
point_cloud.header.frame_id = robot_frame;
pcl_ros::transformPointCloud(tf2::transformToEigen(scan_tf.transform).matrix(),scan, point_cloud);
#endif

int main(int argc, char** argv){
	ROS_INFO("Node started");
    ros::init (argc, argv, "transform_pc_new");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);
    ros::spin ();
};
#endif

#if 0
//This code gave the error that "PhysXCar/odom_local_nwu" passed to lookupTransform argument target_frame does not exist.
#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    sensor_msgs::PointCloud2 cloud_out;

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(0));
        std::cout << transformStamped << std::endl;
        tf2::doTransform(*input, cloud_out, transformStamped);
        pub.publish(cloud_out);
        std::cout<<"Published point cloud"<<std::endl;

    } catch (tf2::TransformException &ex) {
        std::cout<<"inside catch"<<std::endl;
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "transform_pc_new");
    ros::NodeHandle node;

    ROS_INFO("Node started");
    ros::init (argc, argv, "transform_pc");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);
    ros::spin ();
}
#endif


//this code is not working
#if 0
#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;
//geometry_msgs::TransformStamped transformStamped;
/*void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO("Entered cloud_callback\n");
    sensor_msgs::PointCloud2 cloud_out;
    tf2::doTransform(*input, cloud_out, transformStamped);
    pub.publish(cloud_out);
    std::cout<<"Published point cloud"<<std::endl;
}
*/

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "transform_pc_new");
    //ROS_INFO("Node started");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);
    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);

    ros::Rate rate(10.0);//10Hz
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try{
            //this gives optical frame does not exist error
            transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_ned", "StereoLeft_optical", ros::Time(0));
           
            //this gives extrapolation into the future error
            //transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time::now() - ros::Duration(3.0));
            
            //std::cout << transformStamped << std::endl;
        }
        catch (tf2::TransformException &ex) {
            std::cout<<"Inside catch"<<std::endl;
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }

    return 0;
}
#endif


//Rest everyting good, but callback function is not getting fired
#if 0
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

#include <tf2/convert.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO("Entered cloud_callback\n");
    sensor_msgs::PointCloud2 cloud_out;
    tf2::doTransform(*input, cloud_out, transformStamped);
    pub.publish(cloud_out);
    std::cout<<"Published point cloud"<<std::endl;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_pc_new");
  ROS_INFO("Entered node\n");
  std::cout<<"Entered node\n"<<std::endl;
  ros::NodeHandle node;
  std::cout<<"Before subs"<<std::endl;
  // Create a ROS subscriber for the input point cloud
    // Create a ROS publisher for the output point cloud
  pub = node.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);
  ros::Subscriber sub = node.subscribe("/stereo/points2", 1, cloud_callback);
/*
  pub = node.advertise<sensor_msgs::PointCloud2> ("/stereo/points2_upright", 1);
  ros::Subscriber sub = node.subscribe("/stereo/points2", 1, cloudCB);
*/
  std::cout<<"After publisher"<<std::endl;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::cout<<"Defined listenere"<<std::endl;
  ros::Rate rate(10.0);//10Hz
  while (node.ok())
  {
    std::cout<<"Inside while"<<std::endl;
    //geometry_msgs::TransformStamped is of the form - header, frame_id, transform{geometry_msgs/Transform}
    try{
        ROS_INFO("Entered try\n");
      transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(0));//turtle2 is the target frame, turtle1- source frame
        std::cout<<"trye successful"<<std::endl;    
    }
    catch (tf2::TransformException &ex) {
      std::cout<<"entered catch"<<std::endl;
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      std::cout<<"exit catch"<<std::endl;
      //continue;
    }
    rate.sleep();
  }
  return 0;
};
#endif



#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Publisher pub;
tf::TransformListener *tf_listener; 

void callback(const PointCloud::ConstPtr& in)
{
  tf_listener->waitForTransform("/torso", "/camera_link", ros::Time(0), ros::Duration(10.0));
  PointCloud out;
  pcl_ros::transformPointCloud("/base", *in, out, *tf_listener);
  pub.publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_node");
  ros::NodeHandle nh1;
  pub = nh1.advertise<PointCloud> ("transformed_points", 1);
  
  tf_listener = new tf::TransformListener();

  ros::NodeHandle nh2;
  ros::Subscriber sub = nh2.subscribe<PointCloud>("/camera/depth_registered/points", 1, callback);

  ros::spin();
}

#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

ros::Publisher pub;
tf::TransformListener *tf_listener; 

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{   
    sensor_msgs::PointCloud2 transformed_cloud;
   
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::string target_link_name = "/PhysXCar/odom_local_nwu";
    
    tf_listener->waitForTransform(target_link_name, input->header.frame_id, ros::Time(0), ros::Duration(10.0));

    pcl_ros::transformPointCloud("/base", *input, transformed_cloud, *tf_listener);
    transformed_cloud.header = input->header;
    transformed_cloud.header.frame_id = target_link_name;
    pub.publish(transformed_cloud);
}

int main(int argc, char** argv){
    ROS_INFO("Node started");
    ros::init (argc, argv, "transform_pc_new");
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::Subscriber sub = nh1.subscribe ("/stereo/points2", 1, cloud_callback);
    tf_listener = new tf::TransformListener();
    pub = nh2.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);
    ros::spin ();
};