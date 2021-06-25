//Porblem: 

//Try the commented out code in case this code does not work out
//This code transforms the incoming point cloud from stereo_image_proc into the baselink_nwu frame

#if 0
//This is the code from a bookmarked page{in stereo_image_proc} (from Github)
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
//#include <pcl_ros/impl/transfoms.hpp>

ros::Publisher pub;
tf::StampedTransform transform;//contains the latest value of the transform of frames info

#if 1
tf::StampedTransform transform;
#endif

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);//convert from ROS to PCL
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert from PCL to PointCloud2

    //std::string target_frame = "PhysXCar/odom_local_nwu"; 
    //transform the point cloud from frame to frame using a convenient function in PCL
    pcl_ros::transformPointCloud("PhysXCar/odom_local_nwu", *temp_cloud, *cloud_transformed, transformStamped);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "transform_pc");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("stereo/points2", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("stereo/points2/transformed", 1);

    #if 1
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    #endif

    tf::TransformListener listener;
    ros::Rate rate(30.0);
    while (nh.ok())
    {
        try{
            //target_frame, source_frame
            //the transformed listened up is stored in "tf::StampedTransform transform"
            #if 0
            listener.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(), transform);
            #endif

            #if 0
            transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(0));
            #endif
            tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time::now(), ros::Duration(3.0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;

}
#endif


//this code compiles, but gives error: 
#if 0
    "Lookup would require extrapolation into the past.  
    Requested time 1590159749.899727880 but the earliest data is at time 1590159749.900203669, 
    when looking up transform from frame [StereoLeft_optical] to frame [PhysXCar/odom_local_nwu]
    "
#endif
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


ros::Publisher pub;
tf::StampedTransform transform;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    //cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);

    tf::TransformListener listener;
    ros::Rate rate(30.0);
    while (nh.ok()){

        try{
            listener.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;

}
#endif


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


ros::Publisher pub;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl_ros::transformPointCloud("PhysXCar/odom_local_nwu", *temp_cloud, *cloud_transformed, transform);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    //cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}

geometry_msgs::TransformStamped transformStamped;

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2/transformed", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);//10Hz
    while (node.ok())
    {
        //geometry_msgs::TransformStamped is of the form - header, frame_id, transform{geometry_msgs/Transform}
        try{
          transformStamped = tfBuffer.lookupTransform("PhysXCar/odom_local_nwu", "StereoLeft_optical", ros::Time(0));//turtle2 is the target frame, turtle1- source frame
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
geometry_msgs::TransformStamped transformStamped;
        rate.sleep();
    }
    return 0;
}
#endif


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

ros::Publisher pub;
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    std::cout<<"Entered CB function"<<std::endl;
    sensor_msgs::PointCloud2 transformed_cloud;
    std::string target_link_name = "/PhysXCar/odom_local_nwu";
    
   /* tf::TransformListener *tf_listener;
    tf::StampedTransform transform;
    try{
        std::cout<<"In try"<<std::endl;
        tf_listener->waitForTransform(target_link_name, input->header.frame_id, ros::Time::now(), ros::Duration(10.0));
        tf_listener->lookupTransform(target_link_name, input->header.frame_id, ros::Time::now(), transform);
        //transformStamped = tfBuffer.lookupTransform("target_link_name", "input->header.frame_id", ros::Time::now(), ros::Duration(3.0));
        std::cout<<"In try"<<std::endl;
    }catch(tf::TransformException ex){
        std::cout<<"In catch"<<std::endl;
        //ROS_ERROR("%s",ex.what());
    }*/

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener(tfBuffer);
    try{
        std::cout<<"In try"<<std::endl;
        /*tf_listener->waitForTransform(target_link_name, input->header.frame_id, ros::Time::now(), ros::Duration(10.0));
        tf_listener->lookupTransform(target_link_name, input->header.frame_id, ros::Time::now(), transform);
       */
        tfBuffer.lookupTransform(target_link_name, input->header.frame_id, ros::Time::now(), ros::Duration(3.0));
       
        std::cout<<"In try"<<std::endl;
    }catch(tf::TransformException ex){
        std::cout<<"In catch"<<std::endl;
        //ROS_ERROR("%s",ex.what());
    }
    
    std::cout<<"Before transforming"<<std::endl;
    if(!(pcl_ros::transformPointCloud(target_link_name, *input,  transformed_cloud, tf_listener))){
        std::cerr<<"Load challa hai"<<std::endl;
        return;
    }
    std::cout<<"After transforming"<<std::endl;
    transformed_cloud.header = input->header;
    transformed_cloud.header.frame_id = target_link_name;
    pub.publish(transformed_cloud);
}

int main (int argc, char** argv)
{
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