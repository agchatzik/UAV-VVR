 #include <ros/ros.h>
 #include <sensor_msgs/PointCloud2.h>

 #include <pcl/io/pcd_io.h>
 #include <pcl/io/ply_io.h>

 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl_ros/point_cloud.h>

 #include <pcl/filters/voxel_grid.h>
 #include <pcl/features/normal_3d.h>

 #include <pcl/console/parse.h>
 #include <pcl/common/transforms.h>
 #include <pcl/filters/statistical_outlier_removal.h>

 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include <sstream>

 #include <chrono>

 using namespace std::chrono;

 typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

 PointCloud::Ptr input_cloud (new PointCloud);
 PointCloud::Ptr msg (new PointCloud);


 int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_publisher");
    ros::NodeHandle nh;

    /// Take PointCloud from guidance

//    ros::Subscriber sub = nh.subscribe<PointCloud>("/points2", 1, callback); // points from nodelet // points2 from pc_from_depth
//
//    msg->header.frame_id = "map";

    ros::Publisher PC_pub = nh.advertise<PointCloud>("landing_point", 10);
    PointCloud cloud;
    cloud.push_back(pcl::PointXYZ(5,5,5));
    

     ros::Rate loop_rate(4);
     while (nh.ok())
     {
           pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
           PC_pub.publish (cloud.makeShared());
           ros::spinOnce ();
           loop_rate.sleep ();
     }
 }
