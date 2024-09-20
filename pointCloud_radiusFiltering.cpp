#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
 
 
ros::Publisher pub;
 
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
 
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  // Perform the actual filtering
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
     // build the filter  
    outrem.setInputCloud(cloudPtr);
    outrem.setRadiusSearch(0.08);
    outrem.setMinNeighborsInRadius (60);
     // apply filter
    outrem.filter (cloud_filtered);
 
  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_rad;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_rad);
 
  // Publish the data
  pub.publish (cloud_rad);
}
 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "RadiusOutlierRemoval");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/filtered_VoxelGrid", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_RadiusOutlierRemoval", 1);
 
  // Spin
  ros::spin ();
}