#include<ros/ros.h>  
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  
  
//void cloudCB(const sensor_msgs::PointCloud2 &input)  
void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)  
{  
  pcl::PointCloud<pcl::PointXYZ> cloud;  
//  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息  
  pcl::fromPCLPointCloud2(*input, cloud);//从ROS类型消息转为PCL类型消息  
  pcl::io::savePCDFileASCII ("assembly_base.pcd", cloud);//保存pcd  
}  
main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcd_write");  
  ros::NodeHandle nh;  
  ros::Subscriber bat_sub = nh.subscribe("input", 10, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
}  
