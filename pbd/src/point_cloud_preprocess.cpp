#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//void cloudCB(const sensor_msgs::PointCloud2 &input)
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sensor_msgs::PointCloud2 ros_passed_cloud;
ros::Publisher passed_cloud_pub;

void pass_function(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double start_x, double end_x,
        double start_y, double end_y, double start_z, double end_z,
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,bool set_negative)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_cloud);
}


void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input, *raw_cloud);


    //crophull will be used later to segment a better scene
    pass_function(raw_cloud,-0.15,0.5,-1,1,-1,1,passed_cloud, false);  // pass through scene


    pcl::toROSMsg(*passed_cloud, ros_passed_cloud);
    ros_passed_cloud.header.frame_id = "camera_depth_optical_frame";
    passed_cloud_pub.publish(ros_passed_cloud);

}
int main (int argc, char **argv)
{
    ros::init (argc, argv, "point_cloud_preprocess");
    ros::NodeHandle node;
    ros::Subscriber cloud_sub = node.subscribe("/camera/depth/points", 10, cloudCB);//接收点云
    passed_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/passed_cloud",1);
    ros::Duration(2).sleep();
    ros::spin();
    return 0;
}  
