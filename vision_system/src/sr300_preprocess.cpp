#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
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
#include <pcl/filters/voxel_grid.h>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_down_sampled2(new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    double num, thresh, leaf_size;
    ros::param::get("/points_num",num);
    ros::param::get("/thresh",thresh);
    ros::param::get("/leaf_size",leaf_size);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> down_sample;
    down_sample.setInputCloud (input);

    down_sample.setLeafSize (leaf_size,leaf_size,leaf_size);
    down_sample.filter (*cloud_down_sampled2);

    pcl::fromPCLPointCloud2(*cloud_down_sampled2, *cloud_down_sampled);

    pcl::io::savePCDFileASCII ("scene_downsampled.pcd", *cloud_down_sampled);//保存pcd

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_down_sampled);
    sor.setMeanK (num);
    sor.setStddevMulThresh (thresh);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::io::savePCDFileASCII ("scene_filtered.pcd", *cloud_filtered);//保存pcd


    pcl::toROSMsg(*cloud_filtered, ros_passed_cloud);
    ros_passed_cloud.header.frame_id = "sr300_depth_optical_frame";
    passed_cloud_pub.publish(ros_passed_cloud);

}
int main (int argc, char **argv)
{
    ros::init (argc, argv, "sr300_preprocess");
    ros::NodeHandle node;
    ros::Subscriber cloud_sub = node.subscribe("/sr300/depth_registered/points", 1, cloudCB);//接收点云
    passed_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/sr300_passed_cloud",1);
    ros::param::set("/points_num", 75);
    ros::param::set("/thresh",0.5);
    ros::param::set("/leaf_size",0.003);
    ros::Duration(4).sleep(); // wait for camera msg
    ros::spin();
    return 0;
}  
