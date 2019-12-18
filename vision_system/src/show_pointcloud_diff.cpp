#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/console/time.h>  //TicToc
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr full_scene(new PointCloudT);  //template imported
PointCloudT::Ptr partial_scene(new PointCloudT);  //template imported
PointCloudT::Ptr full_scene_downsampled(new PointCloudT);
PointCloudT::Ptr partial_scene_downsampled(new PointCloudT);
PointCloudT::Ptr cloud_diff(new PointCloudT);
PointCloudT::Ptr cloud_diff_filtered(new PointCloudT);

pcl::SegmentDifferences<PointT> seg;

int main (int argc, char **argv)
{
    ros::init (argc, argv, "test_pointcloud_diff");
    ros::NodeHandle nh;
//    ros::Subscriber bat_sub = nh.subscribe("input", 10, cloudCB);//接收点云
//    ros::spin();
    if (pcl::io::loadPCDFile("/home/shyreckdc/catkin_ws/src/pbd/resources/scene_with_obj.pcd",*full_scene) < 0) {
        ROS_INFO("Error loading cloud %s. \n", argv[1]);
        return (-1);
    }

    if (pcl::io::loadPCDFile("/home/shyreckdc/catkin_ws/src/pbd/resources/scene_without_obj.pcd",*partial_scene) < 0) {
        ROS_INFO("Error loading cloud %s. \n", argv[2]);
        return (-1);
    }

    pcl::console::TicToc time;
    time.tic();

    pcl::PCLPointCloud2::Ptr full_scene2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr partial_scene2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr full_scene2_downsampled(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr partial_scene2_downsampled(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*full_scene, *full_scene2);
    pcl::toPCLPointCloud2(*partial_scene, *partial_scene2);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (full_scene2);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*full_scene2_downsampled);
    pcl::fromPCLPointCloud2(*full_scene2_downsampled, *full_scene_downsampled);

    sor.setInputCloud (partial_scene2);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*partial_scene2_downsampled);
    pcl::fromPCLPointCloud2(*partial_scene2_downsampled, *partial_scene_downsampled);

    seg.setInputCloud(full_scene_downsampled);
    seg.setTargetCloud(partial_scene_downsampled);
//    seg.setInputCloud(full_scene);
//    seg.setTargetCloud(partial_scene);
//    seg.setSearchMethod(tree);
    seg.setDistanceThreshold(0.001);
    seg.segment(*cloud_diff);

    pcl::StatisticalOutlierRemoval<PointT> sor_rm_outlier;
    sor_rm_outlier.setInputCloud(cloud_diff);
    sor_rm_outlier.setMeanK(30);
    sor_rm_outlier.setStddevMulThresh(1);
    sor_rm_outlier.filter(*cloud_diff_filtered);

    pcl::io::savePCDFileASCII ("cloud_diff_filtered.pcd", *cloud_diff_filtered);//保存pcd



    ROS_INFO("Finished the pose estimation in %f ms", time.toc());

    return 0;
}  
