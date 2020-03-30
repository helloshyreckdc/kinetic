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
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>

using namespace std;

//void cloudCB(const sensor_msgs::PointCloud2 &input)
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
sensor_msgs::PointCloud2 ros_passed_cloud;
ros::Publisher passed_cloud_pub;

void pass_function(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, double start_x, double end_x,
        double start_y, double end_y, double start_z, double end_z,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,bool set_negative)
{
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_cloud);
}

void pass_function(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, double start_x, double end_x,
                   double start_y, double end_y, double start_z, double end_z,
                   pcl::IndicesPtr output_indices,pcl::IndicesPtr removed_indices, bool set_negative)
{
    pcl::CropBox<pcl::PointXYZRGB> boxFilter(true);   // set to true to get removed indices
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_indices);
    pcl::IndicesConstPtr removed_temp = boxFilter.getRemovedIndices(); // to solve const pointer and non const
    *removed_indices = *removed_temp;
}


void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*input, *raw_cloud);


    pcl::IndicesPtr output_indices(new pcl::Indices);
    pcl::IndicesPtr removed_indices(new pcl::Indices);

    //crophull will be used later to segment a better scene
//    pass_function(raw_cloud,-0.15,0.5,-1,1,-1,1,passed_cloud, false);  // pass through scene

    pass_function(raw_cloud,-0.15,0.5,-1,1,-1,1,output_indices, removed_indices, false);  // pass through scene
//    cout << "output size" << output_indices->size() << endl;
//    cout << "removed size" << removed_indices->size() << endl;

    for(std::size_t i=0; i<removed_indices->size(); i++){
        raw_cloud->points[(*removed_indices)[i]].x = 0;
        raw_cloud->points[(*removed_indices)[i]].y = 0;
        raw_cloud->points[(*removed_indices)[i]].z = 0;
    }

    for (std::size_t i = 0; i < raw_cloud->points.size (); ++i)
    {
        if (!std::isfinite (raw_cloud->points[i].x) ||
            !std::isfinite (raw_cloud->points[i].y) ||
            !std::isfinite (raw_cloud->points[i].z))
        {
            raw_cloud->points[i].x = 0.0;
            raw_cloud->points[i].y = 0.0;
            raw_cloud->points[i].z = 0.0;
        }
    }
//    for (int j = 0; j < output_indices->size(); ++j) {
//        raw_cloud->points[(*output_indices)[j]].r = 255;
//        raw_cloud->points[(*output_indices)[j]].g = 0;
//        raw_cloud->points[(*output_indices)[j]].b = 0;
//    }


//    pcl::copyPointCloud(*raw_cloud,*output_indices,*passed_cloud);

//    pcl::io::savePCDFileASCII ("desk_scene.pcd", *raw_cloud);//保存pcd
//    pcl::io::savePNGFile("test.png",*raw_cloud, "rgb");
    pcl::toROSMsg(*raw_cloud, ros_passed_cloud);
    ros_passed_cloud.header.frame_id = "eye_to_hand_depth_optical_frame";
    passed_cloud_pub.publish(ros_passed_cloud);

}
int main (int argc, char **argv)
{
    ros::init (argc, argv, "eye_to_hand_point_cloud_preprocess");
    ros::NodeHandle node;
    ros::Subscriber cloud_sub = node.subscribe("/eye_to_hand/depth_registered/points", 1, cloudCB);//接收点云
    passed_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/passed_cloud",1);
    ros::Duration(4).sleep(); // wait for camera msg
    ros::spin();
    return 0;
}  
