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
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


using namespace std;

typedef pcl::PointXYZ PointT;

//void cloudCB(const sensor_msgs::PointCloud2 &input)
pcl::StatisticalOutlierRemoval<PointT> sor;
sensor_msgs::PointCloud2 ros_passed_cloud;
ros::Publisher passed_cloud_pub;
tf::TransformListener* tf_listener_ptr;

void pass_function(pcl::PointCloud<PointT>::Ptr input_cloud, double start_x, double end_x,
                   double start_y, double end_y, double start_z, double end_z,
                   pcl::PointCloud<PointT>::Ptr output_cloud,bool set_negative)
{
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_cloud);
}

void pass_function(pcl::PointCloud<PointT>::Ptr input_cloud, double start_x, double end_x,
                   double start_y, double end_y, double start_z, double end_z,
                   pcl::IndicesPtr output_indices,pcl::IndicesPtr removed_indices, bool set_negative)
{
    pcl::CropBox<PointT> boxFilter(true);   // set to true to get removed indices
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_indices);
    pcl::IndicesConstPtr removed_temp = boxFilter.getRemovedIndices(); // to solve const pointer and non const
    *removed_indices = *removed_temp;
}

void pass_function(pcl::PointCloud<PointT>::Ptr input_cloud, double start_x, double end_x,
                   double start_y, double end_y, double start_z, double end_z,
                   pcl::IndicesPtr output_indices,pcl::IndicesPtr removed_indices, bool set_negative, Eigen::Affine3d transform)
{
    pcl::CropBox<PointT> boxFilter(true);   // set to true to get removed indices
    boxFilter.setMin(Eigen::Vector4f(start_x, start_y, start_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(end_x, end_y, end_z, 1.0));
    transform = transform.inverse();
    // transform pointcloud to gripper frame and then filter with a box
    boxFilter.setTransform((Eigen::Affine3f)transform);
//    cout << transform.matrix() << endl;
    boxFilter.setInputCloud(input_cloud);
    boxFilter.setNegative(set_negative);
    boxFilter.filter(*output_indices);
    pcl::IndicesConstPtr removed_temp = boxFilter.getRemovedIndices(); // to solve const pointer and non const
    *removed_indices = *removed_temp;
}


void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)
{
    bool check_obs;
    ros::param::get("/check_obs", check_obs);
    if(check_obs)
    {
        pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud <PointT>);
        pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud <PointT>);
        pcl::PointCloud<PointT>::Ptr passed_cloud(new pcl::PointCloud <PointT>);
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud <PointT>);
        pcl::fromPCLPointCloud2(*input, *raw_cloud);

        for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
            if (!std::isfinite(raw_cloud->points[i].x) ||
                !std::isfinite(raw_cloud->points[i].y) ||
                !std::isfinite(raw_cloud->points[i].z)) {
                raw_cloud->points[i].x = 0.0;
                raw_cloud->points[i].y = 0.0;
                raw_cloud->points[i].z = 0.0;
            }
        }


        pcl::IndicesPtr output_indices(new pcl::Indices);
        pcl::IndicesPtr removed_indices(new pcl::Indices);

        //    tf::TransformListener listener;
        tf::StampedTransform transform;
        //    ros::Duration(1.0).sleep();
        try{
            tf_listener_ptr->lookupTransform("base_link", "eye_to_hand_depth_optical_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }

        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);

        // Executing the transformation
        pcl::transformPointCloud (*raw_cloud, *transformed_cloud, eigen_transform);

//        tf::Vector3 translation = transform.getOrigin();
//        double gripper_center_x = translation.getX();
//        double gripper_center_y = translation.getY();
//        double gripper_center_z = translation.getZ();
//        cout << "x: " << translation.getX() << endl;
//        cout << "y: " << translation.getY() << endl;
//        cout << "z: " << translation.getZ() << endl;

        //crophull will be used later to segment a better scene
        pass_function(transformed_cloud, -0.15, 0.5, -1, 1, -1, 1, passed_cloud, false);  // pass through scene
//    pass_function(raw_cloud,-0.15,0.5,-1,1,-1,1,output_indices, removed_indices, false);  // pass through scene


//            /******  Statistical Removal  ******/
        pcl::StatisticalOutlierRemoval<PointT> sor_rm_outlier;
        sor_rm_outlier.setInputCloud(passed_cloud);
        sor_rm_outlier.setMeanK(30);
        sor_rm_outlier.setStddevMulThresh(1);
        sor_rm_outlier.filter(*filtered_cloud);


        pcl::PointXYZ max;//用于存放三个轴的最大值
        pcl::PointXYZ min;//用于存放三个轴的最小值
        pcl::getMinMax3D(*filtered_cloud, min, max);

        ros::param::set("/obs_max_z", max.z);  // param for check obstacle

//    pcl::copyPointCloud(*raw_cloud,*output_indices,*passed_cloud);

//    pcl::io::savePCDFileASCII ("desk_scene.pcd", *raw_cloud);//保存pcd
//    pcl::io::savePNGFile("test.png",*raw_cloud, "rgb");
        pcl::toROSMsg(*filtered_cloud, ros_passed_cloud);
        ros_passed_cloud.header.frame_id = "base_link";
        passed_cloud_pub.publish(ros_passed_cloud);


        ros::param::set("/check_obs", false);  // param for check obstacle

    }

}
int main (int argc, char **argv)
{
    ros::init(argc, argv, "check_obstacle");
    ros::NodeHandle node;
    ros::param::set("/check_obs", false);  // param for check obstacle
    tf::TransformListener listener;
    tf_listener_ptr = &listener;
    ros::Subscriber cloud_sub = node.subscribe("/eye_to_hand/depth_registered/points", 1, cloudCB);//接收点云,需要修改话题
    passed_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/passed_cloud",1);
    ros::Duration(1).sleep(); // wait for camera msg
    ros::spin();
    return 0;
}  
