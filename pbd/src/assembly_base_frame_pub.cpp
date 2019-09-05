#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>

//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>  //TicToc
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

PointCloudT::Ptr cloud_model(new PointCloudT);  //template imported
PointCloudT::Ptr cloud_down_sampled(new PointCloudT);
PointCloudT::Ptr cloud_pass_through_x(new PointCloudT);
PointCloudT::Ptr cloud_pass_through_y(new PointCloudT);
PointCloudT::Ptr cloud_segmented(new PointCloudT);
PointCloudT::Ptr cloud_outlier_removed (new PointCloudT);

pcl::PCLPointCloud2::Ptr cloud2_scene(new pcl::PCLPointCloud2);  //scene imported
pcl::PCLPointCloud2::Ptr cloud2_down_sampled(new pcl::PCLPointCloud2);

void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)
{
    *cloud2_scene = *input;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "assembly_base_frame_pub");
    ros::NodeHandle node;

    ros::Subscriber scene_sub = node.subscribe("/passed_cloud",10,cloudCB);

    ros::Rate loop_rate(50);

    ros::Duration(7).sleep(); // wait for camera msg

    ros::param::set("/recompute_assembly_base_frame", true);
    bool recompute_assembly_base_frame;

    Eigen::Matrix4d trans_inverse;




    while(ros::ok())
    {
        ros::param::get("/recompute_assembly_base_frame", recompute_assembly_base_frame);
        if(recompute_assembly_base_frame) {
            ros::param::set("/recompute_assembly_base_frame", false);
            ros::spinOnce();

            //Load the template
            if (pcl::io::loadPCDFile("/home/shyreckdc/catkin_ws/src/pbd/resources/assembly_base_model.pcd",*cloud_model) < 0) {
//            if (pcl::io::loadPCDFile(argv[1], *cloud_model) < 0) {
                ROS_INFO("Error loading cloud %s. \n", argv[1]);
                return (-1);
            }

            //Load the scene
//	if(pcl::io::loadPCDFile(argv[2], *cloud2_scene) < 0)
//	{
//		ROS_INFO("Error loading cloud %s. \n", argv[2]);
//		return(-1);
//	}

            pcl::console::TicToc time;
            time.tic();
            /******  Downsampling using a leaf size of 2mm  ******/
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor_downsample;
            sor_downsample.setInputCloud(cloud2_scene);
            sor_downsample.setLeafSize(0.002f, 0.002f, 0.002f);
            sor_downsample.filter(*cloud2_down_sampled);

            pcl::fromPCLPointCloud2(*cloud2_down_sampled, *cloud_down_sampled);
            ROS_INFO("PointCloud after downsampling: %d data points. ",
                     cloud_down_sampled->width * cloud_down_sampled->height);

            /******  Plane segmentation, extract the cube  ******/
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            //Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.01);
            seg.setInputCloud(cloud_down_sampled);
            seg.segment(*inliers, *coefficients);
            //Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_down_sampled);
            extract.setIndices(inliers);
            extract.setNegative(true);  //true corresponds to the target, false corresponds to the plane
            extract.filter(*cloud_segmented);
            ROS_INFO("PointCloud representing the cube segmentation: %d data points. ",
                     cloud_segmented->width * cloud_segmented->height);

            /******  Statistical Removal  ******/
            pcl::StatisticalOutlierRemoval<PointT> sor_rm_outlier;
            sor_rm_outlier.setInputCloud(cloud_segmented);
            sor_rm_outlier.setMeanK(30);
            sor_rm_outlier.setStddevMulThresh(1);
            sor_rm_outlier.filter(*cloud_outlier_removed);
            ROS_INFO("PointCloud after removing outliers: %d data points. ",
                     cloud_outlier_removed->width * cloud_outlier_removed->height);
            pcl::PCDWriter writer;
            writer.write<PointT>("final_cube.pcd", *cloud_outlier_removed, false);

            /******  ICP registration  ******/
            float sum_x = 0;
            float sum_y = 0;
            float sum_z = 0;
            float trans_x = 0;
            float trans_y = 0;
            float trans_z = 0;

            for (size_t i = 0; i < cloud_outlier_removed->points.size(); ++i) {
                sum_x = cloud_outlier_removed->points[i].x + sum_x;
                sum_y = cloud_outlier_removed->points[i].y + sum_y;
                sum_z = cloud_outlier_removed->points[i].z + sum_z;
            }
            trans_x = sum_x / cloud_outlier_removed->points.size();
            trans_y = sum_y / cloud_outlier_removed->points.size();
            trans_z = sum_z / cloud_outlier_removed->points.size();

            // Set initial alignment estimate found using robot odometry.
            Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
            Eigen::Translation3f init_translation(-trans_x, -trans_y, -trans_z);
            Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setMaxCorrespondenceDistance(0.01);
            icp.setMaximumIterations(200);
            icp.setInputSource(cloud_outlier_removed);
            icp.setInputTarget(cloud_model);
            icp.align(*cloud_outlier_removed, init_guess);
            ROS_INFO("Finished the pose estimation in %f ms", time.toc());

            //  Eigen::Matrix3d trans_inter;
            //  Eigen::Matrix3d rot_inverse;

            if (icp.hasConverged()) {
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
                std::cout << "\nICP transformation 100 : cloud_model -> cloud_cube" << std::endl;
                transformation_matrix = icp.getFinalTransformation().cast<double>();

                //    trans_inter << transformation_matrix(0, 0), transformation_matrix(0, 1), transformation_matrix(0, 2),
                //                   transformation_matrix(1, 0), transformation_matrix(1, 1), transformation_matrix(1, 2),
                //                   transformation_matrix(2, 0), transformation_matrix(2, 1), transformation_matrix(2, 2);
                //    rot_inverse = trans_inter.inverse();
                trans_inverse = transformation_matrix.inverse();
                //    trans_inverse << rot_inverse(0, 0), rot_inverse(0, 1), rot_inverse(0, 2), -transformation_matrix(0, 3),
                //                     rot_inverse(1, 0), rot_inverse(1, 1), rot_inverse(1, 2), -transformation_matrix(1, 3),
                //                     rot_inverse(2, 0), rot_inverse(2, 1), rot_inverse(2, 2), -transformation_matrix(2, 3),
                //                     0                , 0                , 0                , 1                           ;

                print4x4Matrix(trans_inverse);
            } else {
                ROS_INFO("\nICP has not converged!\n");
            }
        }
        /******  transform broadcaster  ******/
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Matrix3x3 transm_r;
        transm_r.setValue(trans_inverse(0, 0), trans_inverse(0, 1), trans_inverse(0, 2),
                          trans_inverse(1, 0), trans_inverse(1, 1), trans_inverse(1, 2),
                          trans_inverse(2, 0), trans_inverse(2, 1), trans_inverse(2, 2));
        tf::Quaternion q;
        transm_r.getRotation(q);
        transform.setOrigin(tf::Vector3(trans_inverse(0, 3), trans_inverse(1, 3), trans_inverse(2, 3)));
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "assembly_base"));

        loop_rate.sleep();
    }

    return(0);
}
