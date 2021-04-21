#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>

//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
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

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


using namespace std;
typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

tf::TransformListener* tf_listener_ptr;

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPoint_Exposed : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
    pcl::CorrespondencesPtr getCorrespondencesPtr() {
        for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
            pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
            std::cout << "Index of the source point: " << currentCorrespondence.index_query << std::endl;
            std::cout << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
            std::cout << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
            std::cout << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight << std::endl;
        }
        return this->correspondences_;
    }
};

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
PointCloudT::Ptr cloud_outlier_removed (new PointCloudT);
PointCloudT::Ptr cloud_passed(new PointCloudT);
PointCloudT::Ptr cloud_model_aligned(new PointCloudT);
PointCloudT::Ptr final_cloud(new PointCloudT);

pcl::PCLPointCloud2::Ptr cloud2_scene(new pcl::PCLPointCloud2);  //scene imported
pcl::PCLPointCloud2::Ptr cloud2_down_sampled(new pcl::PCLPointCloud2);

void cloudCB(const pcl::PCLPointCloud2ConstPtr& input)
{
    *cloud2_scene = *input;
//    pcl::PointCloud<pcl::PointXYZRGB> cloud;
//    pcl::fromPCLPointCloud2(*input, cloud);
//    pcl::io::savePCDFileASCII ("passed.pcd", cloud);//保存pcd
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cylinder_frame");
    ros::NodeHandle node;

    tf::TransformListener listener;
    tf_listener_ptr = &listener;

    ros::Subscriber scene_sub = node.subscribe("/passed_cloud",10,cloudCB);

    ros::Rate loop_rate(50);

    ros::Duration(3).sleep(); // wait for camera msg

    ros::param::set("/recompute_cylinder_frame", true);
    bool recompute_cylinder_frame;

//    Eigen::Matrix4d trans_inverse;
    Eigen::Matrix4d eigen_camera2object;


    /******  transform broadcaster  ******/
    static tf::TransformBroadcaster br;
    tf::Transform camera2object;
    tf::StampedTransform gripper2camera;


    while(ros::ok())
    {
        ros::param::get("/recompute_cylinder_frame", recompute_cylinder_frame);
        if(recompute_cylinder_frame) {
            ros::param::set("/recompute_cylinder_frame", false);
            ros::spinOnce();

            //Load the template
            if (pcl::io::loadPCDFile("/home/shyreckdc/catkin_ws/src/peg_in_hole/resources/cylinder.pcd",*cloud_model) < 0) {
//            if (pcl::io::loadPCDFile(argv[1], *cloud_model) < 0) {
                ROS_INFO("Error loading cloud");
                return (-1);
            }


            pcl::console::TicToc time;
            time.tic();
            pcl::fromPCLPointCloud2(*cloud2_scene, *cloud_down_sampled);

            pcl::IndicesPtr passed_inliers (new pcl::Indices);
            pcl::IndicesPtr inliers_outlier_removed (new pcl::Indices);

            pcl::PassThrough<PointT> pass(true);
            pass.setInputCloud (cloud_down_sampled);
//            pass.setIndices(inliers_without_plane);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (-0.001, 0.001);
            pass.setNegative(true);
            pass.filter(*passed_inliers);
            pcl::copyPointCloud(*cloud_down_sampled,*passed_inliers,*cloud_passed);
            pcl::io::savePCDFileASCII ("/home/shyreckdc/catkin_ws/src/peg_in_hole/resources/cloud_passed.pcd", *cloud_passed);//保存pcd

//            /******  Statistical Removal  ******/
            pcl::StatisticalOutlierRemoval<PointT> sor_rm_outlier;
            sor_rm_outlier.setInputCloud(cloud_down_sampled);
            sor_rm_outlier.setIndices(passed_inliers);
            sor_rm_outlier.setMeanK(30);
            sor_rm_outlier.setStddevMulThresh(1);
            sor_rm_outlier.filter(*inliers_outlier_removed);
            pcl::copyPointCloud(*cloud_down_sampled,*inliers_outlier_removed,*cloud_outlier_removed);
            pcl::io::savePCDFileASCII ("/home/shyreckdc/catkin_ws/src/peg_in_hole/resources/final_cloud.pcd", *cloud_outlier_removed);//保存pcd

            /******  ICP registration  ******/
            float sum_x = 0;
            float sum_y = 0;
            float sum_z = 0;
            float trans_x = 0;
            float trans_y = 0;
            float trans_z = 0;

            for (size_t i = 0; i < cloud_outlier_removed->points.size(); ++i) {
                sum_x += cloud_outlier_removed->points[i].x;
                sum_y += cloud_outlier_removed->points[i].y;
                sum_z += cloud_outlier_removed->points[i].z;
            }
            trans_x = sum_x / cloud_outlier_removed->points.size();
            trans_y = sum_y / cloud_outlier_removed->points.size();
            trans_z = sum_z / cloud_outlier_removed->points.size();

            // Set initial alignment estimate found using robot odometry.
            Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
            Eigen::Translation3f init_translation(trans_x, trans_y, trans_z);
            Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

            Eigen::Matrix4d eigen_camera2object = Eigen::Matrix4d::Identity();
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setMaxCorrespondenceDistance(0.01);
            icp.setMaximumIterations(200);
            icp.setInputSource(cloud_model);
//            icp.setIndices(inliers_without_plane);
            icp.setInputTarget(cloud_outlier_removed);
            icp.align(*final_cloud, init_guess);

            ROS_INFO("Finished the pose estimation in %f ms", time.toc());

            if (icp.hasConverged()) {
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
                std::cout << "\nICP transformation 100 : cloud_model -> cloud_cube" << std::endl;
                eigen_camera2object = icp.getFinalTransformation().cast<double>();
                tf::transformEigenToTF((Eigen::Affine3d)eigen_camera2object, camera2object);

            } else {
                ROS_INFO("\nICP has not converged!\n");
                return -2;
            }
        }

       // tf::Matrix3x3 transm_r;
       // transm_r.setValue(eigen_camera2object(0, 0), eigen_camera2object(0, 1), eigen_camera2object(0, 2),
       //                   eigen_camera2object(1, 0), eigen_camera2object(1, 1), eigen_camera2object(1, 2),
       //                   eigen_camera2object(2, 0), eigen_camera2object(2, 1), eigen_camera2object(2, 2));
       // tf::Quaternion q;
       // transm_r.getRotation(q);
       // transform.setOrigin(tf::Vector3(eigen_camera2object(0, 3), eigen_camera2object(1, 3), eigen_camera2object(2, 3)));
       // transform.setRotation(q);

        try{
            tf_listener_ptr->lookupTransform("gripper_center", "eye_to_hand_depth_optical_frame", ros::Time(0), gripper2camera);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }

        tf::Transform gripper2object = gripper2camera*camera2object;



        br.sendTransform(tf::StampedTransform(gripper2object, ros::Time::now(), "gripper_center", "cylinder"));

        loop_rate.sleep();
    }

    return(0);
}
