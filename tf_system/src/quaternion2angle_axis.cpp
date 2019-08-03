#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_system/Trans_angle_axis.h>
#include <geometry_msgs/Transform.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
ros::Publisher trans_angle_axis_pub;
tf_system::Trans_angle_axis trans_angle_axis;
/*Ros->Urscript:四元数（x,y,z,w)转旋转矢量（x,y,z)*/
vector<double> quaternion2angle_axis(const double i,const double j, const double k, const double r)//输入时w,x,y,z,内部存储x,y,z,w
{
	Eigen::Quaterniond q(r,i,j,k);
	Eigen::AngleAxisd rv;
	rv=q;
	vector<double> rotation_vector;
	Eigen::Vector3d rotation_vector3;
	rotation_vector3=rv.axis()*rv.angle();
	rotation_vector.push_back(rotation_vector3(0));
	rotation_vector.push_back(rotation_vector3(1));
	rotation_vector.push_back(rotation_vector3(2));	
if(rotation_vector.at(0)<0.0)
{
	Eigen::Quaterniond q(-r,-i,-j,-k);
	rotation_vector.clear();
	rv=q;
	rotation_vector3=rv.axis()*rv.angle();
	rotation_vector.push_back(rotation_vector3(0));
	rotation_vector.push_back(rotation_vector3(1));
	rotation_vector.push_back(rotation_vector3(2));	
}
	return rotation_vector;
}

/*Eigen::Vector3d quaternion2angle_axis(Eigen::Quaterniond q)//直接输入Eigen:Quaternioind形式
{
	Eigen::AngleAxisd rv;
	rv=q;
	Eigen::Vector3d rotation_vector;
	rotation_vector=rv.axis()*rv.angle();
	return rotation_vector;
	/*std::string s;
	  s = "x:"+rotation_vector.x
*/



void quat2angle_axisCallback(const geometry_msgs::Transform transf)
{

	vector<double> angle_axis = quaternion2angle_axis(transf.rotation.x,transf.rotation.y,transf.rotation.z,transf.rotation.w);
	trans_angle_axis.angle_axis.x = angle_axis.at(0);
	trans_angle_axis.angle_axis.y = angle_axis.at(1);
	trans_angle_axis.angle_axis.z = angle_axis.at(2);
    trans_angle_axis.translation = transf.translation;
    trans_angle_axis.header.stamp = ros::Time::now();
	trans_angle_axis_pub.publish(trans_angle_axis);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"quaternion2angle_axis");
	ros::NodeHandle node;
	trans_angle_axis_pub = node.advertise<tf_system::Trans_angle_axis>("trans_angle_axis",10);
	ros::Subscriber sub = node.subscribe("ref_traj",10,quat2angle_axisCallback);
	ros::spin();

	return 0;

}





//Eigen::Quaterniond q(-0.120,0.796,-0.534, -0.260);
//  Eigen::Vector3d aa = quaternion2angle_axis(-0.796,0.534, 0.260, 0.120);
//  cout<<aa<<endl;
//  cout<<quaternion2angle_axis(q)<<endl;
//
//  return 0;
//
//  }
