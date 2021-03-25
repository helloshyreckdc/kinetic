//
// Created by shyreckdc on 19-7-31.
// subscribe matlab speed topic and publish to ur

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf_system/Trans_angle_axis.h>
#include <string>
#include <pbd/to_string.h>
using namespace std;

std_msgs::String ur_string;
ros::Publisher exec_ref_traj_pub;
bool robot_in_goal = false;

//template < typename T > std::string to_string( const T& n )
//{
//    std::ostringstream stm ;
//    stm << n ;
//    return stm.str() ;
//}

void sub_poseCB(const geometry_msgs::Twist &msg){

	std::string s = "speedl(["
		+to_string(msg.linear.x)+","
		+to_string(msg.linear.y)+","
		+to_string(msg.linear.z)+","
		+to_string(msg.angular.x)+","
		+to_string(msg.angular.y)+","
		+to_string(msg.angular.z)
		+"],0.5,1)";
	ur_string.data = s;

}


int main(int argc, char** argv){
	ros::init(argc, argv, "exe_matlab_ur_velocity");

	ros::NodeHandle node;

	exec_ref_traj_pub = node.advertise<std_msgs::String>("/ur_driver/URScript",10);
	ros::Subscriber trans_angle_axis_sub = node.subscribe("/ur/velocity",10,sub_poseCB);


	//    ros::param::set("/reset_mat", false); // to solve matlab ros topic cache

	ros::Duration(1).sleep();


	ros::Rate rate(50);
	ros::Time begin;
	begin = ros::Time::now();


	while(node.ok()){
		ros::spinOnce();
		if(!ur_string.data.empty()) {
			exec_ref_traj_pub.publish(ur_string);
			cout << ur_string << endl;
		}
		ros::Duration duration = ros::Time::now()-begin;

		//        ROS_INFO("%f", duration.toSec());


		ros::param::get("/robot_in_goal", robot_in_goal); // to solve matlab ros topic cache
		if(robot_in_goal)
		{
			ur_string.data = "stopl(20)";
			exec_ref_traj_pub.publish(ur_string);
			break;
		}
		rate.sleep();
	}

	ur_string.data = "stopl(20)";
	exec_ref_traj_pub.publish(ur_string);
			cout << ur_string << endl;


	return 0;
}
