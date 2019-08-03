//
// Created by shyreckdc on 19-7-31.
// calculate speed according to current pose and reference pose

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
#include <reference_6d_pose/to_string.h>
using namespace std;

std_msgs::String ur_string;
ros::Publisher exec_ref_traj_pub;

//template < typename T > std::string to_string( const T& n )
//{
//    std::ostringstream stm ;
//    stm << n ;
//    return stm.str() ;
//}
double speed = 0;
double step = 0.01;
void sub_poseCB(const tf_system::Trans_angle_axis &trans_angle_axis){

//    std::string s = "movel(p[" + to_string(trans_angle_axis.translation.x) + ","
//                    + to_string(trans_angle_axis.translation.y) + ","
//                    + to_string(trans_angle_axis.translation.z) + ","
//                    + to_string(trans_angle_axis.angle_axis.x) + ","
//                    + to_string(trans_angle_axis.angle_axis.y) + ","
//                    + to_string(trans_angle_axis.angle_axis.z) + "],0.1,0.1,0,0)";

    if(speed>0.1||speed<-0.1)
        step = -step;

    speed = speed + step;
    std::string s = "speedl([0,0,"+to_string(speed)+",0,0,0],0.5,1)";
    ur_string.data = s;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "replay_trajectory");

    ros::NodeHandle node;

    exec_ref_traj_pub = node.advertise<std_msgs::String>("/ur_driver/URScript",10);
    ros::Subscriber trans_angle_axis_sub = node.subscribe("/trans_angle_axis",10,sub_poseCB);

    double bag_length = 0;
    ros::param::get("/demo_bag_length", bag_length);

    ros::Duration(3).sleep();
    system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());


    ros::Rate rate(50);
    ros::Time begin;
    begin = ros::Time::now();

    while(node.ok()){
        ros::spinOnce();
        exec_ref_traj_pub.publish(ur_string);
        ros::Duration duration = ros::Time::now()-begin;
//        ROS_INFO("%f", duration.toSec());
        if(duration.toSec() > bag_length)
        {
            ur_string.data = "stopl(1)";
            exec_ref_traj_pub.publish(ur_string);
            break;
        }
        rate.sleep();
    }


    system("gnome-terminal -x rosnode kill replay_bag");
    return 0;
}