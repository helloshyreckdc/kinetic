//
// Created by shyreckdc on 19-7-31.
// now, this code is used to move the robot to the origin of demonstration.
// In the future, this code would be used to grasp an object and then go to origin(according to object relative frame)

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <tf_system/Trans_angle_axis.h>
#include <string>
#include <reference_6d_pose/to_string.h>
using namespace std;

std_msgs::String ur_string;
ros::Publisher exec_ref_traj_pub;

void sub_poseCB(const tf_system::Trans_angle_axis trans_angle_axis){

    std::string s = "movel(p[" + to_string(trans_angle_axis.translation.x) + ","
                    + to_string(trans_angle_axis.translation.y) + ","
                    + to_string(trans_angle_axis.translation.z) + ","
                    + to_string(trans_angle_axis.angle_axis.x) + ","
                    + to_string(trans_angle_axis.angle_axis.y) + ","
                    + to_string(trans_angle_axis.angle_axis.z) + "],0.1,0.1,0,0)";

    ur_string.data = s;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "replay_go_to_origin");

    ros::NodeHandle node;

    exec_ref_traj_pub = node.advertise<std_msgs::String>("/ur_driver/URScript",10);
    ros::Subscriber trans_angle_axis_sub = node.subscribe("/trans_angle_axis",10,sub_poseCB);

    ros::Duration(2).sleep();
    system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());


    ros::Rate rate(50);

    while(node.ok()){
        ros::spinOnce();
        if(ur_string.data != ""){
            exec_ref_traj_pub.publish(ur_string);
            break;
        }
        rate.sleep();
    }


    system("gnome-terminal -x rosnode kill replay_bag");
    return 0;
};