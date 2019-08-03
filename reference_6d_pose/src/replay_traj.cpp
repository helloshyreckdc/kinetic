//
// Created by shyreckdc on 19-7-31.
//

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
bool start_demo = false;
bool recording_bag = false;
bool move_to_origin = false;

std_msgs::String ur_string;
ros::Publisher exec_ref_traj_pub;
ros::Time previous_bag_time;
ros::Time current_bag_time;

//template < typename T > std::string to_string( const T& n )
//{
//    std::ostringstream stm ;
//    stm << n ;
//    return stm.str() ;
//}
double speed = 0;
double step = 0.01;
void sub_poseCB(const tf_system::Trans_angle_axis trans_angle_axis){

//    std::string s = "movel(p[" + to_string(trans_angle_axis.translation.x) + ","
//                    + to_string(trans_angle_axis.translation.y) + ","
//                    + to_string(trans_angle_axis.translation.z) + ","
//                    + to_string(trans_angle_axis.angle_axis.x) + ","
//                    + to_string(trans_angle_axis.angle_axis.y) + ","
//                    + to_string(trans_angle_axis.angle_axis.z) + "],0.1,0.1,0,0)";

    if(speed>0.1||speed<-0.1)
        step = -step;

    speed = speed + step;
    std::string s = "speedl(["+to_string(speed)+",0,"+to_string(speed)+",0,0,0],0.5,1)";
    ur_string.data = s;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "replay_trajectory");

    ros::NodeHandle node;

    exec_ref_traj_pub = node.advertise<std_msgs::String>("/ur_driver/URScript",10);
    ros::Subscriber trans_angle_axis_sub = node.subscribe("/trans_angle_axis",10,sub_poseCB);

    double package_length = atof(argv[2]);

    ros::Duration(3).sleep();
    system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());


    ros::Rate rate(50);
    ros::Time begin;
    begin = ros::Time::now();

    while(node.ok()){
        ros::spinOnce();
        exec_ref_traj_pub.publish(ur_string);
        if(move_to_origin) {
            system("gnome-terminal -x rosnode kill replay_bag");
            ros::Duration(6.0).sleep();
            move_to_origin = false;
            system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());
            begin = ros::Time::now();
//            system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());
        }
        ros::Duration duration = ros::Time::now()-begin;
        ROS_INFO("%f", duration.toSec());
        if(duration.toSec() > package_length)
            break;
        rate.sleep();
    }


//    ROS_INFO("haven't started demo");
//
//    ros::Rate rate(50.0);
//
//    ros::param::set("/start_demo", start_demo);
//    while (node.ok()){
//        ros::param::get("/start_demo", start_demo);
//        geometry_msgs::TransformStamped transformStamped;
//        try {
//            transformStamped = tfBuffer.lookupTransform("base", "tool0",
//                                                        ros::Time(0), ros::Duration(2.0)); //wait 2 seconds,else can't find frames
//        }
//        catch (tf2::TransformException &ex) {
//            ROS_WARN("%s", ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
//
//        ref_traj = transformStamped.transform;
//        ref_traj_pub.publish(ref_traj);
//
//        if (start_demo && !recording_bag) {
//            //gnome-terminal used to open a new terminal, -x denotes execute and the others are the command
//            //if we don't use gnome-terminal, rosbag record will block the terminal and other system("")
//            //command would not execute
//            std::string time = "`date +%Y%m%d_%H_%m`";
//            system(("gnome-terminal -x rosbag record -O demo"+time+" /ref_traj __name:=my_bag").c_str());
//            recording_bag = true;
//        }
//        if(!start_demo && recording_bag){
//            //this would close rosbag record
//            system("gnome-terminal -x rosnode kill my_bag");
//            recording_bag = false;
//            ROS_INFO("Finish demo");
//            break;
//
//
//        }
//        rate.sleep();
//    }
    system("gnome-terminal -x rosnode kill replay_bag");
    return 0;
}