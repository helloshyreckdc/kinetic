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

bool start_demo = false;
bool recording_bag = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "record_trajectory");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher ref_traj_pub = node.advertise<geometry_msgs::Transform>("ref_traj",10);
    geometry_msgs::Transform ref_traj;

    ROS_INFO("haven't started demo");

    ros::Rate rate(50.0);

    ros::param::set("/start_demo", start_demo);
    while (node.ok()){
        ros::param::get("/start_demo", start_demo);
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base", "tool0",
                                                        ros::Time(0), ros::Duration(2.0)); //wait 2 seconds,else can't find frames
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ref_traj = transformStamped.transform;
        ref_traj_pub.publish(ref_traj);

        if (start_demo && !recording_bag) {
            //gnome-terminal used to open a new terminal, -x denotes execute and the others are the command
            //if we don't use gnome-terminal, rosbag record will block the terminal and other system("")
            //command would not execute
            std::string time = "`date +%Y%m%d_%H_%m_%s`";
            system(("gnome-terminal -x rosbag record -O demo"+time+" /ref_traj __name:=record_bag").c_str());
            recording_bag = true;
        }
        if(!start_demo && recording_bag){
            //this would close rosbag record
            system("gnome-terminal -x rosnode kill record_bag");
            recording_bag = false;
            ROS_INFO("Finish demo");
            break;


        }
        rate.sleep();
    }
    return 0;
}