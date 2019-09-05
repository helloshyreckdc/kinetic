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


int main(int argc, char** argv){
    ros::init(argc, argv, "pub_ref_traj");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher ref_traj_pub = node.advertise<geometry_msgs::Transform>("/ref_traj",10);
    geometry_msgs::Transform ref_traj;

    ros::Duration(3).sleep();

    ros::Rate rate(50.0);

    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base", "tool0_target",
                                                        ros::Time(0), ros::Duration(2.0)); //wait 2 seconds,else can't find frames
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ref_traj = transformStamped.transform;
        ref_traj_pub.publish(ref_traj);

        rate.sleep();
    }
    return 0;
}