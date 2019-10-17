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
    ros::init(argc, argv, "pub_sr300_pose");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher sr300_pose_pub = node.advertise<geometry_msgs::Transform>("/sr300_pose",10);
    geometry_msgs::Transform sr300_pose;

    ros::Rate rate(50.0);

    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base", "sr300_color_optical_frame",
                                                        ros::Time(0), ros::Duration(2.0)); //wait 2 seconds,else can't find frames
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        sr300_pose = transformStamped.transform;
        sr300_pose_pub.publish(sr300_pose);

        rate.sleep();
    }
    return 0;
}