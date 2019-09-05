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
#include <tf/transform_broadcaster.h>


tf::Vector3 translation;
tf::Quaternion rotation;
bool pub_frame = false;

void object_frame_ref_trajCB(const geometry_msgs::Transform msg){
    translation.setValue(msg.translation.x, msg.translation.y, msg.translation.z);
    rotation.setValue(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
    pub_frame = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_tool0_target_frame");

    ros::NodeHandle node;

    ros::Subscriber scene_sub = node.subscribe("/object_frame_ref_traj",10,object_frame_ref_trajCB);
    geometry_msgs::Transform current_pose;

    ros::Rate rate(50.0);

    ros::Duration(10).sleep();

    while (node.ok()){

        ros::spinOnce();

        if(pub_frame){
            pub_frame = false;
            /******  transform broadcaster  ******/
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(translation);
            transform.setRotation(rotation);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "assembly_base", "tool0_target"));
        }

        rate.sleep();
    }
    return 0;
}