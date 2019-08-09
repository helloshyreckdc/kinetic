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
#include <pbd/to_string.h>
using namespace std;

std_msgs::String ur_string;
ros::Publisher exec_ref_traj_pub;

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
    ros::init(argc, argv, "replay_trajectory");

    ros::NodeHandle node;

    exec_ref_traj_pub = node.advertise<std_msgs::String>("/ur_driver/URScript",10);
    ros::Subscriber trans_angle_axis_sub = node.subscribe("/ur/velocity",10,sub_poseCB);

    double bag_length = 0;
    ros::param::get("/demo_bag_length", bag_length);

    ros::Duration(3).sleep();
    system(("gnome-terminal -x rosbag play "+ to_string(argv[1]) +" __name:=replay_bag").c_str());


    ros::Rate rate(50);
    ros::Time begin;
    begin = ros::Time::now();

    int count = 0;  //used to delete the first two point in /ur/velocity

    while(node.ok()){
        ros::spinOnce();
        if(count > 5)
        {
            exec_ref_traj_pub.publish(ur_string);
            cout << ur_string << endl;
        }
        ++count;
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