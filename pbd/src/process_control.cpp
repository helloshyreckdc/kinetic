//
// Created by shyreckdc on 19-8-10.
// control the process of pbd
//
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "process_control");

    ros::NodeHandle node;

    ros::Rate rate(50.0);

    ros::param::set("/process_control",0);
    ros::param::set("/bag_name","demo50hz.bag");
    string bag_name,previous_bag_name;
    previous_bag_name = "bag";

    int previous_task_id = 0;
    int current_task_id = 0;
    const int start_demo = 1;
    const int end_demo = 2;
    const int replay_origin = 5;
    const int replay = 10;
    const int end_exec = 11;

    while (node.ok()){
        ros::param::get("/bag_name",bag_name);
        if(previous_bag_name != bag_name){

            rosbag::Bag bag;
            bag.open(bag_name,rosbag::bagmode::Read);
            rosbag::View view(bag);
            ros::Duration bag_length_time = view.getEndTime() - view.getBeginTime();
            double bag_length = bag_length_time.toSec();
            ros::param::set("/demo_bag_length",bag_length);
            bag.close();

            previous_bag_name = bag_name;
        }

        ros::param::get("/process_control",current_task_id);
        if(previous_task_id != current_task_id){
            switch(current_task_id){
                case start_demo:
                    system("gnome-terminal -x rosrun pbd record __name:=record");
                    ros::Duration(2).sleep();
                    ros::param::set("/start_demo", true);
                    break;

                case end_demo:
                    ros::param::set("/start_demo", false);
                    break;

                case replay_origin:
//                    system(("rosrun pbd replay_go_to_origin "
//                    + bag_name + " __name:=replay_go_to_origin").c_str());
                    system(("gnome-terminal -x rosrun pbd replay_go_to_origin "
                            + bag_name + " __name:=replay_go_to_origin").c_str());
                    break;

                case replay:
                    system(("gnome-terminal -x rosrun pbd replay_traj "+bag_name+" __name:=replay_traj").c_str());
//                    system(("rosrun pbd replay_traj "+bag_name+" __name:=replay_traj").c_str());
                    break;

                case end_exec:
                    system("gnome-terminal -x rosnode kill /replay_traj");
                    break;

                default:
                    cout << "error command!" << endl;
                    break;
            }

            previous_task_id = current_task_id;
        }

        rate.sleep();
    }
    return 0;
}



