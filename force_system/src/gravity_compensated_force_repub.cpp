//
// Created by shyreckdc on 19-7-31.
// now, this code is used to move the robot to the origin of demonstration.
// In the future, this code would be used to grasp an object and then go to origin(according to object relative frame)

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace Eigen;
using namespace std;
VectorXd wrench_vec(6);
VectorXd sensor_bias(6);
MatrixXd rotation_mat(3,3);

void ati_poseCB(const geometry_msgs::Transform msg)
{
    Eigen::Quaterniond q(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
    rotation_mat = q.toRotationMatrix();
}

void sub_forceCB(const geometry_msgs::WrenchStamped msg){
    wrench_vec(0) = msg.wrench.force.x;
    wrench_vec(1) = msg.wrench.force.y;
    wrench_vec(2) = msg.wrench.force.z;
    wrench_vec(3) = msg.wrench.torque.x;
    wrench_vec(4) = msg.wrench.torque.y;
    wrench_vec(5) = msg.wrench.torque.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gravity_compensated_force_repub");

    ros::NodeHandle node;

    ros::Publisher gravity_compensated_force_pub =
            node.advertise<geometry_msgs::WrenchStamped>("/gravity_compensated_force",10);
    ros::Subscriber force_sub = node.subscribe("/averaged_calibrated_force",10,sub_forceCB);
    ros::Subscriber sub = node.subscribe("/ati_current_pose",10,ati_poseCB);

    geometry_msgs::WrenchStamped gravity_compensated_wrench;

    ros::param::set("/recompensate_gravity_and_sensor_bias",false);

    Vector3d g_world(0,0,-9.794); // gravity in world
    MatrixXd mass_matrix;
    mass_matrix.setZero(6,3);
    MatrixXd Rworld2base(3,3);
    Rworld2base.setIdentity();
    VectorXd compensated_wrench_vec(6);
    bool recompensate = false;

    std::string fin = "/home/shyreckdc/catkin_ws/src/force_system/config/gravity_bias.yaml";
    YAML::Node yamlConfig = YAML::LoadFile(fin);

    vector<double> file_bias_array = yamlConfig["sensor_bias"].as<vector<double> >();
    for(int i=0;i<6;i++){
        sensor_bias(i) = file_bias_array[i];
    }

    vector<double> file_mass_matrix_array = yamlConfig["mass_matrix"].as<vector<double> >();
    for(int column=0;column<3;column++) {
        for (int row = 0; row < 6; row++) {
            mass_matrix(row, column) = file_mass_matrix_array[column*6+row];
        }
    }

    vector<double> file_world2base_array = yamlConfig["world2base"].as<vector<double> >();
    for(int column=0;column<3;column++) {
        for (int row = 0; row < 3; row++) {
            Rworld2base(row, column) = file_world2base_array[column*3+row];
        }
    }

    ros::Duration(2).sleep();

    ros::Rate rate(50);



    while(node.ok()){

        ros::spinOnce();

        ros::param::get("/recompensate_gravity_and_sensor_bias",recompensate);
        if(recompensate){

            std::vector<double> bias;
            ros::param::get("/sensor_bias",bias);
            for(int i=0;i<6;i++){
                sensor_bias(i) = bias[i];
            }

            std::vector<double> mass_matrix_array;
            ros::param::get("/mass_matrix",mass_matrix_array);
            for(int column=0;column<3;column++) {
                for (int row = 0; row < 6; row++) {
                    mass_matrix(row, column) = mass_matrix_array[column*6+row];
                }
            }

            std::vector<double> world2base_array;
            ros::param::get("/Rworld2base",world2base_array);
            for(int column=0;column<3;column++) {
                for (int row = 0; row < 3; row++) {
                    Rworld2base(row, column) = world2base_array[column*3+row];
                }
            }

            ros::param::set("/recompensate_gravity_and_sensor_bias",false);

            ofstream fout("/home/shyreckdc/catkin_ws/src/force_system/config/gravity_bias.yaml");
            YAML::Emitter out(fout);
            out << YAML::BeginMap;
            out << YAML::Key << "sensor_bias";
            out << YAML::Value << YAML::Flow << bias;
            out << YAML::Key << "mass_matrix";
            out << YAML::Value << YAML::Flow << mass_matrix_array;
            out << YAML::Key << "world2base";
            out << YAML::Value << YAML::Flow << world2base_array;
            out << YAML::EndMap;

        }

        compensated_wrench_vec = wrench_vec - sensor_bias - mass_matrix*(Rworld2base*rotation_mat).transpose()*g_world;
//		cout << "wrench_vec: " << wrench_vec << endl;
//		cout << "sensor_bias: " << sensor_bias << endl;
//		cout << "mass_matrix: " << mass_matrix << endl;
//		cout << "Rworld2base: " << Rworld2base << endl;
//		cout << "rotation_mat" << rotation_mat << endl;
//
//		cout << "compensated_wrench" << compensated_wrench_vec << endl;

        gravity_compensated_wrench.wrench.force.x = compensated_wrench_vec(0);
        gravity_compensated_wrench.wrench.force.y = compensated_wrench_vec(1);
        gravity_compensated_wrench.wrench.force.z = compensated_wrench_vec(2);
        gravity_compensated_wrench.wrench.torque.x = compensated_wrench_vec(3);
        gravity_compensated_wrench.wrench.torque.y = compensated_wrench_vec(4);
        gravity_compensated_wrench.wrench.torque.z = compensated_wrench_vec(5);

        gravity_compensated_force_pub.publish(gravity_compensated_wrench);

        rate.sleep();
    }

    return 0;
}
