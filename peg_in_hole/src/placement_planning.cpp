#include "ros/ros.h"

#include "geometry_msgs/Point32.h"
#include "std_msgs/String.h"

#include <stdlib.h>

#include <sstream>

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "libs/JAKAZuRobot.h"
#include "libs/jakaAPI.h"
#include "libs/jkerr.h"
#include "libs/jktypes.h"

const double PI = 3.1415926;

using namespace std;
using namespace Eigen;

// global variables
geometry_msgs::Point32 previous_point;

bool start_demo = false;
bool start_exe = false;


double x_bias; // bias from specified point
double y_bias; // bias from specified point
double z_bias; // bias between obs and robot ee_link point

Affine3d T_start_pose, T_via_point1, T_via_point2, T_end_pose;
Matrix3d R_start, R_compensate, R_end;
AngleAxisd angle_axis;
double angle;

vector<geometry_msgs::Point32> traj_points;

JAKAZuRobot robot;

void print4x4Matrix(const Affine3d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void print3x3Matrix(const Matrix3d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
}

// compute distance between points
double point_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
    return sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2) + pow((p1.z-p2.z),2));
}

// subscribe center point of object
void center_pointCB(geometry_msgs::Point32 point)
{
    ros::param::get("/start_demo", start_demo);
    if(point_distance(point, previous_point) > 0.002 && start_demo==true)
    {
        traj_points.push_back(point);
        previous_point = point;
    }

}

// convert 4 by 4 matrix to CartesianPose
CartesianPose T_to_cartesianPose(Affine3d T)
{
    CartesianPose pose;
    pose.tran.x = T(0,3);
    pose.tran.y = T(1,3);
    pose.tran.z = T(2,3);

    //        Rpy* rpy;
    //        RotMatrix* rot;
    //        rot->x.x = T(0,0);
    //        rot->x.y = T(1,0);
    //        rot->x.y = T(2,0);
    //        rot->y.x = T(0,1);
    //        rot->y.y = T(1,1);
    //        rot->y.z = T(2,1);
    //        rot->z.x = T(0,2);
    //        rot->z.y = T(1,2);
    //        rot->z.z = T(2,2);
    //
    //        robot.rpy_to_rot_matrix(rpy, rot);
    //        pose.rpy = *rpy;

    pose.rpy.rx = PI/2;
    pose.rpy.ry = 0;
    pose.rpy.rz = 0;

    return pose;

}

// joint move with pose
void sendJointMovePoint(CartesianPose pose)
{
    JointValue ref_joint_pose;
    JointValue joint_pose;
    robot.get_joint_position(&ref_joint_pose);
    robot.kine_inverse(&ref_joint_pose, &pose, &joint_pose);
    robot.joint_move(&joint_pose, MoveMode::ABS, true, 0.5);
}

// compensate joint pose is usually for adding the rotatation of the joint 6
void sendJointMovePoint(CartesianPose pose, double compensate_joint_6)
{
    JointValue ref_joint_pose;
    JointValue joint_pose;
    robot.get_joint_position(&ref_joint_pose);
    robot.kine_inverse(&ref_joint_pose, &pose, &joint_pose);
    joint_pose.jVal[5] += compensate_joint_6;
    robot.joint_move(&joint_pose, MoveMode::ABS, true, 0.5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj");

    ros::NodeHandle n;

    //        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//        ros::Subscriber point_sub = n.subscribe("/base_centerxyz", 1, center_pointCB);


    // init global variables
    ros::param::set("/place_seq", 0);
    int pre_place_seq = 0;
    int place_seq = 0;


    double via_x = 1; % via_point
    double via_y = 1;
    double via_z = 1;




    ros::Rate loop_rate(5);

    while (ros::ok())
    {

        ros::param::get("/place_seq", place_seq);
        if(place_seq != pre_place_seq)
        {
            pre_place_seq = place_seq;

            ros::param::get("/obs_max_z", via_z);
            x_bias = rand() / double(RAND_MAX) * 0.03;
            y_bias = rand() / double(RAND_MAX) * 0.03;
            z_bias = 0.3;
            //via_x += x_bias;
            //via_y += y_bias;
            //via_z += z_bias;
            //specify pose parallel to the box
            //move_via_point

            switch(place_seq)
            {
                case 1:
                    //move_end_point
                    break;

                case 2:
                    //move_end_point
                    break;

                case 3:
                    //move_end_point
                    break;

                case 4:
                    //move_end_point
                    break;

                default:
                    //move_to_center_point
                    break;


            }

            std::vector<double> start_icp_pose;
            ros::param::get("/start_tran",start_icp_pose);
            for (int row = 0; row < 4; row++) {
                for(int column=0;column<4;column++) {
                    T_start_pose(row, column) = start_icp_pose[row*4+column];
                }
            }
            T_start_pose(2,3) += z_bias;

            std::vector<double> end_icp_pose;
            ros::param::get("/end_tran",end_icp_pose);
            for (int row = 0; row < 4; row++) {
                for(int column=0;column<4;column++) {
                    T_end_pose(row, column) = start_icp_pose[row*4+column];
                }
            }
            T_end_pose(2,3) += z_bias;

            R_start = T_start_pose.rotation();
            R_end = T_end_pose.rotation();
            R_compensate = R_start.inverse()*R_end;

            print3x3Matrix(R_compensate);
            angle_axis = R_compensate;
            angle = angle_axis.angle();
            Vector3d rot_axis = angle_axis.axis();

            // go to nearest pose
            if(angle > PI/4)
            {
                angle -= PI/2;
            }
            else if(angle < -PI/4)
            {
                angle += PI/2;
            }

            cout << "angle: " << angle*180/PI<< endl;
            cout << "axis: " << rot_axis << endl;

            // extract points
            int size = traj_points.size();
            int via_point1_index = size/3;
            int via_point2_index = via_point1_index*2;

            T_via_point1 = T_start_pose;
            T_via_point2 = T_end_pose;

            T_via_point1(0,3) = traj_points[via_point1_index].x;
            T_via_point1(1,3) = traj_points[via_point1_index].y;
            T_via_point1(2,3) = traj_points[via_point1_index].z + z_bias;

            T_via_point2(0,3) = traj_points[via_point2_index].x;
            T_via_point2(1,3) = traj_points[via_point2_index].y;
            T_via_point2(2,3) = traj_points[via_point2_index].z + z_bias;

            break;

        }



        loop_rate.sleep();
    }

    //        // test robot
    std::cout<<robot.login_in("192.168.2.76")<<std::endl;
    robot.power_on();
    robot.enable_robot();
    //
    JointValue refJoint;
    ////        robot.get_joint_position(&refJoint);
    refJoint.jVal[0] = PI/2;
    refJoint.jVal[1] = PI/2;
    refJoint.jVal[2] = -PI/2;
    refJoint.jVal[3] = PI/2;
    refJoint.jVal[4] = PI/2;
    refJoint.jVal[5] = 0;
    //
    //        refJoint.jVal[0] += 0.2;
    //        cout <<        robot.joint_move(&refJoint, MoveMode::ABS, TRUE, 0.5)<< endl;

    /*
    // test matrix
    R_start(0,0) = 0.133;
    R_start(0,1) = 0.989;
    R_start(0,2) = -0.07;
    R_start(1,0) = -0.987;
    R_start(1,1) = 0.126;
    R_start(1,2) = -0.094;
    R_start(2,0) = -0.085;
    R_start(2,1) = 0.081;
    R_start(2,2) = 0.993;

    R_end(0,0) = 0.738;
    R_end(0,1) = 0.673;
    R_end(0,2) = -0.04;
    R_end(1,0) = -0.674;
    R_end(1,1) = 0.736;
    R_end(1,2) = -0.054;
    R_end(2,0) = -0.007;
    R_end(2,1) = 0.067;
    R_end(2,2) = 0.998;

    R_compensate = R_start.inverse()*R_end;

    print3x3Matrix(R_compensate);
    angle_axis = R_compensate;
    double angle = angle_axis.angle();
    Vector3d rot_axis = angle_axis.axis();
     */


    //        ros::Duration(3).sleep();
    //control robot
    sendJointMovePoint(T_to_cartesianPose(T_start_pose));

    robot.set_digital_output(IOType::IO_CABINET,1,true); // grasp
    ros::Duration(0.5).sleep();

    sendJointMovePoint(T_to_cartesianPose(T_via_point1));
    sendJointMovePoint(T_to_cartesianPose(T_via_point2), -angle);


    sendJointMovePoint(T_to_cartesianPose(T_end_pose), -angle);

    CartesianPose* tcp;
    robot.get_tcp_position(tcp);
    tcp->tran.z -= 10;  // move robot down 1cm because the demonstration end pose is above the box
    sendJointMovePoint(*tcp);

    robot.joint_move(&refJoint, MoveMode::ABS, TRUE, 0.5);  // back to home pose

    robot.set_digital_output(IOType::IO_CABINET,1,false); // release
    ros::Duration(0.5).sleep();

    //        robot.disable_robot();
    //        robot.power_off();

    return 0;
}
