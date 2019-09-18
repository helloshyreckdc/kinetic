//OPENCV INCLUDES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>


// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <unistd.h> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
//相机内参数结构体
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};
//图片点结构体
struct ImagePoint
{
     double x,y;
 };
//空间点结构体
struct SpacialPoint{
  double x,y,z;
};

struct Extrinsic_Parameter
{
  Mat transform;
  Mat k;
  Mat t;
};
//寻找最开始标定版的姿态 
//右x下y返回1  上x右y返回2 左x上y返回3 下x左y返回4 
int findFirstBoardPose(String referenceframe,String targetframe);
double getFirstDistance(String referenceframe,String targetframe);

//use  rotation(1*3) vector and t(1*3) vector to build transform matrix
Mat buildTransformMatrix_RT(Mat r,Mat t );  

//use  quat()vector and t(1*3) vetor to build transform matrix 
Mat buildTransformMatrix_QT(Mat q,Mat t);   
Mat dcm2quat(Mat dcm );
Mat quat2dcm(Mat qnb);
Mat change2StdQuat(Mat pose);
int sign (double i);

Mat findKFromTrans(Mat transform);
Mat findTFromTrans(Mat transform);
double norm(Mat k ); //3*1 to double
Mat quatMulti(Mat q1,Mat q2);
double squareofQuat(Mat quat);
bool moveToPose(String jointgroup,geometry_msgs::Pose  target_pose);
// 在笛卡尔空间进行规划
bool moveToPoseCartesianPath(String jointgroup,geometry_msgs::Pose  target_pose);
// 使用关节值来控制机器人
bool movetoPoseJointstate(String jointgroup,Mat jointstate);
//寻找二维码 参数marker是二维码对应的frameid cameraframe是相机的frame 结果通过resulf返回
//如果找到返回true 如果没有找到返回false
bool findMarkerPoseture(String marker ,String cameraframe,tf::StampedTransform *result);
bool findBoardPoseture(String marker ,String cameraframe,tf::StampedTransform *result);
//生成末端姿态的变化矩阵
Mat  createEndRotation(double angle_,int divisor,int degree);  

//将stampedTransform类型转化为Mat类型  单位改为mm 
Mat stampedTransform2Mat(tf::StampedTransform st);  //mm

//获取当前机器人姿态 也可以是其他tf
tf::StampedTransform  getCurrentPose(String  referenceframe,String targetframe);
//将mat类型转化为对应的geometry_msgs::Pose 类型
geometry_msgs::Pose  mat2geometryPose(Mat transform);
//计算空间点到图片上的投影  空间的点会先通过Mat transform转换到相机坐标系中
ImagePoint  projectPoint(CAMERA_INTRINSIC_PARAMETERS camera, SpacialPoint point,Mat transform);
//根据提供的C D矩阵计算手眼矩阵的旋转部分  返回3*3 Matrix
Mat calculate_R(int i,int j,Extrinsic_Parameter C_i,Extrinsic_Parameter C_j,Extrinsic_Parameter D_i,Extrinsic_Parameter D_j);
//根据提供的C D矩阵计算手眼矩阵的偏移部分  返回3*3 Matrix
Mat calculate_T(Mat R,int i, int j,Extrinsic_Parameter C_i,Extrinsic_Parameter C_j,Extrinsic_Parameter D_i,Extrinsic_Parameter D_j);
//将结果的１＊７矩阵变成Mat矩阵
Mat resultRow2Mat(Mat resultRow);
//在得到手眼标定结果后　规定二维码的位置　计算二维码相对相机的姿态
Mat createQRextinsic(double angle,double distance,int divisor,int degree);
//在得到手眼标定结果后  规定标定班的位置  计算标定班相对相机的姿态  degree为第几次
Mat createBoardextrinsic(double angle,double distance,int divisor,int degree,int firstPose);
//read parameters 
//生成最后的base_link到camera_link的指令
void base_link2camera_link(Mat transbase2camera_rbg_optical_frame);
void ee_link2camera_link(Mat transbase2camera_rbg_optical_frame);
class ParameterReader
{
public:
    ParameterReader( string filename="calibration_parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
};
