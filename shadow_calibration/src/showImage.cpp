//OPENCV INCLUDES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <unistd.h> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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
#include "calibrationbase.h"
using namespace std;
using namespace cv;

//PARAMETERS
int width ,height, squaresize;
CAMERA_INTRINSIC_PARAMETERS camera;
string  camera_dataPath;
SpacialPoint spoint1,spoint2,spoint3;
ImagePoint point1,point2,point3;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);

    cv::namedWindow("Autocalibration");
  }
  ~ImageConverter()
  {
    cv::destroyWindow("Autocalibration");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat image=cv_ptr->image;
    tf::StampedTransform boardPoseture=getCurrentPose("/camera_rgb_optical_frame","/calibration_board" );

     Mat transform=stampedTransform2Mat(boardPoseture);
     point1=projectPoint(camera,spoint1,transform);
     point2=projectPoint(camera,spoint2,transform);
     point3=projectPoint(camera,spoint3,transform);
     line(image,Point(point1.x,point1.y),Point(point2.x,point2.y),Scalar(0,0,255),2,8);
     line(image,Point(point1.x,point1.y),Point(point3.x,point3.y),Scalar(0,255,0),2,8);
     cout<<point1.x<<" "<<point1.y<<endl;
     cout<<point2.x<<" "<<point2.y<<endl;
     Mat image2;
     resize(image,image2,Size(640,480),0,0,CV_INTER_LINEAR);
   cv::imshow("Autocalibration", image2);
    cv::waitKey(30);
  }
};

int main(int argc, char** argv)
{   

  ros::init(argc, argv, "showImage");
  ros::start();
    if(argc != 5)
         {
             cerr << endl << "Usage: rosrun autocalibration  showImage   width  height  squaresize cameradata" << endl;
             ros::shutdown();
             return 1;
         }
    width=atoi(argv[1]);
       height=atoi(argv[2]);
       squaresize=atoi(argv[3]);
      camera_dataPath=argv[4];
      Mat cameraMatrix;
      FileStorage fs(camera_dataPath,FileStorage::READ);
               fs["camera_matrix"]>>cameraMatrix;
               fs.release();
      camera.fx=cameraMatrix.at<float>(0,0);
       camera.fy=cameraMatrix.at<float>(1,1);
        camera.cx=cameraMatrix.at<float>(0,2);
         camera.cy=cameraMatrix.at<float>(1,2);
   spoint1.x=0;
   spoint1.y=0;
   spoint1.z=0;

   spoint2.x=(double)(squaresize*(width-1));
   spoint2.y=0;
   spoint2.z=0;

   spoint3.x=0;
   spoint3.y=(double)(squaresize*(height-1));
   spoint3.z=0;

   ros::NodeHandle n;
   ImageConverter ic;   //show image
   ros::spin();
   return 0;
      
}


