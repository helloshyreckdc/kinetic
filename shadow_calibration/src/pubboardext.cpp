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
#include <tf/transform_broadcaster.h>
#include "calibrationbase.h"

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
#include <setting.h>
using namespace std;
using namespace cv;

//CALIBRATION PARAMETERS
int width ,height, squaresize;
Settings  s;
Mat object_point;
Mat cameraMatrix;
Mat distCoeffs;
string  camera_dataPath;
Size imageSize;
/*******************************************************************/

/******************************************************************/
void initSettings();
Mat calcBoardCornerPositions(Size boardSize, float squareSize);
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
   tf::TransformBroadcaster br;
  tf::Transform transform;

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

    Mat view= cv_ptr->image;
   imageSize=view.size();
   if( s.flipVertical )    flip( view, view, 0 );
           vector<Point2f> pointBuf;
           bool found;
           Mat viewtemp ;
           cvtColor(view,viewtemp,COLOR_BGR2GRAY);
           cout<<s.boardSize<<endl;
           found = findChessboardCorners( view, s.boardSize, pointBuf,
               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
           if ( found)                // If done with success,
           {        cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>found corners>>>>>>>>>>>>>>>>>>>>>>"<<endl;
                 // improve the found corners' coordinate accuracy for chessboard
                   if( s.calibrationPattern == Settings::CHESSBOARD)
                   {
                       Mat viewGray;
                       cvtColor(view, viewGray, COLOR_BGR2GRAY);
                       cornerSubPix( viewGray, pointBuf, Size(11,11),
                           Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                   }
                   drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );




                   Mat drawcorner;
                   drawcorner=view.clone();

                   Mat  imagepoint(pointBuf.size(),2,CV_32F);
                   for(int i=0;i<pointBuf.size();i++)
                   {

                      imagepoint.at<float>(i,0)=pointBuf[i].x;
                      imagepoint.at<float>(i,1)=pointBuf[i].y;

                   }

                   CvMat* cv_rotation_vector=cvCreateMat(3,1,CV_32F);
                   CvMat* cv_translation_vector=cvCreateMat(3,1,CV_32F);

                   CvMat cv_object_point=object_point;
                   CvMat cv_imagepoint=imagepoint;
                   CvMat cv_cameraMatrix=cameraMatrix;
                   CvMat cv_distCoeffs=distCoeffs;


                   cvFindExtrinsicCameraParams2(&cv_object_point,&cv_imagepoint,&cv_cameraMatrix,&cv_distCoeffs,cv_rotation_vector,cv_translation_vector);
                   cout<<cv_translation_vector<<endl;
                   float *rotation1=(float*)cvPtr2D(cv_rotation_vector, 0, 0);
                   float r1,r2,r3;
                   r1=*rotation1;
                   rotation1++;
                   r2=*rotation1;
                   rotation1++;
                   r3=*rotation1;




                   float *translation1=(float*)cvPtr2D(cv_translation_vector, 0, 0);
                   float t1,t2,t3;
                   t1=*translation1;
                   translation1++;
                   t2=*translation1;
                   translation1++;
                   t3=*translation1;


                    //setOrigin
                    transform.setOrigin( tf::Vector3(0.001*t1,0.001*t2,0.001*t3) );

                    Mat rodri(3,1,CV_64F);
                    rodri.at<double>(0,0)=r1;
                    rodri.at<double>(1,0)=r2;
                    rodri.at<double>(2,0)=r3;
                    Mat dcm(3,3,CV_64F);
                    Rodrigues(rodri,dcm);
                    Mat quat=dcm2quat(dcm);
                    double w,x,y,z;
                    w=-quat.at<double>(0,0);
                    x=quat.at<double>(0,1);
                    y=quat.at<double>(0,2);
                    z=quat.at<double>(0,3);
                    tf::Quaternion q(x,y,z,w);
                    //setRotation
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame","/calibration_board"));
                   
                     ros::spinOnce();

           }

    cout<<found<<endl;
    

/*****************************************************************************************************/

//   cv::imshow("Autocalibration", view);
//    cv::waitKey(50);

  }
};

int main(int argc, char** argv)
{    

    ros::init(argc, argv, "pubboardext");
    ros::start();
    if(argc != 5)
         {
             cerr << endl << "Usage: rosrun autocalibration  pubboardext     width  height  squaresize cameradata" << endl;
	     cerr<<argc<<endl;
             ros::shutdown();
             return 1;
         }
    width=atoi(argv[1]);
    height=atoi(argv[2]);
    squaresize=atoi(argv[3]);
   camera_dataPath=argv[4];

   initSettings();
   
   ros::NodeHandle n;
   ImageConverter ic;   //show image
   ros::spin();
   return 0;
      
}

void initSettings()
{
    s.boardSize.width=width;
    s.boardSize.height=height;
    s.squareSize=squaresize;
    s.patternToUse=1;
    s.input=" ";
    s.flipVertical=0;
    s.delay=100;
    s.nrFrames=25;
    s.aspectRatio=1;
    s.calibZeroTangentDist=1;
    s.calibFixPrincipalPoint=1;
    s.outputFileName=" ";
    s.bwritePoints=1;
    s.bwriteExtrinsics=1;
    s.showUndistorsed=1;
    s.inputType=Settings::IMAGE_LIST;

    s.flag = 0;
    if(s.calibFixPrincipalPoint) s.flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
    cout<<"flag="<<s.flag<<endl;
    if(s.calibZeroTangentDist)   s.flag |= CV_CALIB_ZERO_TANGENT_DIST;
      cout<<"flag="<<s.flag<<endl;
    if(s.aspectRatio)            s.flag |= CV_CALIB_FIX_ASPECT_RATIO;
    cout<<"flag="<<s.flag<<endl;

   s. calibrationPattern = Settings::CHESSBOARD;

    if (s.calibrationPattern == Settings::NOT_EXISTING)
        {
            cerr << " Inexistent camera calibration mode: " <<s. patternToUse << endl;
            s.goodInput = false;
        }
    s.atImageList = 0;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;

    object_point=calcBoardCornerPositions(s.boardSize,s.squareSize);
    cout<<object_point<<endl;

/**************************INPUT CAMERA_DATA**d***********************************/

    FileStorage fs(camera_dataPath,FileStorage::READ);
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coeffs"]>>distCoeffs;
    fs.release();
}

Mat  calcBoardCornerPositions(Size boardSize, float squareSize)
{
    int width=boardSize.width;
    int height =boardSize.height;

    Mat corners(width*height,3,CV_32F);
        for( int i = 0; i < boardSize.height; ++i )
           {

            for( int j = 0; j < boardSize.width; ++j )
            {
                corners.at<float>(i*boardSize.width+j,0)=float( j*squareSize );
                corners.at<float>(i*boardSize.width+j,1)=float( i*squareSize );
                corners.at<float>(i*boardSize.width+j,2)=0.0;
            }
          }

        return corners;
}


