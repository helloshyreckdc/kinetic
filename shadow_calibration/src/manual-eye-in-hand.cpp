#include"calibrationbase.h"
using namespace std;
using namespace cv;
//PARAMETERSD
bool moveitSuccess;
bool markerSuccess;
bool boardSuccess;

CAMERA_INTRINSIC_PARAMETERS camera;
Extrinsic_Parameter E[20],Y[20],C[190],D[190];


double calculate_value(Mat qx ,Mat tx,int NumOFCD);
void initPoint();
Mat transend2board;
Mat transbase2camera;
SpacialPoint spoint1,spoint2,spoint3,spoint4;
ImagePoint point1,point2,point3,point4;

void process(const sensor_msgs::ImageConstPtr& cam_image)
{
cv_bridge::CvImagePtr cv_ptr;
try
{
  cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
}

catch (cv_bridge::Exception& e)
{
  ROS_ERROR("cv_bridge exception:%s",e.what());
  return;
}
Mat image= cv_ptr->image;
//read tf
tf::StampedTransform  currPose;
currPose=getCurrentPose("/base_link","/ee_link");

Mat transbase2end=stampedTransform2Mat(currPose);
Mat extrinsic(4,4,CV_64F);
extrinsic=transbase2camera.inv()*transbase2end*transend2board;
point1=projectPoint(camera,spoint1,extrinsic);
point2=projectPoint(camera,spoint2,extrinsic);
point3=projectPoint(camera,spoint3,extrinsic);
point4=projectPoint(camera,spoint4,extrinsic);
line(image,Point(point1.x,point1.y),Point(point2.x,point2.y),Scalar(0,0,255),2,8);
line(image,Point(point2.x,point2.y),Point(point3.x,point3.y),Scalar(0,255,0),2,8);
line(image,Point(point3.x,point3.y),Point(point4.x,point4.y),Scalar(0,255,0),2,8);
line(image,Point(point4.x,point4.y),Point(point1.x,point1.y),Scalar(0,255,0),2,8);
imshow("demo",image);
waitKey(30);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Eye_in_hand_calibration");
    ros::start();

    if(argc != 2)
     {
         cerr << endl << "Usage: rosrun shadow_calibration manual-eye-in-hand     numberofpose" << endl;
	 cout<<argc<<endl;
         ros::shutdown();
         return 1;
     }
    ros::NodeHandle n;
     camera = getDefaultCamera();
     int numOfPose=atoi(argv[1]);

    initPoint();
/************************************************************************************/
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("start Shadow Calibration ");
  ROS_INFO("Number of Pose = %d",numOfPose);


 int successCount=0;
 int CDcount=0;
 int resultrows=0;

  for (int i=0;i<numOfPose;i++)
 {
    cout<<"please move the robot and press Enter to continue"<<endl;
    getchar();
    sleep(3);
    ros::spinOnce();
     //get board poseture
      tf::StampedTransform markerPoseture;
      markerSuccess=findBoardPoseture("/calibration_board" ,
                                       "/camera_rgb_optical_frame",&markerPoseture);
  moveitSuccess=true;
  cout<<"moveitsuccess= "<<moveitSuccess<<endl;
  cout<<"boardsuccess= "<<markerSuccess<<endl;
//  //the robot move correctly and find marker pose
  if((moveitSuccess==true)&&(markerSuccess==true))
    {
     E[successCount].transform=stampedTransform2Mat(markerPoseture);
     E[successCount].k=findKFromTrans(E[successCount].transform);
     E[successCount].t=findTFromTrans(E[successCount].transform);


     Y[successCount].transform= stampedTransform2Mat(getCurrentPose("base_link","ee_link"));
     Y[successCount].k=findKFromTrans(Y[successCount].transform);
      Y[successCount].t=findTFromTrans(Y[successCount].transform);
       successCount++;

    }
}



  for(int i=0;i<successCount;i++)
  {
      for (int j=0;j<successCount;j++)
      {
              if((i!=j)&&(i<j))
              {
              C[CDcount].transform=E[i].transform*(E[j].transform.inv());
              C[CDcount].k=findKFromTrans(C[CDcount].transform);
              C[CDcount].t=findTFromTrans(C[CDcount].transform);

              D[CDcount].transform=Y[i].transform.inv()*(Y[j].transform);
              D[CDcount].k=findKFromTrans(D[CDcount].transform);
              D[CDcount].t=findTFromTrans(D[CDcount].transform);

              CDcount++;
              }
      }
  }

  resultrows=CDcount*(CDcount-1)/2;
  cout<<resultrows<<endl;
  Mat newResult(resultrows,8,CV_64F);
  int count=0;
  for(int i=0;i<CDcount;i++)
  {
      for (int j=0;j<CDcount;j++)
      {
              if((i!=j)&&(i<j))
              {
                  Mat R=calculate_R(i,j,C[i], C[j],D[i], D[j]);
                  Mat t=calculate_T(R,i,j,C[i], C[j],D[i], D[j]);
                  Mat quat=dcm2quat(R);
                   newResult.at<double>(count,0)=quat.at<double>(0,0);
                   newResult.at<double>(count,1)=quat.at<double>(0,1);
                   newResult.at<double>(count,2)=quat.at<double>(0,2);
                   newResult.at<double>(count,3)=quat.at<double>(0,3);

                   newResult.at<double>(count,4)=t.at<double>(0,0);
                   newResult.at<double>(count,5)=t.at<double>(1,0);
                   newResult.at<double>(count,6)=t.at<double>(2,0);
                   newResult.at<double>(count,7)=calculate_value(quat,t,CDcount);
                    count++;

              }
      }
  }
  FileStorage fs2("/home/mzm/newresult.xml",FileStorage::WRITE);
  fs2<<"newresult"<<newResult;
  fs2.release();
  ROS_INFO("Done");

  //寻找最好的结果
  Mat bestresult(1,8,CV_64F);
  double bestvalue=1000000000000.0;
  int bestcount;

  for(int i=0;i<resultrows;i++)
  {
      if(newResult.at<double>(i,7)<1000000000000.0)
      {
        if(newResult.at<double>(i,7)<bestvalue)
        {
           bestresult.at<double>(0,0)=newResult.at<double>(i,0);
           bestresult.at<double>(0,1)=newResult.at<double>(i,1);
           bestresult.at<double>(0,2)=newResult.at<double>(i,2);
           bestresult.at<double>(0,3)=newResult.at<double>(i,3);
           bestresult.at<double>(0,4)=newResult.at<double>(i,4);
           bestresult.at<double>(0,5)=newResult.at<double>(i,5);
           bestresult.at<double>(0,6)=newResult.at<double>(i,6);
           bestresult.at<double>(0,7)=newResult.at<double>(i,7);
           bestvalue=newResult.at<double>(i,7);
           bestcount=i;
          }
      }
  }

  //开始创建最好结果
  Mat bestresult_q(1,4,CV_64F);
  Mat bestresult_t(1,3,CV_64F);
  bestresult_q.at<double>(0,0)=bestresult.at<double>(0,0);
  bestresult_q.at<double>(0,1)=bestresult.at<double>(0,1);
  bestresult_q.at<double>(0,2)=bestresult.at<double>(0,2);
  bestresult_q.at<double>(0,3)=bestresult.at<double>(0,3);

  bestresult_t.at<double>(0,0)=bestresult.at<double>(0,4);
  bestresult_t.at<double>(0,1)=bestresult.at<double>(0,5);
  bestresult_t.at<double>(0,2)=bestresult.at<double>(0,6);
  Mat trans_result=buildTransformMatrix_QT(bestresult_q,bestresult_t);      //best  result
  cout<<trans_result<<endl;
  Extrinsic_Parameter F[successCount];

   for(int i =0;i<successCount;i++)
   {
    F[i].transform= (Y[i].transform.inv()*trans_result)*E[i].transform;   //find F
    F[i].k=findKFromTrans(F[i].transform);
    F[i].t=findTFromTrans(F[i].transform);
   }
   Mat trans_x=trans_result;
   Extrinsic_Parameter F_real;
   Mat tempk=F[0].k;
   Mat tempt=F[0].t;

   for(int i=1;i<successCount;i++)
   {
    tempk+=F[i].k;
    tempt+=F[i].t;
   }
   F_real.k=tempk/successCount;
   F_real.t=tempt/successCount;

   F_real.transform=buildTransformMatrix_RT(F_real.k.t(),F_real.t.t());
   cout<<"F_real"<<endl<<F_real.transform<<endl;
/******************************************************************************/
   transbase2camera=trans_x;
   transend2board=F_real.transform;
   FileStorage fs3("/home/mzm/end2board.xml",FileStorage::WRITE);
   fs3<<"End2board"<<F_real.transform;
   fs3<<"transbase2camera"<<trans_result;
   fs3.release();

  ee_link2camera_link(trans_x);
//   image_transport::ImageTransport it(n);
//   image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw",1,process);
//   cv::namedWindow("demo");

//  ros::Rate  rate(10);
//  while(ros::ok()) rate.sleep();

   return 0;

}
double calculate_value(Mat qx ,Mat tx,int NumOFCD)
{

//   cout<<"qx"<<endl<<qx<<endl;
//   cout<<"tx"<<endl<<tx<<endl;
  double  sumOfValue=0;


   for(int i=0;i<NumOFCD;i++)
   {

       Mat nc=C[i].k/norm(C[i].k);
       Mat nd=D[i].k/norm(D[i].k);

       Mat  ncq(1,4,CV_64F);
       Mat  ndq(1,4,CV_64F);
       for(int j =0;j<3;j++)
       {
          ncq.at<double>(0,j+1)=nc.at<double>(j,0);
           ndq.at<double>(0,j+1)=nd.at<double>(j,0);
        }
       ncq.at<double>(0,0)=0.0;
       ndq.at<double>(0,0)=0.0;
       Mat tc(1,4,CV_64F);
       Mat td(1,4,CV_64F);

       for(int j =0;j<3;j++)
       {

          tc.at<double>(0,j+1)=C[i].t.at<double>(j,0);
           td.at<double>(0,j+1)=D[i].t.at<double>(j,0);
       }

      tc.at<double>(0,0)=0.0;
      td.at<double>(0,0)=0.0;

      Mat Rd(3,3,CV_64F);
          Rodrigues(D[i].k,Rd);
      Mat temp=(Rd-Mat::eye(3,3,CV_64F))*tx;

      Mat tempt(1,4,CV_64F);
      tempt.at<double>(0,0)=0.0;
      tempt.at<double>(0,1)=temp.at<double>(0,0);
      tempt.at<double>(0,2)=temp.at<double>(1,0);
      tempt.at<double>(0,3)=temp.at<double>(2,0);
       sumOfValue=sumOfValue+squareofQuat(quatMulti(ndq,qx)-quatMulti(qx,ncq));
       sumOfValue=sumOfValue+squareofQuat(quatMulti(qx,tc)-quatMulti(tempt,qx)-quatMulti(td,qx));
   }
    sumOfValue=sumOfValue +1000*(1-squareofQuat(qx))*(1-squareofQuat(qx));
     return sumOfValue;

}

void initPoint()
{
    spoint1.x=0;
    spoint1.y=0;
    spoint1.z=0;

    spoint2.x=75;
    spoint2.y=0;
    spoint2.z=0;

    spoint3.x=75;
    spoint3.y=50;
    spoint3.z=0;

    spoint4.x=0;
    spoint4.y=50;
    spoint4.z=0;


}




