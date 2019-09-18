#include"calibrationbase.h"
using namespace std;
using namespace cv;

//寻找最开始标定版的姿态
//右x下y返回1  上x右y返回2 左x上y返回3 下x左y返回4
int findFirstBoardPose(String referenceframe,String targetframe)
{
   tf::StampedTransform st=getCurrentPose(referenceframe,targetframe);
   Mat extrinsic=stampedTransform2Mat(st);
   Mat point0(4,1,CV_64F);
   Mat pointx(4,1,CV_64F);
   point0.at<double>(0,0)=0;
   point0.at<double>(1,0)=0;
   point0.at<double>(2,0)=0;
   point0.at<double>(3,0)=1;

   pointx.at<double>(0,0)=25;
   pointx.at<double>(1,0)=0;
   pointx.at<double>(2,0)=0;
   pointx.at<double>(3,0)=1;

   Mat camerapoint0=extrinsic*point0;
   Mat camerapointx=extrinsic*pointx;
   Mat camerapointRelative=camerapointx-camerapoint0;
   double x=camerapointRelative.at<double>(0,0);
   double y=camerapointRelative.at<double>(1,0);

   if((abs(x)<=abs(y))&&(y>0))
   {
       return 4;
   }
   else if((abs(x)<=abs(y))&&(y<0))
   {
       return 2;
   }
   else if((abs(x)>abs(y))&&(x>0))
   {
       return 1;
   }
   else
   {

      return 3;
   }



}

double getFirstDistance(String referenceframe,String targetframe)
{
   tf::StampedTransform st=getCurrentPose(referenceframe,targetframe);
   Mat extrinsic=stampedTransform2Mat(st);
   double distance=extrinsic.at<double>(2,3);
   return distance ;


}

//use  rotation(1*3) vector and t(1*3) vector
Mat buildTransformMatrix_RT(Mat r,Mat t )
{
    if((r.cols==3 && r.rows==1 )&&( t.cols==3 && t.rows==1))
    {
     Mat trans=Mat::zeros(4,4,CV_64F);
     Mat rotation;
     Rodrigues(r,rotation);
     for(int i=0;i<3;i++)
     {
         for (int j=0;j<3;j++)
         {
              trans.at<double>(i,j)=rotation.at<double>(i,j);
         }
     }
     trans.at<double>(0,3)=t.at<double>(0,0);
     trans.at<double>(1,3)=t.at<double>(0,1);
     trans.at<double>(2,3)=t.at<double>(0,2);
     trans.at<double>(3,3)=1.0;
     return trans;
    }
    else
    {
        cout<<"error  the mat r or mat t is not correct "<<endl;
        return  Mat::zeros(1,1,CV_64F);
    }
}

//use  quat()vector and t(1*3) vetor to build transform matrix
Mat buildTransformMatrix_QT(Mat q,Mat t)
{
    if((q.cols==4 && q.rows==1 )&&( t.cols==3 && t.rows==1))
    {
     Mat trans=Mat::zeros(4,4,CV_64F);
     Mat rotation;
     rotation=quat2dcm(q);
     for(int i=0;i<3;i++)
     {
         for (int j=0;j<3;j++)
         {
              trans.at<double>(i,j)=rotation.at<double>(i,j);
         }
     }
     trans.at<double>(0,3)=t.at<double>(0,0);
     trans.at<double>(1,3)=t.at<double>(0,1);
     trans.at<double>(2,3)=t.at<double>(0,2);
     trans.at<double>(3,3)=1.0;
     return trans;
    }
    else
    {
        cout<<"error  the mat q or mat t is not correct "<<endl;
    }
}
//calculate quat(1*4 ) from rotation matirx
Mat dcm2quat(Mat dcm )
{
    Mat qnb(1,4,CV_64F);
        double temp11=dcm.at<double>(0,0);
        double temp12=dcm.at<double>(0,1);
        double temp13=dcm.at<double>(0,2);
        double temp21=dcm.at<double>(1,0);
        double temp22=dcm.at<double>(1,1);
        double temp23=dcm.at<double>(1,2);
        double temp31=dcm.at<double>(2,0);
        double temp32=dcm.at<double>(2,1);
        double temp33=dcm.at<double>(2,2);
      qnb.at<double>(0,0)=sqrt(abs(1.0 + temp11 + temp22 + temp33))/2.0;
      qnb.at<double>(0,1)=-1.0*sign(temp32-temp23) * sqrt(abs(1.0 + temp11 - temp22 - temp33))/2.0;
      qnb.at<double>(0,2)=-1.0*sign(temp13-temp31) * sqrt(abs(1.0 - temp11 + temp22 - temp33))/2.0;
      qnb.at<double>(0,3)=-1.0*sign(temp21-temp12) * sqrt(abs(1.0 - temp11 - temp22 + temp33))/2.0;
      return qnb;
}
//calculat  rotation matrix from quat (1*4)
Mat quat2dcm(Mat qnb)
{
    double qnb1=qnb.at<double>(0,0);
    double qnb2=qnb.at<double>(0,1);
    double qnb3=qnb.at<double>(0,2);
    double qnb4=qnb.at<double>(0,3);

    double q11=qnb1*qnb1 ;
    double q12=qnb1*qnb2;
    double q13=qnb1*qnb3;
    double q14=qnb1*qnb4;
        double q21=qnb2*qnb1;
        double q22=qnb2*qnb2;
        double q23=qnb2*qnb3;
        double q24=qnb2*qnb4;
    double q31=qnb3*qnb1;
    double q32=qnb3*qnb2;
    double q33=qnb3*qnb3;
    double q34=qnb3*qnb4;
        double q41=qnb4*qnb1;
        double q42=qnb4*qnb2;
        double q43=qnb4*qnb3;
        double q44=qnb4*qnb4;

    Mat Cnb(3,3,CV_64F);
    Cnb.at<double>(0,0)=q11+q22-q33-q44;
    Cnb.at<double>(0,1)=2*(q23+q14);
    Cnb.at<double>(0,2)=2*(q24-q13);
            Cnb.at<double>(1,0)=2*(q23-q14);
            Cnb.at<double>(1,1)=q11-q22+q33-q44;
            Cnb.at<double>(1,2)=2*(q34+q12);
                Cnb.at<double>(2,0)=2*(q24+q13);
                Cnb.at<double>(2,1)=2*(q34-q12);
                Cnb.at<double>(2,2)=q11-q22-q33+q44;

    return Cnb;
}

int sign (double i)
{  if(i>0)
        return 1;
    else if(i<0)
        return -1;
    else
        return 0;
 }

Mat change2StdQuat(Mat pose_ros)
{
    int n=pose_ros.rows;
    Mat pose(n,7,CV_64F);
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<3;j++)
        {
        pose.at<double>(i,j)=pose_ros.at<double>(i,j)*1000;
         }
    }
      double temp;
    for (int i=0;i<n;i++)
       {
        temp=pose_ros.at<double>(i,6);
        pose.at<double>(i,6)=pose_ros.at<double>(i,5);
        pose.at<double>(i,5)=pose_ros.at<double>(i,4);
        pose.at<double>(i,4)=pose_ros.at<double>(i,3);
        pose.at<double>(i,3)=-temp;
      }
      return pose;
}

Mat findKFromTrans(Mat transform)
{
  Mat rotation=transform(Range(0,3),Range(0,3));
  Mat k(3,1,CV_64F);
  Rodrigues(rotation,k);
  return k;
}
Mat findTFromTrans(Mat transform)
{
    Mat t;
    t=transform(Range(0,3),Range(3,4));
    return t ;
}

double norm(Mat k ) //3*1 to double
{
 double  sum=0;
 double temp1=k.at<double>(0,0)*k.at<double>(0,0);
  double temp2=k.at<double>(1,0)*k.at<double>(1,0);
   double temp3=k.at<double>(2,0)*k.at<double>(2,0);
  sum=temp1+temp2+temp3;
  sum=sqrt(sum);
  return sum;
}

Mat quatMulti(Mat q1,Mat q2)
{
   double w1=q1.at<double>(0,0);
   double x1=q1.at<double>(0,1);
   double y1=q1.at<double>(0,2);
   double z1=q1.at<double>(0,3);

   double w2=q2.at<double>(0,0);
   double x2=q2.at<double>(0,1);
   double y2=q2.at<double>(0,2);
   double z2=q2.at<double>(0,3);

  Mat quat(1,4,CV_64F);
 quat.at<double>(0,0)=w1*w2-x1*x2-y1*y2-z1*z2;
 quat.at<double>(0,1)= w1*x2+x1*w2+z1*y2-y1*z2;
 quat.at<double>(0,2)=  w1*y2+y1*w2+x1*z2-z1*x2;
 quat.at<double>(0,3)=  w1*z2+z1*w2+y1*x2-x1*y2;
 return quat;
}

double squareofQuat(Mat quat)
{
   double sum;
      sum=sum+ (quat.at<double>(0,0))*quat.at<double>(0,0) ;
      sum=sum+ (quat.at<double>(0,1))*quat.at<double>(0,1) ;
      sum=sum+ (quat.at<double>(0,2))*quat.at<double>(0,2) ;
      sum=sum+ (quat.at<double>(0,3))*quat.at<double>(0,3) ;
   return sum ;
}

//something is wrong
bool moveToPose(String jointgroup,geometry_msgs::Pose  target_pose)
{
//
//    moveit::planning_interface::MoveGroup group("manipulator");
//    moveit::planning_interface::MoveGroup::Plan my_plan;
//    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
//    group.setPoseReferenceFrame("base_link");
//    group.setPoseTarget(target_pose);
//    //cout<<group.getEndEffectorLink()<<endl;
//    //cout<<group.getPoseTarget()<<endl;
//    group.setPlannerId("RRTConnectkConfigDefault");
//    bool success = group.plan(my_plan);
//
//    sleep(5);
//    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
//    if(success){
//      group.execute(my_plan);
//    }
//    sleep(5);
//    ros::spinOnce();
//    return success;
}

bool moveToPoseCartesianPath(String jointgroup,geometry_msgs::Pose  target_pose)
{
//
//    moveit::planning_interface::MoveGroup group("manipulator");
//    moveit::planning_interface::MoveGroup::Plan my_plan;
//
//    double dx,dy,dz;
//
//
//    std::vector<geometry_msgs::Pose> waypoints;
//    geometry_msgs::Pose start_pose ;
//    tf::StampedTransform st=getCurrentPose("/base_link","/ee_link");
//    Mat startPoseMat=stampedTransform2Mat(st);
//    start_pose=mat2geometryPose(startPoseMat);
//    geometry_msgs::Pose target_pose3 = start_pose;
//
//     dx=(target_pose.position.x-start_pose.position.x)/2;
//     dy=(target_pose.position.y-start_pose.position.y)/2;
//     dz=(target_pose.position.z-start_pose.position.z)/2;
//
//
//
//    target_pose3.position.x += dx;
//    target_pose3.position.y += dy;
//    target_pose3.position.z += dz;
//
//    waypoints.push_back(target_pose3);  // up and out
//
//    target_pose3.position.x += dx;
//    target_pose3.position.y += dy;
//    target_pose3.position.z += dz;
//
//    waypoints.push_back(target_pose3);  // left
//
//
//    waypoints.push_back(target_pose);
//
//
//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction = group.computeCartesianPath(waypoints,
//                                                 0.01,  // eef_step
//                                                 0.0,   // jump_threshold
//                                                 trajectory);
//    my_plan.trajectory_=trajectory;
//
//    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
//          fraction * 100.0);
//    /* Sleep to give Rviz time to visualize the plan. */
//
//    sleep(5.0);
//   if(fraction==1)
//   {
//       group.execute(my_plan);
//       return true;
//   }
//
}
// 使用关节值来控制机器人
bool movetoPoseJointstate(String jointgroup,Mat jointstate)
{
//if((jointstate.cols==6 && jointstate.rows==1 ))
//{
//        ROS_INFO("JOINTSTATE VALUE IS GOOD ");
// }
// else
//{
//ROS_ERROR("JOINTSTATE VAULE IS WRONG");
//return false;
//}
//
//   std::vector<double> pose;
//    pose.resize(6);
//    pose[0] = jointstate.at<double>(0,0);
//    pose[1] = jointstate.at<double>(0,1);
//    pose[2] =jointstate.at<double>(0,2);
//    pose[3] =jointstate.at<double>(0,3);
//    pose[4] =jointstate.at<double>(0,4);
//    pose[5] =jointstate.at<double>(0,5);
//
//
//    moveit::planning_interface::MoveGroup::Plan my_plan;
//     moveit::planning_interface::MoveGroup Jointgroup("manipulator");
//     Jointgroup.setPoseReferenceFrame("base_link");
//     Jointgroup.setPlannerId("RRTConnectkConfigDefault");
//
//     Jointgroup.setJointValueTarget(pose);
//     bool success =Jointgroup.plan(my_plan);
//     ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
//     sleep(3);
//    if(success)
//    { Jointgroup.execute(my_plan);
//        sleep(5);
//        return true;
//    }
//    else
//    {
//    return false ;
//    }
//
//     ros::spinOnce();

}
// marker = /ar_marker_4   cameraframe = /camera_rgb_optical_frame
 bool findMarkerPoseture(String marker ,String cameraframe,tf::StampedTransform *result)
 {
     tf::TransformListener listener;
     tf::StampedTransform transform;
     ros::Time 	timeFirst,timeSecond;
     ros::Rate rate(10);
     for(int i=0;i<15;i++){
        ros::spinOnce();
          try{

                listener.waitForTransform(cameraframe,marker,  ros::Time(0), ros::Duration(2));
                listener.lookupTransform(cameraframe,marker, ros::Time(0), transform);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
              }
            if(i==0)
            {
                   timeFirst=transform.stamp_;
                   timeSecond=transform.stamp_;
                  rate.sleep();
                   continue;
            }
            else{

                    timeSecond=transform.stamp_;
                   if(timeFirst!=timeSecond)
                       {
                        //  ROS_INFO("AR marker poseture is good");
                          result->setData(transform);
                          return  true;
                       }
                    rate.sleep();
                   }
      }

//the ar poseture is  wrong

     ROS_INFO("AR marker poseture is missing");
      return false;
 }

 bool findBoardPoseture(String marker ,String cameraframe,tf::StampedTransform *result)
 {
     tf::TransformListener listener;
     tf::StampedTransform transform;
     ros::spinOnce();
          try{
                listener.waitForTransform(cameraframe,marker,  ros::Time(0), ros::Duration(2));
                listener.lookupTransform(cameraframe,marker, ros::Time(0), transform);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                return false;
              }

                        //  ROS_INFO("AR marker poseture is good");
             result->setData(transform);
            return  true;

 }

 Mat  createEndRotation(double angle_,int divisor,int degree)
 {
  double PI=3.1415926;
  double angle=angle_/180*PI;
  float x,y,z;
  x=0;
  y=cos(PI*2/divisor*degree);
  z=sin(PI*2/divisor*degree);
  double r;
  r=x*x+y*y+z*z;
  Mat direction(1,3,CV_64F);
  direction.at<double>(0,0)=x/sqrt(r)*angle;
  direction.at<double>(0,1)=y/sqrt(r)*angle;
  direction.at<double>(0,2)=z/sqrt(r)*angle;

  Mat rotation;
  Rodrigues(direction,rotation);
  Mat transform=Mat::zeros(4,4,CV_64F);
 transform.at<double>(0,0)=rotation.at<double>(0,0);
 transform.at<double>(0,1)=rotation.at<double>(0,1);
 transform.at<double>(0,2)=rotation.at<double>(0,2);

 transform.at<double>(1,0)=rotation.at<double>(1,0);
 transform.at<double>(1,1)=rotation.at<double>(1,1);
 transform.at<double>(1,2)=rotation.at<double>(1,2);

 transform.at<double>(2,0)=rotation.at<double>(2,0);
 transform.at<double>(2,1)=rotation.at<double>(2,1);
 transform.at<double>(2,2)=rotation.at<double>(2,2);
 transform.at<double>(3,3)=1;
 return  transform;
 }

 Mat stampedTransform2Mat(tf::StampedTransform st)
 {
    Mat transform=Mat::zeros(4,4,CV_64F);
    double x,y,z,qx,qy,qz ,qw;
    x=st.getOrigin().x();
    y=st.getOrigin().y();
    z=st.getOrigin().z();
    qx=st.getRotation().getAxis().x();
    qy=st.getRotation().getAxis().y();
    qz=st.getRotation().getAxis().z();
    qw=st.getRotation().getW();
    double  norm;
    norm=sqrt(1.0-qw*qw);
    double normold;
    normold=sqrt(qx*qx+qy*qy+qz*qz);
     qx=qx/normold*norm;
     qy=qy/normold*norm;
     qz=qz/normold*norm;
     Mat quat=Mat::zeros(1,4,CV_64F); //change to standard quat
     quat.at<double>(0,0)=-qw;
     quat.at<double>(0,1)=qx;
     quat.at<double>(0,2)=qy;
     quat.at<double>(0,3)=qz;
     Mat rotation=Mat::zeros(4,4,CV_64F);
     rotation =quat2dcm(quat);
     transform.at<double>(0,0)=rotation.at<double>(0,0);
     transform.at<double>(0,1)=rotation.at<double>(0,1);
     transform.at<double>(0,2)=rotation.at<double>(0,2);

     transform.at<double>(1,0)=rotation.at<double>(1,0);
     transform.at<double>(1,1)=rotation.at<double>(1,1);
     transform.at<double>(1,2)=rotation.at<double>(1,2);

     transform.at<double>(2,0)=rotation.at<double>(2,0);
     transform.at<double>(2,1)=rotation.at<double>(2,1);
     transform.at<double>(2,2)=rotation.at<double>(2,2);

     transform.at<double>(0,3)=x*1000; //  use mm
     transform.at<double>(1,3)=y*1000;
     transform.at<double>(2,3)=z*1000;
     transform.at<double>(3,3)=1;
     return transform;
 }

 tf::StampedTransform  getCurrentPose(String  referenceframe,String targetframe)
 {
     tf::TransformListener listener;
     tf::StampedTransform transform;
     ros::Rate rate(10);

          try{

                listener.waitForTransform(referenceframe,targetframe, ros::Time(0), ros::Duration(1));
                listener.lookupTransform(referenceframe,targetframe, ros::Time(0), transform);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
              }

        return transform;

 }

 geometry_msgs::Pose  mat2geometryPose(Mat transform)   //mm -m
 {
      geometry_msgs::Pose resultPose;

      Mat rotation=transform(Range(0,3),Range(0,3));
     // cout<<rotation<<endl;
     Mat  quat =dcm2quat(rotation );
    //  cout<<quat<<endl;
      resultPose.orientation.w = -quat.at<double>(0,0);
      resultPose.orientation.x = quat.at<double>(0,1);
      resultPose.orientation.y = quat.at<double>(0,2);
      resultPose.orientation.z = quat.at<double>(0,3);

      resultPose.position.x = transform.at<double>(0,3)*0.001;
      resultPose.position.y = transform.at<double>(1,3)*0.001;
      resultPose.position.z = transform.at<double>(2,3)*0.001;

      return   resultPose;

 }

 ImagePoint  projectPoint(CAMERA_INTRINSIC_PARAMETERS camera, SpacialPoint point,Mat transform)
 {
     ImagePoint imagepoint;
 //initial point and camera matrix
     Mat   HomoPoint(4,1,CV_64F);

     HomoPoint.at<double>(0,0)=point.x;
     HomoPoint.at<double>(0,1)=point.y;
     HomoPoint.at<double>(0,2)=point.z;
     HomoPoint.at<double>(0,3)=1;

     Mat camera_matrix(3,3,CV_64F);

     camera_matrix.at<double>(0,0)=camera.fx;
     camera_matrix.at<double>(0,1)=0.0;
     camera_matrix.at<double>(0,2)=camera.cx;

     camera_matrix.at<double>(1,0)=0.0;
     camera_matrix.at<double>(1,1)=camera.fy;
     camera_matrix.at<double>(1,2)=camera.cy;

     camera_matrix.at<double>(2,0)=0.0;
     camera_matrix.at<double>(2,1)=0.0;
     camera_matrix.at<double>(2,2)=1.0;

     //calculate
     Mat temp1(3,1,CV_64F);
     double x,y;
     temp1=transform*HomoPoint;  //tranform to current camera coordinates
     Mat temp2(3,1,CV_64F);
     temp2.at<double>(0,0)=temp1.at<double>(0,0)/temp1.at<double>(0,2);
     temp2.at<double>(0,1)=temp1.at<double>(0,1)/temp1.at<double>(0,2);
     temp2.at<double>(0,2)=1.0;
     Mat temp3(3,1,CV_64F);

     temp3=camera_matrix*temp2;
     x=temp3.at<double>(0,0);
     y=temp3.at<double>(0,1);
     imagepoint.x=x;
     imagepoint.y=y;

     return imagepoint;

 }

 Mat calculate_R(int i,int j,Extrinsic_Parameter C_i,Extrinsic_Parameter C_j,Extrinsic_Parameter D_i,Extrinsic_Parameter D_j)
 {
      Mat temp1(3,3,CV_64F);
      Mat temp2(3,3,CV_64F);
      Mat  R;
      Mat tempcrossd,tempcrossc;
      tempcrossc=C_i.k.cross(C_j.k);
      tempcrossd=D_i.k.cross(D_j.k);
      temp1.at<double>(0,0)=D_i.k.at<double>(0,0);
      temp1.at<double>(1,0)=D_i.k.at<double>(1,0);
      temp1.at<double>(2,0)=D_i.k.at<double>(2,0);

      temp1.at<double>(0,1)=D_j.k.at<double>(0,0);
      temp1.at<double>(1,1)=D_j.k.at<double>(1,0);
      temp1.at<double>(2,1)=D_j.k.at<double>(2,0);

      temp1.at<double>(0,2)=tempcrossd.at<double>(0,0);
      temp1.at<double>(1,2)=tempcrossd.at<double>(1,0);
      temp1.at<double>(2,2)=tempcrossd.at<double>(2,0);

      temp2.at<double>(0,0)=C_i.k.at<double>(0,0);
      temp2.at<double>(1,0)=C_i.k.at<double>(1,0);
      temp2.at<double>(2,0)=C_i.k.at<double>(2,0);

      temp2.at<double>(0,1)=C_j.k.at<double>(0,0);
      temp2.at<double>(1,1)=C_j.k.at<double>(1,0);
      temp2.at<double>(2,1)=C_j.k.at<double>(2,0);
           temp2.at<double>(0,2)=tempcrossc.at<double>(0,0);
           temp2.at<double>(1,2)=tempcrossc.at<double>(1,0);
           temp2.at<double>(2,2)=tempcrossc.at<double>(2,0);
      R=temp1*temp2.inv();
      return R;
 }

 Mat calculate_T(Mat R,int i, int j,Extrinsic_Parameter C_i,Extrinsic_Parameter C_j,Extrinsic_Parameter D_i,Extrinsic_Parameter D_j)
 {
     Mat  c=R*C_i.t-D_i.t;
     Mat  d=R*C_j.t-D_j.t;
     Mat  temprodri;
     Rodrigues(D_i.k,temprodri);
     Mat  a=temprodri-Mat::eye(3,3,CV_64F);
     Rodrigues(D_j.k,temprodri);
     Mat  b=temprodri-Mat::eye(3,3,CV_64F);
     Mat h(6,3,CV_64F);
     for(int i=0;i<3;i++)
        {
         for(int j=0;j<3;j++)
         {
            h.at<double>(i,j)=a.at<double>(i,j);
         }
     }
     for(int i=0;i<3;i++)
        {

         for(int j=0;j<3;j++)
         {
            h.at<double>(i+3,j)=b.at<double>(i,j);
         }
     }

     Mat y(6,1,CV_64F);;
     y(Range(0,3),Range(0,1))=c;
     y(Range(3,6),Range(0,1))=d;
     for(int i=0;i<3;i++)
     {
         y.at<double>(i,0)=c.at<double>(i,0);
     }
     for(int i=0;i<3;i++)
     {
         y.at<double>(i+3,0)=d.at<double>(i,0);
     }

     Mat hreverse(3,6,CV_64F);
     for(int i=0;i<6;i++)
     {
         for(int j=0;j<3;j++)
         {
             hreverse.at<double>(j,i)=h.at<double>(i,j);
         }
     }
     Mat temp;
     temp=hreverse*h;
     Mat T=temp.inv()*hreverse*y;
      return T;

 }
 // resultRow 1*7
 Mat resultRow2Mat(Mat resultRow){
 Mat quat(1,4,CV_64F);
 quat.at<double>(0,0)=resultRow.at<double>(0,0);
 quat.at<double>(0,1)=resultRow.at<double>(0,1);
 quat.at<double>(0,2)=resultRow.at<double>(0,2);
 quat.at<double>(0,3)=resultRow.at<double>(0,3);
 Mat t(1,3,CV_64F);


 t.at<double>(0,0)=resultRow.at<double>(0,4);
 t.at<double>(0,1)=resultRow.at<double>(0,5);
 t.at<double>(0,2)=resultRow.at<double>(0,6);
 cout<<quat<<endl;
 cout<<t<<endl;
 Mat trans=buildTransformMatrix_QT(quat,t);
 return trans;

 }

Mat createQRextinsic(double angle,double distance,int divisor,int degree)
 {
     double PI=3.1415926;
     double angle_=angle/180*PI;
     double x,y,z;
     double tx,ty,tz;
     double p=distance*sin(angle_);

     x=cos(PI*2/divisor*degree);
     y=sin(PI*2/divisor*degree);
     z=0;
     double r;
     r=x*x+y*y+z*z;
     Mat direction(1,3,CV_64F);
     direction.at<double>(0,0)=x/sqrt(r)*(angle_-PI);
     direction.at<double>(0,1)=y/sqrt(r)*(angle_-PI);
     direction.at<double>(0,2)=z/sqrt(r)*(angle_-PI);

     Mat rotation;
     Rodrigues(direction,rotation);

     tx=p*cos(2*PI/divisor*degree-PI/2);
     ty=p*sin(2*PI/divisor*degree-PI/2);
     tz=distance;

     Mat transform=Mat::zeros(4,4,CV_64F);
     transform.at<double>(0,0)=rotation.at<double>(0,0);
     transform.at<double>(0,1)=rotation.at<double>(0,1);
     transform.at<double>(0,2)=rotation.at<double>(0,2);

    transform.at<double>(1,0)=rotation.at<double>(1,0);
    transform.at<double>(1,1)=rotation.at<double>(1,1);
    transform.at<double>(1,2)=rotation.at<double>(1,2);

    transform.at<double>(2,0)=rotation.at<double>(2,0);
    transform.at<double>(2,1)=rotation.at<double>(2,1);
    transform.at<double>(2,2)=rotation.at<double>(2,2);
    transform.at<double>(3,3)=1;
    transform.at<double>(0,3)=tx;
    transform.at<double>(1,3)=ty;
    transform.at<double>(2,3)=tz;
    return  transform;
 }

Mat createBoardextrinsic(double angle,double distance,int divisor,int degree,int firstPose)
{
    double PI=3.1415926;
    double angle_=angle/180.0*PI;
    double angle45=30.0/180.0*PI;
    double x,y,z;
    double tx,ty,tz;
    double p=distance*tan(angle_);
    Mat transR(3,1,CV_64F);
    transR.at<double>(0,0)=0;
    transR.at<double>(0,1)=0;
    transR.at<double>(0,2)=(1-firstPose)*PI/2;
    Mat transRotation;
    Rodrigues(transR,transRotation);
    Mat trans0(4,4,CV_64F);
    trans0.at<double>(0,0)=transRotation.at<double>(0,0);
    trans0.at<double>(0,1)=transRotation.at<double>(0,1);
    trans0.at<double>(0,2)=transRotation.at<double>(0,2);
    trans0.at<double>(0,3)=0;

    trans0.at<double>(1,0)=transRotation.at<double>(1,0);
    trans0.at<double>(1,1)=transRotation.at<double>(1,1);
    trans0.at<double>(1,2)=transRotation.at<double>(1,2);
    trans0.at<double>(1,3)=0;

    trans0.at<double>(2,0)=transRotation.at<double>(2,0);
    trans0.at<double>(2,1)=transRotation.at<double>(2,1);
    trans0.at<double>(2,2)=transRotation.at<double>(2,2);
    trans0.at<double>(2,3)=0;

    trans0.at<double>(3,0)=0;
    trans0.at<double>(3,1)=0;
    trans0.at<double>(3,2)=0;
    trans0.at<double>(3,3)=1;

    x=cos(PI*2/divisor*degree);
    y=sin(PI*2/divisor*degree);
    z=0;
    double r;
    r=x*x+y*y+z*z;
    Mat direction(1,3,CV_64F);
    direction.at<double>(0,0)=x/sqrt(r)*angle45;
    direction.at<double>(0,1)=y/sqrt(r)*angle45;
    direction.at<double>(0,2)=z/sqrt(r)*angle45;

    Mat rotation;
    Rodrigues(direction,rotation);

    tx=p*cos(2*PI/divisor*degree-PI/2);
    ty=p*sin(2*PI/divisor*degree-PI/2);
    tz=distance;

    Mat transform=Mat::zeros(4,4,CV_64F);
    transform.at<double>(0,0)=rotation.at<double>(0,0); 
    transform.at<double>(0,1)=rotation.at<double>(0,1);
    transform.at<double>(0,2)=rotation.at<double>(0,2);

   transform.at<double>(1,0)=rotation.at<double>(1,0);
   transform.at<double>(1,1)=rotation.at<double>(1,1);
   transform.at<double>(1,2)=rotation.at<double>(1,2);

   transform.at<double>(2,0)=rotation.at<double>(2,0);
   transform.at<double>(2,1)=rotation.at<double>(2,1);
   transform.at<double>(2,2)=rotation.at<double>(2,2);
   transform.at<double>(3,3)=1;
   transform.at<double>(0,3)=tx;
   transform.at<double>(1,3)=ty;
   transform.at<double>(2,3)=tz;
   Mat result=trans0*transform;
   return  result;

}
//生成最后的base_link到camera_link的指令
void base_link2camera_link(Mat transbase2camera_rbg_optical_frame)
{
Mat transrgb2cameralink(4,4,CV_64F);
transrgb2cameralink.at<double>(0,0)=0;
transrgb2cameralink.at<double>(0,1)=0;
transrgb2cameralink.at<double>(0,2)=1;
transrgb2cameralink.at<double>(0,3)=0;

transrgb2cameralink.at<double>(1,0)=-1;
transrgb2cameralink.at<double>(1,1)=0;
transrgb2cameralink.at<double>(1,2)=0;
transrgb2cameralink.at<double>(1,3)=-45;

transrgb2cameralink.at<double>(2,0)=0;
transrgb2cameralink.at<double>(2,1)=-1;
transrgb2cameralink.at<double>(2,2)=0;
transrgb2cameralink.at<double>(2,3)=0;

transrgb2cameralink.at<double>(3,0)=0;
transrgb2cameralink.at<double>(3,1)=0;
transrgb2cameralink.at<double>(3,2)=0;
transrgb2cameralink.at<double>(3,3)=1;

Mat transbase2cameralink=transbase2camera_rbg_optical_frame*(transrgb2cameralink.inv());
Mat dcm=transbase2cameralink(Range(0,3),Range(0,3));
Mat quat =dcm2quat(dcm);
Mat t=findTFromTrans(transbase2cameralink);

ROS_INFO("please run :");
double x,y,z ,qx,qy,qz,qw;

x=0.001*t.at<double>(0,0);
y=0.001*t.at<double>(1,0);
z=0.001*t.at<double>(2,0);
qx=quat.at<double>(0,1);
qy=quat.at<double>(0,2);
qz=quat.at<double>(0,3);
qw=-quat.at<double>(0,0);
ROS_INFO("rosrun tf static_transform_publisher  %f  %f  %f  %f  %f  %f  %f    /base_link /camera_link 40",
    x,y,z,qx,qy,qz,qw);

}

void ee_link2camera_link(Mat transbase2camera_rbg_optical_frame)
{Mat transrgb2cameralink(4,4,CV_64F);
transrgb2cameralink.at<double>(0,0)=0;
transrgb2cameralink.at<double>(0,1)=0;
transrgb2cameralink.at<double>(0,2)=1;
transrgb2cameralink.at<double>(0,3)=0;

transrgb2cameralink.at<double>(1,0)=-1;
transrgb2cameralink.at<double>(1,1)=0;
transrgb2cameralink.at<double>(1,2)=0;
transrgb2cameralink.at<double>(1,3)=-45;

transrgb2cameralink.at<double>(2,0)=0;
transrgb2cameralink.at<double>(2,1)=-1;
transrgb2cameralink.at<double>(2,2)=0;
transrgb2cameralink.at<double>(2,3)=0;

transrgb2cameralink.at<double>(3,0)=0;
transrgb2cameralink.at<double>(3,1)=0;
transrgb2cameralink.at<double>(3,2)=0;
transrgb2cameralink.at<double>(3,3)=1;

Mat transbase2cameralink=transbase2camera_rbg_optical_frame*(transrgb2cameralink.inv());
Mat dcm=transbase2cameralink(Range(0,3),Range(0,3));
Mat quat =dcm2quat(dcm);
Mat t=findTFromTrans(transbase2cameralink);

ROS_INFO("please run :");
double x,y,z ,qx,qy,qz,qw;

x=0.001*t.at<double>(0,0);
y=0.001*t.at<double>(1,0);
z=0.001*t.at<double>(2,0);
qx=quat.at<double>(0,1);
qy=quat.at<double>(0,2);
qz=quat.at<double>(0,3);
qw=-quat.at<double>(0,0);
ROS_INFO("rosrun tf static_transform_publisher  %f  %f  %f  %f  %f  %f  %f    /ee_link /camera_link 40",
    x,y,z,qx,qy,qz,qw);
}
