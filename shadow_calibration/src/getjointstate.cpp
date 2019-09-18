#include"calibrationbase.h"
#include <sensor_msgs/JointState.h>
using namespace std;
using namespace cv;
char jointstatepath[100];


class JointStateRecorder
{
  ros::NodeHandle nh_;

//  ros::NodeHandle it_;

  ros::Subscriber sub ;

public:
    JointStateRecorder()
//    : it_(nh_)
  {
    sub = nh_.subscribe("joint_states", 1, &JointStateRecorder::JointstateCallback,this);
   namedWindow("shadow");
   count =0;
   image=Mat::zeros(150,200,CV_64F);
   imshow("shadow",image);
  }

 int  count ;
 Mat image;
  ~JointStateRecorder()
  {
   destroyWindow("shadow");
  }
 void JointstateCallback(const sensor_msgs::JointState& jointstate)
  {
   char  key=waitKey(30);
   if((int)key==10)
   {
      cout<<"store success"<<endl;
      count=count+1;
      Mat jointstateMat(count,6,CV_64F);
        if(count!=1)
        {
        FileStorage fs1(jointstatepath,FileStorage::READ);

        Mat temp;
        fs1["jointstate"]>>temp;

        for(int i=0;i<count-1;i++)
            {
            for(int j=0;j<6;j++)
            {
               jointstateMat.at<double>(i,j)=temp.at<double>(i,j);
            }
            }
        fs1.release();
         }
        FileStorage fs(jointstatepath,FileStorage::WRITE);

        jointstateMat.at<double>(count-1,0)=(double )jointstate.position[0];
         jointstateMat.at<double>(count-1,1)=(double )jointstate.position[1];
         jointstateMat.at<double>(count-1,2)=(double )jointstate.position[2];
         jointstateMat.at<double>(count-1,3)=(double )jointstate.position[3];
        jointstateMat.at<double>(count-1,4)=(double )jointstate.position[4];
         jointstateMat.at<double>(count-1,5)=(double )jointstate.position[5];
        fs<<"jointstate"<<jointstateMat;
        fs.release();
   }
 }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "getjointstate");
    ros::start();
    cout<<"please input jointstate file path    example  : /home/yourname/jointstate.xml"<<endl;
    scanf("%s",jointstatepath);

    JointStateRecorder  jointstate;
    ros::spin();


    return 0;
}
