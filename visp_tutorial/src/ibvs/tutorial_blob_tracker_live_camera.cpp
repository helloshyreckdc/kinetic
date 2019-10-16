#include <ros/ros.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <boost/thread.hpp>

vpImage<vpRGBa> Irgb;
vpImage<unsigned char> I;
vpCameraParameters cam;
boost::mutex lock_;
bool got_image;

void frameCallback(const sensor_msgs::ImageConstPtr& image){
    //, const sensor_msgs::CameraInfoConstPtr& cam_info){
    boost::mutex::scoped_lock(lock_);
    Irgb= visp_bridge::toVispImageRGBa(*image);
//    cam = visp_bridge::toVispCameraParameters(*cam_info);
    vpImageConvert::convert(Irgb,I);
    got_image = true;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "blob_tracking");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw",1,frameCallback);


    try {


        vpDot2 blob;
        blob.setGraphics(true);
        blob.setGraphicsThickness(2);

        vpImagePoint germ;
        bool init_done = false;
        ros::Duration(2).sleep();
        ros::spinOnce();

//            vpDisplayX d(I, vpDisplay::SCALE_AUTO);
        vpDisplayX d(I, 0, 0, "camera view");
        vpDisplay::setTitle(I, "My image");

        std::cout << "test" << std::endl;

        ros::Rate loop_rate = 30;
        while (ros::ok()) {
            try {
                ros::spinOnce();

                vpDisplay::display(I);
//            vpDisplay::flush(I);


                if (!init_done) {
                    vpDisplay::displayText(I, vpImagePoint(10, 10), "Click in the blob to initialize the tracker",
                                           vpColor::red);
                    if (vpDisplay::getClick(I, germ, true)) {
                        blob.initTracking(I, germ);
                        init_done = true;
                        std::cout << "hello" << std::endl;
                    }
                } else {
                    blob.track(I);
                }

                vpDisplay::flush(I);
//                vpDisplay::getClick(I);
            } catch (...) {
                init_done = false;
            }
            loop_rate.sleep();
        }
    }catch (const vpException &e) {
        std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    }
//#endif
}
