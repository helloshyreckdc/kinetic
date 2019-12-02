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
bool learn = true;

void frameCallback(const sensor_msgs::ImageConstPtr& image){
    boost::mutex::scoped_lock(lock_);
    Irgb= visp_bridge::toVispImageRGBa(*image);
    vpImageConvert::convert(Irgb,I);
}

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam = visp_bridge::toVispCameraParameters(*cam_info);
    std::cout << cam << std::endl;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "blob_tracking");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/rgb/image_raw",1,frameCallback);
    ros::Subscriber cam_sub = nh.subscribe("/camera/rgb/camera_info",1,camInfoCallback);

    ros::Duration(2).sleep();

    std::cout << "test" << std::endl;

    ros::spinOnce();
    vpDisplayX d(I, 0, 0, "camera view");
    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDot2 blob;


    if (learn) {
        // Learn the characteristics of the blob to auto detect
        blob.setGraphics(true);
        blob.setGraphicsThickness(1);
        blob.initTracking(I);
        blob.track(I);
        std::cout << "Blob characteristics: " << std::endl;
        std::cout << " width : " << blob.getWidth() << std::endl;
        std::cout << " height: " << blob.getHeight() << std::endl;
#if VISP_VERSION_INT > VP_VERSION_INT(2, 7, 0)
        std::cout << " area: " << blob.getArea() << std::endl;
#endif
        std::cout << " gray level min: " << blob.getGrayLevelMin() << std::endl;
        std::cout << " gray level max: " << blob.getGrayLevelMax() << std::endl;
        std::cout << " grayLevelPrecision: " << blob.getGrayLevelPrecision() << std::endl;
        std::cout << " sizePrecision: " << blob.getSizePrecision() << std::endl;
        std::cout << " ellipsoidShapePrecision: " << blob.getEllipsoidShapePrecision() << std::endl;
    }
    else {
        // Set blob characteristics for the auto detection
        blob.setWidth(50);
        blob.setHeight(50);
#if VISP_VERSION_INT > VP_VERSION_INT(2, 7, 0)
        blob.setArea(1700);
#endif
        blob.setGrayLevelMin(0);
        blob.setGrayLevelMax(30);
        blob.setGrayLevelPrecision(0.8);
        blob.setSizePrecision(0.65);
        blob.setEllipsoidShapePrecision(0.65);
    }
    std::list<vpDot2> blob_list;
    blob.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), blob_list);
    if (learn) {
        // The blob that is tracked by initTracking() is not in the list of auto
        // detected blobs We add it:
        blob_list.push_back(blob);
    }
    std::cout << "Number of auto detected blob: " << blob_list.size() << std::endl;
    std::cout << "A click to exit..." << std::endl;
    while (ros::ok()) {
        ros::spinOnce();
        vpDisplay::display(I);
        for (std::list<vpDot2>::iterator it = blob_list.begin(); it != blob_list.end(); ++it) {
            (*it).setGraphics(true);
            (*it).setGraphicsThickness(3);
            (*it).track(I);
        }
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false))
            break;
        vpTime::wait(40);
    }

}
