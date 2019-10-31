#include <ros/ros.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpKeyPoint.h>

#include <sensor_msgs/Image.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

vpImage<vpRGBa> Irgb;
vpImage<unsigned char> I_original;
vpImage<unsigned char> I;
vpCameraParameters cam;

void frameCallback(const sensor_msgs::ImageConstPtr& image){
    Irgb = visp_bridge::toVispImageRGBa(*image);
    vpImageConvert::convert(Irgb,I);
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "matching_keypoint");
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe("/sr300/color/image_raw",1,frameCallback);

    ros::Duration(2).sleep();

    ros::spinOnce();

	const std::string detectorName = "ORB";
	const std::string extractorName = "ORB";
	// Hamming distance must be used with ORB
	const std::string matcherName = "BruteForce-Hamming";
	vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
	vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
	std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;
	vpImage<unsigned char> Idisp;
	Idisp.resize(I.getHeight(), 2 * I.getWidth());
	Idisp.insert(I, vpImagePoint(0, 0));
	Idisp.insert(I, vpImagePoint(0, I.getWidth()));
	vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with ORB keypoints");
	vpDisplay::display(Idisp);
	vpDisplay::flush(Idisp);
	while (ros::ok()){
	    ros::spinOnce();
		Idisp.insert(I, vpImagePoint(0, I.getWidth()));
		vpDisplay::display(Idisp);
		vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()),
				vpColor::white, 2);
		unsigned int nbMatch = keypoint.matchPoint(I);
		std::cout << "Matches=" << nbMatch << std::endl;
		vpImagePoint iPref, iPcur;
		for (unsigned int i = 0; i < nbMatch; i++) {
			keypoint.getMatchedPoints(i, iPref, iPcur);
			vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
		}
		vpDisplay::flush(Idisp);
		if (vpDisplay::getClick(Idisp, false))
			break;
	}
	vpDisplay::getClick(Idisp);
	return 0;
}
