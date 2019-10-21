#include <ros/ros.h>
#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <pbd/to_string.h>

vpImage<vpRGBa> Irgb;
vpImage<unsigned char> I;
vpCameraParameters cam;
geometry_msgs::Twist tool0_vel;
std_msgs::String ur_string;

void frameCallback(const sensor_msgs::ImageConstPtr& image){
    Irgb = visp_bridge::toVispImageRGBa(*image);
    vpImageConvert::convert(Irgb,I);
}

void tool0velCallback(const geometry_msgs::Twist& msg){
    std::string s = "speedl(["
                    +to_string(msg.linear.x)+","
                    +to_string(msg.linear.y)+","
                    +to_string(msg.linear.z)+","
                    +to_string(msg.angular.x)+","
                    +to_string(msg.angular.y)+","
                    +to_string(msg.angular.z)
                    +"],0.5,1)";
    ur_string.data = s;
}

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam = visp_bridge::toVispCameraParameters(*cam_info);
}

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
		std::vector<vpImagePoint> *traj_vip)
{
	for (size_t i = 0; i < vip.size(); i++) {
		if (traj_vip[i].size()) {
			// Add the point only if distance with the previous > 1 pixel
			if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
				traj_vip[i].push_back(vip[i]);
			}
		}
		else {
			traj_vip[i].push_back(vip[i]);
		}
	}
	for (size_t i = 0; i < vip.size(); i++) {
		for (size_t j = 1; j < traj_vip[i].size(); j++) {
			vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
		}
	}
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "servo_ibvs");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/sr300/color/image_raw",1,frameCallback);
    ros::Subscriber cam_sub = nh.subscribe("/sr300/color/camera_info",1,camInfoCallback);
    ros::Subscriber visual_servo_vel_sub = nh.subscribe("/visual_servo/ur/velocity",1,tool0velCallback);
    ros::Publisher vel_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript",10);
    ros::Publisher sr300_vel_pub = nh.advertise<geometry_msgs::Twist>("/sr300_vel",10);

    geometry_msgs::Twist sr300_vel;

    //initialization
    tool0_vel.linear.x = 0;
    tool0_vel.linear.y = 0;
    tool0_vel.linear.z = 0;
    tool0_vel.angular.x = 0;
    tool0_vel.angular.y = 0;
    tool0_vel.angular.z = 0;

    ros::Duration(2).sleep();

	double opt_tagSize = 0.100;
	std::string opt_robot_ip = "192.168.1.1";
	std::string opt_eMc_filename = "";
	bool display_tag = true;
	int opt_quad_decimate = 2;
	bool opt_verbose = false;
	bool opt_plot = false;
	bool opt_adaptive_gain = false;
	bool opt_task_sequencing = false;
	double convergence_threshold = 0.00005;

	for (int i = 1; i < argc; i++) {
		if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
			opt_tagSize = std::stod(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
			opt_robot_ip = std::string(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--eMc" && i + 1 < argc) {
			opt_eMc_filename = std::string(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--verbose") {
			opt_verbose = true;
		}
		else if (std::string(argv[i]) == "--plot") {
			opt_plot = true;
		}
		else if (std::string(argv[i]) == "--adaptive_gain") {
			opt_adaptive_gain = true;
		}
		else if (std::string(argv[i]) == "--task_sequencing") {
			opt_task_sequencing = true;
		}
		else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
			opt_quad_decimate = std::stoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--no-convergence-threshold") {
			convergence_threshold = 0.;
		}
		else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
			std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--tag_size <marker size in meter; default " << opt_tagSize << ">] [--eMc <eMc extrinsic file>] "
				<< "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] [--adaptive_gain] [--plot] [--task_sequencing] [--no-convergence-threshold] [--verbose] [--help] [-h]"
				<< "\n";
			return EXIT_SUCCESS;
		}
	}


	try {
        ros::spinOnce();
		// Get camera extrinsics
		vpPoseVector ePc;
		// Set camera extrinsics default values
        ePc[0] = 0.1521501994; ePc[1] = 0.102386358569; ePc[2] = 0.0150548553837;
        ePc[3] = -1.6259; ePc[4] = 0.2122; ePc[5] =-0.2214;

		// If provided, read camera extrinsics from --eMc <file>
		if (!opt_eMc_filename.empty()) {
			ePc.loadYAML(opt_eMc_filename, ePc);
		}
		else {
			std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << "\n";
		}
		vpHomogeneousMatrix eMc(ePc);
		std::cout << "eMc:\n" << eMc << "\n";

		// Get camera intrinsics
		std::cout << "cam:\n" << cam << "\n";


#if defined(VISP_HAVE_X11)
		vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

		vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
		vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
		//vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
		vpDetectorAprilTag detector(tagFamily);
		detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
		detector.setDisplayTag(display_tag);
		detector.setAprilTagQuadDecimate(opt_quad_decimate);

		// Servo
		vpHomogeneousMatrix cdMc, cMo, oMo;

		// Desired pose used to compute the desired features
        vpRotationMatrix rotationMatrix;
        rotationMatrix[0][0] =  1; rotationMatrix[0][1] =  0; rotationMatrix[0][2] =  0;
        rotationMatrix[1][0] =  0; rotationMatrix[1][1] = -1; rotationMatrix[1][2] =  0;
        rotationMatrix[2][0] =  0; rotationMatrix[2][1] =  0; rotationMatrix[2][2] = -1;
        vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, opt_tagSize * 3), rotationMatrix);

		// Create visual features
		std::vector<vpFeaturePoint> p(4), pd(4); // We use 4 points

		// Define 4 3D points corresponding to the CAD model of the Apriltag
		std::vector<vpPoint> point(4);
		point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
		point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
		point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
		point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);

		vpServo task;
		// Add the 4 visual feature points
		for (size_t i = 0; i < p.size(); i++) {
			task.addFeature(p[i], pd[i]);
		}
		task.setServo(vpServo::EYEINHAND_CAMERA);
		task.setInteractionMatrixType(vpServo::CURRENT);

		if (opt_adaptive_gain) {
			vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
			task.setLambda(lambda);
		}
		else {
			task.setLambda(0.5);
		}

		vpPlot *plotter = nullptr;
		int iter_plot = 0;

		if (opt_plot) {
			plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
			plotter->setTitle(0, "Visual features error");
			plotter->setTitle(1, "Camera velocities");
			plotter->initGraph(0, 8);
			plotter->initGraph(1, 6);
			plotter->setLegend(0, 0, "error_feat_p1_x");
			plotter->setLegend(0, 1, "error_feat_p1_y");
			plotter->setLegend(0, 2, "error_feat_p2_x");
			plotter->setLegend(0, 3, "error_feat_p2_y");
			plotter->setLegend(0, 4, "error_feat_p3_x");
			plotter->setLegend(0, 5, "error_feat_p3_y");
			plotter->setLegend(0, 6, "error_feat_p4_x");
			plotter->setLegend(0, 7, "error_feat_p4_y");
			plotter->setLegend(1, 0, "vc_x");
			plotter->setLegend(1, 1, "vc_y");
			plotter->setLegend(1, 2, "vc_z");
			plotter->setLegend(1, 3, "wc_x");
			plotter->setLegend(1, 4, "wc_y");
			plotter->setLegend(1, 5, "wc_z");
		}

		bool final_quit = false;
		bool has_converged = false;
		bool send_velocities = false;
		bool servo_started = false;
		std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

		static double t_init_servo = vpTime::measureTimeMs();


		while (!has_converged && !final_quit) {
			double t_start = vpTime::measureTimeMs();

			ros::spinOnce();

			vpDisplay::display(I);

			std::vector<vpHomogeneousMatrix> cMo_vec;
			detector.detect(I, opt_tagSize, cam, cMo_vec);

			std::stringstream ss;
			ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
			vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

			vpColVector v_c(6);

			// Only one tag is detected
			if (cMo_vec.size() == 1) {
				cMo = cMo_vec[0];

				static bool first_time = true;
				if (first_time) {
					// Introduce security wrt tag positionning in order to avoid PI rotation
					std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
					v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
					for (size_t i = 0; i < 2; i++) {
						v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
					}
					if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
						oMo = v_oMo[0];
					}
					else {
						std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
						oMo = v_oMo[1];   // Introduce PI rotation
					}

					// Compute the desired position of the features from the desired pose
					for (size_t i = 0; i < point.size(); i++) {
						vpColVector cP, p;
						point[i].changeFrame(cdMo * oMo, cP);
						point[i].projection(cP, p);

						pd[i].set_x(p[0]);
						pd[i].set_y(p[1]);
						pd[i].set_Z(cP[2]);
					}
				}

				// Get tag corners
				std::vector<vpImagePoint> corners = detector.getPolygon(0);

				// Update visual features
				for (size_t i = 0; i < corners.size(); i++) {
					// Update the point feature from the tag corners location
					vpFeatureBuilder::create(p[i], cam, corners[i]);
					// Set the feature Z coordinate from the pose
					vpColVector cP;
					point[i].changeFrame(cMo, cP);

					p[i].set_Z(cP[2]);
				}

				if (opt_task_sequencing) {
					if (! servo_started) {
						if (send_velocities) {
							servo_started = true;
						}
						t_init_servo = vpTime::measureTimeMs();
					}
					v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo)/1000.);
				}
				else {
					v_c = task.computeControlLaw();
				}

				// Display the current and desired feature points in the image display
				vpServoDisplay::display(task, cam, I);
				for (size_t i = 0; i < corners.size(); i++) {
					std::stringstream ss;
					ss << i;
					// Display current point indexes
					vpDisplay::displayText(I, corners[i]+vpImagePoint(15, 15), ss.str(), vpColor::red);
					// Display desired point indexes
					vpImagePoint ip;
					vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
					vpDisplay::displayText(I, ip+vpImagePoint(15, 15), ss.str(), vpColor::red);
				}
				if (first_time) {
					traj_corners = new std::vector<vpImagePoint> [corners.size()];
				}
				// Display the trajectory of the points used as features
				display_point_trajectory(I, corners, traj_corners);

				if (opt_plot) {
					plotter->plot(0, iter_plot, task.getError());
					plotter->plot(1, iter_plot, v_c);
					iter_plot++;
				}

				if (opt_verbose) {
					std::cout << "v_c: " << v_c.t() << std::endl;
				}

				double error = task.getError().sumSquare();
				ss.str("");
				ss << "error: " << error;
				vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

				if (opt_verbose)
					std::cout << "error: " << error << std::endl;

				if (error < convergence_threshold) {
					has_converged = true;
					std::cout << "Servo task has converged" << "\n";
					vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
				}
				if (first_time) {
					first_time = false;
				}
			} // end if (cMo_vec.size() == 1)
			else {
				v_c = 0;
			}

			if (!send_velocities) {
				v_c = 0;
			}

            sr300_vel.linear.x = v_c[0];
            sr300_vel.linear.y = v_c[1];
            sr300_vel.linear.z = v_c[2];
            sr300_vel.angular.x = v_c[3];
            sr300_vel.angular.y = v_c[4];
            sr300_vel.angular.z = v_c[5];
            sr300_vel_pub.publish(sr300_vel);

			// Send to the robot
			ros::spinOnce();
			vel_pub.publish(ur_string);

			ss.str("");
			ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
			vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
			vpDisplay::flush(I);

			vpMouseButton::vpMouseButtonType button;
			if (vpDisplay::getClick(I, button, false)) {
				switch (button) {
					case vpMouseButton::button1:
						send_velocities = !send_velocities;
						break;

					case vpMouseButton::button3:
						final_quit = true;
						v_c = 0;
						break;

					default:
						break;
				}
			}
		}
		std::cout << "Stop the robot " << std::endl;
        //send stopl to ur
        ur_string.data = "stopl(1)";

		if (opt_plot && plotter != nullptr) {
			delete plotter;
			plotter = nullptr;
		}

		task.kill();

		if (!final_quit) {
			while (!final_quit) {
			    ros::spinOnce();
				vpDisplay::display(I);

				vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
				vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

				if (vpDisplay::getClick(I, false)) {
					final_quit = true;
				}

				vpDisplay::flush(I);
			}
		}
		if (traj_corners) {
			delete [] traj_corners;
		}
	}
	catch(const vpException &e) {
		std::cout << "ViSP exception: " << e.what() << std::endl;
		std::cout << "Stop the robot " << std::endl;
        //send stopl to ur
        ur_string.data = "stopl(1)";
		return EXIT_FAILURE;
	}
	catch(const std::exception &e) {
		std::cout << "robot exception: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}

