//
// Created by shyreckdc on 19-7-31.
// now, this code is used to move the robot to the origin of demonstration.
// In the future, this code would be used to grasp an object and then go to origin(according to object relative frame)

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string>


using namespace std;
double wrench[6];


void sub_forceCB(const geometry_msgs::WrenchStamped msg){

	wrench[0] = msg.wrench.force.x;
	wrench[1] = msg.wrench.force.y;
	wrench[2] = msg.wrench.force.z;
	wrench[3] = msg.wrench.torque.x;
	wrench[4] = msg.wrench.torque.y;
	wrench[5] = msg.wrench.torque.z;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "average_force_repub");

	ros::NodeHandle node;

	ros::Publisher averaged_force_pub = node.advertise<geometry_msgs::WrenchStamped>("/averaged_raw_force",10);
	ros::Publisher averaged_calibrated_force_pub = node.advertise<geometry_msgs::WrenchStamped>("/averaged_calibrated_force",10);
	ros::Subscriber raw_force_sub = node.subscribe("/netft_data",10,sub_forceCB);

	geometry_msgs::WrenchStamped averaged_wrench;
	geometry_msgs::WrenchStamped averaged_calibrated_wrench;

	double sum_wrench[6];  // to store and calculate average
	double calibrated_wrench[6];
	// old matrix
	//    double calibration_matrix[6][6] ={{1.0135, 0.0208, 0.0009, 0.0086,-0.1756,-0.0220},
	//                                      {-0.0147, 1.0157, 0.0033, 0.1353,-0.0232, 0.0216},
	//                                      {-0.0011, 0.0009, 0.9959, 0.0741,-0.0005, 0.0004},
	//                                      {-0.0002, 0.0002,-0.0006, 1.0158, 0.0165, 0.0024},
	//                                      {-0.0006,-0.0002, 0.0001,-0.0138, 1.0123, 0.0010},
	//                                      {0.0003,-0.0008,-0.0005, 0.0041,-0.0041, 1.0049 }};

	// // 20200820 calibrated
	// double calibration_matrix[6][6] ={{1.016627265566103 ,  0.014411077248261 , -0.012203194508094 ,  0.047406376888210 , -0.140715327051013,  -0.019061551597761},
	// 	{	-0.012737197196829,   1.013682790016704,   0.008597716718625,   0.061854111721263,  -0.019306778005564,   0.025681449396807},
	// 	{	0.007949317115998 ,  0.010213844458555 ,  0.987704543222232 ,  0.118107391999277 ,  0.014861183265961 ,  0.004075417780819},
	// 	{	0.000536821273572 ,  0.000187844536160 ,  0.000738265209536 ,  1.006463388032173 ,  0.011456210616513 ,  0.000172966333659},
	// 	{	-0.000541815370621,  -0.000012928447483,  -0.000376068534747,  -0.014404839766347,   1.012018256800562,   0.000374958534684},
	// 	{	0.000076808610657 , -0.000626957841388 ,  0.000770690747346 , -0.003582270388252 , -0.000396185574385 ,  1.008102226415717}};

	// 20210421 calibrated
	double calibration_matrix[6][6] ={{1.023615932278978,   0.026827312092062,  -0.002421367768393,   0.056960463961674,  -0.196445216868041,   0.152605079386957},
		{	-0.017158705342187,   1.009936531905829,  -0.011925687639387,   0.158656225800054,  -0.005457017955836,   0.012339786173757},
		{	0.007070510920667,  0.009473700184022,   0.994939176982555,   0.005803648326034,   0.045346587157599,   0.007463856670341},
		{	0.000757015176408,  0.001301816685492,  -0.000719538776279,   1.020570524534400,  -0.002200856202894,   0.000055014208959},
		{	-0.001508341335907,   0.000081579069661,  -0.000195987469931,   0.003992353976365,   1.025118934187676,  -0.005718208253823}, 
		{	0.001120504656105, -0.002066436767296,  -0.000063257396200,  -0.001996624271116,   0.013975202461202,   1.018649382775834}};

		// initialize
		for(int i=0;i<6;i++){
			sum_wrench[i] = 0;
		}

	for(int i=0;i<6;i++){
		calibrated_wrench[i] = 0;
	}

	ros::Duration(2).sleep();

	ros::Rate rate(500);

	int count = 0;

	int lower_rate_ratio = 10;

	while(node.ok()){

		ros::spinOnce();

		for(int i=0;i<6;i++){
			sum_wrench[i] += wrench[i];
		}

		++count;

		if(count == lower_rate_ratio){
			for(int i=0;i<6;i++){ sum_wrench[i] /= lower_rate_ratio;
			}

			averaged_wrench.wrench.force.x = sum_wrench[0];
			averaged_wrench.wrench.force.y = sum_wrench[1];
			averaged_wrench.wrench.force.z = sum_wrench[2];
			averaged_wrench.wrench.torque.x = sum_wrench[3];
			averaged_wrench.wrench.torque.y = sum_wrench[4];
			averaged_wrench.wrench.torque.z = sum_wrench[5];

			averaged_force_pub.publish(averaged_wrench);

			for(int i=0;i<6;i++){
				for(int j=0;j<6;j++){
					calibrated_wrench[i] += calibration_matrix[i][j]*sum_wrench[j];
				}
			}

			averaged_calibrated_wrench.wrench.force.x = calibrated_wrench[0];
			averaged_calibrated_wrench.wrench.force.y = calibrated_wrench[1];
			averaged_calibrated_wrench.wrench.force.z = calibrated_wrench[2];
			averaged_calibrated_wrench.wrench.torque.x = calibrated_wrench[3];
			averaged_calibrated_wrench.wrench.torque.y = calibrated_wrench[4];
			averaged_calibrated_wrench.wrench.torque.z = calibrated_wrench[5];

			averaged_calibrated_force_pub.publish(averaged_calibrated_wrench);

			for(int i=0;i<6;i++){
				calibrated_wrench[i] = 0;
			}

			count = 0;

			for(int i=0;i<6;i++){
				sum_wrench[i] = 0;
			}
		}
		rate.sleep();
	}

	return 0;
}
