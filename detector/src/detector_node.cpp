#include "detector.h"

int main(int argc, char** argv)
{
	//init node
	ros::init(argc, argv, "detector_node");
	//create nodehandles
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");
	//create control driver
	detector::Detector detector;
	//initialize control
	detector.initialize(nh, priv_nh);
	//spin ROS
	detector.run();
	//shutdown ROS
	ros::shutdown();
}

