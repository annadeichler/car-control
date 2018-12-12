#include "control.h"

int main(int argc, char** argv)
{
	//init node
	ros::init(argc, argv, "control_node");
	//create nodehandles
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");
	//create control driver
	control::Control Control;
	//initialize control
	Control.initialize(nh, priv_nh);
	//spin ROS
	Control.run();
	//shutdown ROS
	ros::shutdown();
}

