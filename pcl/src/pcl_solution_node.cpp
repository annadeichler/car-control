#include "pcl_solution.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
	//init node
	ros::init(argc, argv, "pcl_solution_node");
	//create nodehandles
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");
	//create control driver
	pcl_solution::PclSolution pclSolution;
	//initialize control
	pclSolution.initialize(nh, priv_nh);
	//spin ROS
	pclSolution.run();
	//shutdown ROS
	ros::shutdown();
}

