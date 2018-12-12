#include <ros/ros.h>
#include <prius_msgs/Control.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <math.h>
#include "signal.h"


namespace control
{
	class Control
	{
	private:
		bool stop;
	public:
		// constructor, deconstructor
		Control();
		~Control();
		// initialize functions, subscribers and publisher
		void initialize(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
		// callback functions: based on the detection boxes, adjust the throttle and steering parameters
		void pclSolCb(vision_msgs::Detection3DArray const & detections);
		void opencvSolCb(vision_msgs::Detection2DArray const & detections);
		// pass the throttle and steering messages to /prius_msg/Control
		void drive(std_msgs::Header const & header);
		// running the control node
		bool run(void);

	private:
		// shutdown the pclSolCb in person.world to aviod overwriting parameters
		bool shutdown(void);
		// throttle and steer parameters
		float throttle_param_ = 1.0;
		float steer_param_ = 0.0;
		// subscriers and publisher
		ros::Subscriber opencvSol_sub_;
		ros::Subscriber pclSol_sub_;
		ros::Publisher prius_pub_;
	};

}
