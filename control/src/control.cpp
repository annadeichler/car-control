#include "control.h"

namespace control
{
// constructor and desctructor
Control::Control(){}
Control::~Control(){}

// initialization function for ros node
void 
Control::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) {

	ROS_INFO("Initializing control solution");
	stop = false;
	//subscriber and publisher
	pclSol_sub_ = nh.subscribe("/pcl_solution_node/detections",1, &Control::pclSolCb, this);
	opencvSol_sub_ = nh.subscribe("/opencv_solution_node/detections",1, &Control::opencvSolCb, this);
	prius_pub_ = nh.advertise<prius_msgs::Control>("/prius", 1000);
}

// callback funtion, adjust the control parameters based on the detection boxes
void 
Control::pclSolCb(vision_msgs::Detection3DArray const & detections) {
	vision_msgs::Detection3DArray detections_list;
	vision_msgs::Detection3DArray filtered_detections;
	vision_msgs::Detection3D  min_det;
    float min_dist = 1000;
    vision_msgs::Detection3D det3D;
	if (!detections.detections.empty()){  
		for (std::size_t i = 0; i < detections.detections.size(); ++i) {
			det3D = detections.detections.at(i);
			float distance = sqrt(pow(det3D.bbox.center.position.x,2.0)+pow(det3D.bbox.center.position.y,2.0));
			if (det3D.bbox.center.position.x > 0 && distance < 4){
				filtered_detections.detections.push_back(det3D);
				if (distance < min_dist){
					min_det = det3D;
					min_dist = distance;
				}
			}
		}
	}
	if (!filtered_detections.detections.empty()){
		if (min_det.bbox.center.position.y >0){
			ros::param::set("steer_param", -1);
		    ros::param::set("throttle_param", 1);
		}
		else if(stop!=true){
			ros::param::set("steer_param", 1);
		    ros::param::set("throttle_param", 1);
		}
	}else if(stop!=true){
		ros::param::set("steer_param", 0);
		ros::param::set("throttle_param", 1);
	}
	drive(detections_list.header);
}

// callback funtion, adjust the control parameters based on the detection boxes
void 
Control::opencvSolCb(vision_msgs::Detection2DArray const & detections){
	vision_msgs::Detection2DArray detections_list;
    vision_msgs::Detection2D det2D;
	if (!detections.detections.empty()){
		for (std::size_t i = 0; i < detections.detections.size(); ++i) {
			det2D = detections.detections.at(i);
			float size = det2D.bbox.size_y*det2D.bbox.size_x;
			if (size > 25000){
				stop = true;
				ros::param::set("steer_param", 0);
				ros::param::set("throttle_param",0);
			}

		}
	}
	
}
// pass the control parameters to /prius_msg
void Control::drive(std_msgs::Header const & header){
	prius_msgs::Control prius_msg;
	ros::param::get("throttle_param", throttle_param_);
	ros::param::get("steer_param", steer_param_);
	prius_msg.shift_gears = prius_msg.FORWARD;
	if(throttle_param_ == 1){
		prius_msg.shift_gears = prius_msg.FORWARD;
		prius_msg.throttle = 1;
	}
	if(throttle_param_ == 0){
		prius_msg.brake = 1;
		prius_msg.throttle = 0;
		prius_msg.shift_gears = prius_msg.NEUTRAL;
	}
	if(steer_param_ == 1){
		prius_msg.steer = 1;
	}else if(steer_param_ == -1){
		prius_msg.steer = -1;
	}else{
		prius_msg.steer = 0;
	}
	prius_pub_.publish(prius_msg);

	if (prius_msg.brake ==1){
		shutdown();
	}
}

// shutdown the pclSolCb in person.world to advoid parameter overwritting
bool Control::shutdown(void){
	ros::shutdown;
	return true;
}

// running the control node
bool Control::run(void){
	ROS_INFO("Running control node");
	if (stop == true){
		std::cout<<"SHUTTING DOWN-----------------"<<std::endl;
		this->shutdown();
		return false;
	}
	ros::spin();
}

}
