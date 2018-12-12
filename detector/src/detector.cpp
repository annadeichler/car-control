#include "detector.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection2DArray.h>

#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <algorithm>

namespace detector
{
// constructor and desctructor
Detector::Detector(){}
Detector::~Detector(){}

using namespace cv;

// initialization function for ros node
void 
Detector::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) {

	ROS_INFO("Initializing opencv solution");

	// shared_ptr within a class to hold the NodeHandle object, so the  NodeHandle can be used in other class member
	it_ =boost::make_shared<image_transport::ImageTransport>(nh);
	int queue_size_ = 1;
	image_sub_ = it_->subscribe("/prius/front_camera/image_raw", 1, &Detector::imageCb, this);
	image_pub_ = it_->advertise("/opencv_solution_node/visual",1);
	detections_pub_ = nh.advertise<vision_msgs::Detection2DArray>("/opencv_solution_node/detections",1);
	hog_ = new cv::HOGDescriptor();

}

// ROS image message callback function
void 
Detector::imageCb(const sensor_msgs::ImageConstPtr& msg) {

	check_imFormat(msg);
	// convert image
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

	get_detections(cv_ptr, cv::Scalar(0,0,255));

}

// processing ROS image to openCV format, detecting and publishing pedestrians information 
void Detector::get_detections(cv_bridge::CvImageConstPtr& cv_ptr,  const cv::Scalar color)
 {

 	cv::Mat img = cv_ptr->image;
	cv::Mat tmp = img.clone();
	vision_msgs::Detection2D det2D;
	vision_msgs::BoundingBox2D bbox;
	vision_msgs::Detection2DArray detections_list;

	std::vector<cv::Rect> detections;
	int hitThreshold(0);
	Size winStride(Size(8, 8));

	hog_->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	hog_->detectMultiScale(img, detections, 0,  Size(8, 8), Size(32,32), 1.05, 2);


	double alpha = 0.8;
	for (const auto&det : detections) {
		//draw rectangle on img copy
		cv::rectangle(tmp, det, color);
		// 2DArray msg setup
		bbox.size_x = det.width;
		bbox.size_y = det.height;
		bbox.center.x = det.x + 0.5 * det.width;
		bbox.center.y = det.y + 0.5 * det.height;
		det2D.bbox = bbox;
		detections_list.detections.push_back(det2D);

	}
	cv::addWeighted(img, 1-alpha, tmp, alpha, 0, img);
	
	// covert to image message
	cv_bridge::CvImage img_msg;
	img_msg.header = cv_ptr->header;
	img_msg.encoding = "bgr8";
	img_msg.image = tmp;
	image_pub_.publish(img_msg.toImageMsg());;

	detections_pub_.publish(detections_list);

}

// running the opencv_solution node
void Detector::run(){
	ROS_INFO("Running opencv_solution node");
	ros::spin();
}

// image conversion helper function
void 
Detector::check_imFormat(const sensor_msgs::ImageConstPtr& msg)
{
	// not essential - just checking encoding
	cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat conversion_mat;
	try
	{
			cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
			conversion_mat = cv_ptr->image;

	}catch(cv_bridge::Exception& e)	{
		try
		{
			cv_ptr = cv_bridge::toCvShare(msg);
			if (msg->encoding == "8UC3")
			{
				// rgb
					conversion_mat = cv_ptr->image;		
			}else {
				ROS_INFO("imageCb() could not convert image from '%s'  to rgb8 (%s)", msg->encoding.c_str(), e.what());
					return;	
			} 
		}catch(cv_bridge::Exception& e){
				ROS_INFO("imageCb() while trying to convert image from '%s'  to rgb8 (%s)", msg->encoding.c_str(), e.what());
		}
	}

}

}


