#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "opencv2/features2d/features2d.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

#include <memory>

#include <boost/thread/recursive_mutex.hpp>

namespace detector
{
	class Detector
	{
	public:
		Detector();
		~Detector();
		void initialize(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
		void imageCb(const  sensor_msgs::ImageConstPtr& ms);
		void get_detections(cv_bridge::CvImageConstPtr& cv_ptr, const cv::Scalar color);
		void check_imFormat(const sensor_msgs::ImageConstPtr& msg);
		cv::Ptr<cv::HOGDescriptor> hog_;

		void run();

	private:
		boost::shared_ptr<image_transport::ImageTransport> it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		ros::Publisher detections_pub_;
		
	};

}
