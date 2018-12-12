#include "pcl_object.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/point_cloud.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include "pcl/common/angles.h"
#include <pcl/common/common_headers.h>

#include "pcl/PointIndices.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudPCL;
typedef pcl::PointXYZ PointPCL;

namespace pcl_solution
{
	class PclSolution
	{
	public:
		PclSolution();
		~PclSolution();

		void initialize(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
		void pclCb(const sensor_msgs::PointCloud2ConstPtr msg);
		void EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          std::vector<pclObject> &objects, const std_msgs::Header& header);
		void GroundPlaneRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		                    pcl::PointIndices::Ptr indices,
		                    pcl::ModelCoefficients::Ptr coeff);

		void run();

	private:
		ros::Subscriber pcl_sub_;
		sensor_msgs::Image image_;
		ros::Publisher detections_pub_;
	
	};


}
