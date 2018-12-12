#include "pcl_solution.h"
#include <vision_msgs/Detection3DArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using pcl_conversions::fromPCL;

namespace pcl_solution
{
	// constructor and desctructor
	PclSolution::PclSolution(){}
	PclSolution::~PclSolution(){}

	// initialization function for ros node
	void 
	PclSolution::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) {
		ROS_INFO("Initializing pcl solution");
		pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2ConstPtr>("/point_cloud",1000, &PclSolution::pclCb,this);
		detections_pub_ = nh.advertise<vision_msgs::Detection3DArray>("/pcl_solution_node/detections",1);
		
	}

	// function for segmenting ground plane 
	void
	PclSolution::GroundPlaneRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr ground_indices,
	                    pcl::ModelCoefficients::Ptr coeff)
	{
		pcl::PointIndices indices_internal;
		pcl::SACSegmentation<PointPCL> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.3);
		seg.setInputCloud(cloud);
		// make sure axis is right
		Eigen::Vector3f axis;
		axis << 0, 0, 1;
		seg.setAxis(axis);
		seg.setEpsAngle(pcl::deg2rad(10.0));
		seg.segment(indices_internal, *coeff);
		double distance_above_plane;
		ros::param::param("distance_above_plane", distance_above_plane, 0.3);
		// ignores points above the plane
		for (size_t i = 0; i < cloud->size(); ++i) {
			const PointPCL& pt = cloud->points[i];
			float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
			        coeff->values[2] * pt.z + coeff->values[3];
			if (val <= distance_above_plane) {
				ground_indices->indices.push_back(i);
			}
		}

		if (ground_indices->indices.size() == 0) {
			ROS_ERROR("Unable to find surface.");
			return;
		}	
	}

	// function for detecting clusters
	void
	PclSolution::EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	                          std::vector<pclObject> &objects, const std_msgs::Header& pcl_header)
	{

		pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
		GroundPlaneRemoval(cloud, ground_inliers, coeff);

		PointCloudPCL::Ptr cloud_out(new PointCloudPCL());
		pcl::ExtractIndices<PointPCL> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ground_inliers);
		extract.setNegative(false);
		extract.filter(*cloud_out);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::ExtractIndices<PointPCL> extract_cluster;

		pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
		extract_cluster.setInputCloud(cloud);
		extract_cluster.setIndices(ground_inliers);
		extract_cluster.setNegative(true);
		extract_cluster.filter(above_surface_indices->indices);

		// add eucledian clustering parameters to ros parameter server
		double cluster_tolerance;
		int min_cluster_size, max_cluster_size;
		ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.5);
		ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
		ros::param::param("ec_max_cluster_size", max_cluster_size, 25000);

		//get obstacles clusters in form of cluster point indices
		pcl::EuclideanClusterExtraction<PointPCL> euclid;
		euclid.setInputCloud(cloud);
		euclid.setIndices(above_surface_indices);
		euclid.setClusterTolerance(cluster_tolerance);
		euclid.setMinClusterSize(min_cluster_size);
		euclid.setMaxClusterSize(max_cluster_size);
		euclid.extract(cluster_indices);
		size_t min_size = std::numeric_limits<size_t>::max();
		size_t max_size = std::numeric_limits<size_t>::min();
		for (size_t i = 0; i < cluster_indices.size(); ++i) {
			size_t cluster_size = (cluster_indices)[i].indices.size();
			if (cluster_size < min_size) {
					min_size = cluster_size;
				}
			if (cluster_size > max_size) {
		 		 max_size = cluster_size;
			}
		}

		// publishing cluster detections as bounding boxes
		vision_msgs::Detection3DArray detections_list;
		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			vision_msgs::Detection3D det3D;
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
	 			//add a point to the end of the existing vector
	 			cloud_cluster->points.push_back (cloud->points[*pit]); //*
	 		}
	 		Eigen::Vector4f centroid;
	 		Eigen::Vector4f min_pt, max_pt;
	 		pcl::compute3DCentroid (*cloud_cluster, centroid);
	 		pcl::getMinMax3D (*cloud_cluster, min_pt, max_pt);

		    vision_msgs::BoundingBox3D bbox;
		    bbox.center.position.x = centroid[0];
		    bbox.center.position.y = centroid[1];
		    bbox.center.position.z = centroid[2];
		    bbox.size.x = max_pt.x() - min_pt.x();
		    bbox.size.y = max_pt.y() - min_pt.y();
		    bbox.size.z = max_pt.z() - min_pt.z();
		    det3D.bbox = bbox;
		    det3D.header = pcl_header;
		    detections_list.detections.push_back(det3D);
		}
		detections_list.header = pcl_header;
		detections_pub_.publish(detections_list);
	}

	// point cloud call back function
	void
	PclSolution::pclCb(const sensor_msgs::PointCloud2ConstPtr msg){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *cloud);
		std_msgs::Header header_pcl = msg->header;
		if(cloud->isOrganized())
		{
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);		
		}
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud);
		vg.setLeafSize (0.05f, 0.05f, 0.05f);
		vg.filter (*cloud_filtered);

		std::vector<pclObject> objects;
		EuclideanClusterExtraction(cloud, objects, header_pcl);
	}


	void PclSolution::run(){
		ROS_INFO("Running pcl_solution node");
		ros::spin();
	}

}
