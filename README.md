# Autonomous car simulation using ROS, OpenCV, PCL
A car is driving in a simulated environment autonomously, adjusting it's behaviour
to the surroundings. 


## Installation requirements
ROS Kinetic and CMake is necessary for this repository.

## packages

### Image detector module / opencv_solution
This package implements pedestrian detection using the OpenCV library.
It contains a node, which subscibes to an image topic,
and a publisher, which published the human detections. 

### PCL module / pcl_solution
This package implements obstacle detection using the PCL library.
It contains a node, which subscibes to the /point_cloud topic and a publisher,
which publishes the detections.	
In processing the point cloud, first the ground plane is removed with
PCL's SACSegmentation, then the obstacles are extracted using Eucledia cluster 
extraction with PCL functions.

### Control module
This package implements the control of the autonomous vehicle. 
It contains a node, which subsctibes to the pcl_solution and opencv_solution
nodes topics. It avoids obstacles by adjusting its steering based on the 
pcl detections and it stops, when a pedestrian gests too close.

The car simulation package used in this project is accessible at https://github.com/osrf/car_demo.
