# autonomous car simulation using ROS, OpenCV, PCL
a car is driving in a simulated environment autonomously, adjusting it's behaviour
to the surroundings, 


## installation requirements

ROS Kinetic and CMake is necessary for this repository

## packages

### detector
this package implements pedestrian detection using the OpenCV library.
it contains a node, which subscibes to an image topic,
and a publisher, which published the human detections on the 

### pcl_solution
this package implements obstacle detection using the PCL library.
it contains a node, which subscibes to the /point_cloud topic and a publisher,
which publishes the detections.	
in processing the point cloud, first the ground plane is removed with
PCL's SACSegmentation, then the obstacles are extracted using Eucledia cluster 
extraction with PCL functions.

### control
this package implements the control of the autonomous vehicle. 
it contains a node, which subsctibes to the pcl_solution and opencv_solution
nodes topics. It avoids obstacles by adjusting its steering based on the 
pcl detections and it stops, when a pedestrian gests too close.

car simulation is accessible at https://github.com/osrf/car_demo.
