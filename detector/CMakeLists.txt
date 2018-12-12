cmake_minimum_required(VERSION 2.8.3)
project(detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	std_msgs
	image_transport
	cv_bridge
	vision_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp  sensor_msgs std_msgs  image_transport cv_bridge )

find_package(OpenCV 3.3.0 REQUIRED)
find_package(Boost COMPONENTS filesystem system)


include_directories(include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS})

add_library(detector
	src/detector.cpp
	src/detector_node.cpp
)

target_link_libraries(detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_dependencies(detector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_executable(detector_node
  src/detector_node.cpp)
add_dependencies(detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(detector_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )