cmake_minimum_required(VERSION 2.8.3)
project(pcl_solution)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp  pcl_ros sensor_msgs std_msgs 
  DEPENDS eigen)

find_package( PCL REQUIRED )
find_package( Eigen REQUIRED )
find_package(Boost COMPONENTS filesystem system)


include_directories(include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS})


add_definitions(${Eigen_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

file(GLOB_RECURSE header_files include/${PROJECT_NAME}/*.h)

add_library(pcl
  src/pcl.cpp
	src/pcl_node.cpp
)

target_link_libraries(pcl
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}

)

add_dependencies(pcl
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(pcl_node
  src/pcl_node.cpp)
add_dependencies(pcl_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pcl_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

