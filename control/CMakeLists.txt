cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  prius_msgs
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
)
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp  sensor_msgs std_msgs  prius_msgs  vision_msgs)

include_directories(include
	${catkin_INCLUDE_DIRS})

add_library(control
	src/control.cpp
	src/control_node.cpp
)

target_link_libraries(control
    ${catkin_LIBRARIES}
    )

add_dependencies(control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_executable(control_node
  src/control_node.cpp)
add_dependencies(control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(control_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
