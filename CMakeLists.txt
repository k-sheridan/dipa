cmake_minimum_required(VERSION 2.8.3)
project(dipa)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/mantis
	${catkin_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}
 	${GLUT_INCLUDE_DIRS}
 	${GLEW_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
# CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS opencv
)

add_library(dipaGridRenderer include/dipa/GridRenderer.cpp)
target_link_libraries(dipaGridRenderer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY})
