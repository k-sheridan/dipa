cmake_minimum_required(VERSION 2.8.3)
project(dipa)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
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

add_library(dipaParams include/dipa/DipaParams.h)
set_target_properties(dipaParams PROPERTIES LINKER_LANGUAGE CXX)


add_library(feature_tracker include/dipa/planar_odometry/FeatureTracker.cpp)
target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} dipaParams)

add_library(dipaTypes include/dipa/DipaTypes.h)
set_target_properties(dipaTypes PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(dipaTypes ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} dipaParams)

add_library(dipaGridRenderer include/dipa/GridRenderer.cpp)
target_link_libraries(dipaGridRenderer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY} dipaTypes dipaParams)

add_library(dipa include/dipa/Dipa.cpp)
target_link_libraries(dipa ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} dipaGridRenderer dipaTypes dipaParams feature_tracker)

add_executable(dipa_test test/unit_test.cpp)
target_link_libraries(dipa_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY} dipaGridRenderer dipa dipaParams)

add_executable(dipa_node src/dipa_node.cpp)
target_link_libraries(dipa_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY} dipaGridRenderer dipa dipaParams)

#add_executable(dipa_gl_test test/gl_test.cpp)
#target_link_libraries(dipa_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY} )