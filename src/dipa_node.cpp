/*
 * dipa_node.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#include <ros/ros.h>

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp> // OpenCV window I/O
#include <opencv2/imgproc.hpp> // OpenCV image transformations
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mapshift.hpp"
#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradshift.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mappergradsimilar.hpp"
#include "opencv2/reg/mappergradaffine.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>

#include <dipa/GridRenderer.h>
#include <dipa/Dipa.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "dipa_test");

	double x, y, z, r, p, yaw;

	ros::param::param<double>("initial_x", x, 0.25);
	ros::param::param<double>("initial_y", y, -0.75);
	ros::param::param<double>("initial_z", z, 1.0);
	ros::param::param<double>("initial_roll", r, 180);
	ros::param::param<double>("initial_pitch", p, 0);
	ros::param::param<double>("initial_yaw", yaw, 0);

	tf::Vector3 origin = tf::Vector3(x, y, z);
	tf::Matrix3x3 rot;
	rot.setRPY(r, p, yaw);


	//Dipa dipa(tf::Transform(rot, origin));
	Dipa dipa(tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(0, 0, 1)));

	return 0;
}
