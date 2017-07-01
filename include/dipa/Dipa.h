/*
 * Dipa.h
 *
 *  Created on: Jul 1, 2017
 *      Author: kevin
 */

#ifndef DIPA_INCLUDE_DIPA_DIPA_H_
#define DIPA_INCLUDE_DIPA_DIPA_H_

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

#include <dipa/DipaTypes.h>

class Dipa {
public:

	tf::TransformListener* tf_listener;

	cv::Size image_size;
	cv::Mat_<float> image_K;

	std::vector<cv::Point2f> detected_corners;

	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree;

	Dipa();
	virtual ~Dipa();

	void run(){
		ros::spin();
	}

	void setupKDTree();

	void findClosestPoints(Matches& model);
};

#endif /* DIPA_INCLUDE_DIPA_DIPA_H_ */
