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

#define SUPER_DEBUG false

#define MAX_ITERATIONS 20

#define CONVERGENCE_DELTA 0.1

#define FAST_THRESHOLD 10
#define CANNY_HYSTERESIS 20
#define CANNY_BLUR_SIGMA 2
#define CANNY_BLUR_KERNEL cv::Size(0, 0)

#define BOTTOM_CAMERA_TOPIC "/m7/camera/image_rect"
#define BOTTOM_CAMERA_FRAME "bottomCamera"

#define BASE_FRAME "base_link"

#define WORLD_FRAME "world"

#include <dipa/GridRenderer.h>

#include <dipa/DipaTypes.h>

class Dipa {
public:

	tf::TransformListener tf_listener;

	GridRenderer renderer;

	cv::Size image_size;
	cv::Mat_<float> image_K;

	std::vector<cv::Point2f> detected_corners;

	DipaState state;

	//cv::flann::Index* kdtree;

	Dipa(tf::Transform initial_world_to_base_transform);
	virtual ~Dipa();

	void run(){
		ros::spin();
	}

	void bottomCamCb(const sensor_msgs::ImageConstPtr& img);
	//void bottomCamCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr cam);

	//void setupKDTree();

	void detectFeatures(cv::Mat img);

	void findClosestPoints(Matches& model);

	void tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec);

	tf::Transform rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec);

	tf::Transform runICP(tf::Transform w2c_guess);
};

#endif /* DIPA_INCLUDE_DIPA_DIPA_H_ */
