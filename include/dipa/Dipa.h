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

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <dipa/DipaParams.h>

#include <dipa/GridRenderer.h>

#include <dipa/DipaTypes.h>

#include <dipa/planar_odometry/FeatureTracker.h>

class Dipa {
public:

	tf::TransformListener tf_listener;

	GridRenderer renderer;

	FeatureTracker vo;

	cv::Size image_size;
	cv::Mat_<float> image_K;

	bool TRACKING_LOST;

	std::vector<cv::Point2f> detected_corners;

	DipaState state;

	ros::Publisher odom_pub;

#if PUBLISH_INSIGHT
	//insight
	ros::Publisher insight_pub;
	Matches alignment;
#endif

	ros::Subscriber pose_realignment_sub;
	ros::Time time_at_last_realignment; //  the msg stamp of the last pose realignment

	//cv::flann::Index* kdtree;

	Dipa(tf::Transform initial_world_to_base_transform, bool debug=false);
	virtual ~Dipa();

	void run(){
		ros::spin();
	}

	//void bottomCamCb(const sensor_msgs::ImageConstPtr& img);
	void bottomCamCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	//realignment sub
	void realignmentCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

	//void setupKDTree();

	void detectFeatures(cv::Mat img);

	std::vector<cv::Point2f> findLineIntersections(std::vector<cv::Vec2f> lines, cv::Rect boundingBox);

	void findClosestPoints(Matches& model);

	void tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec);

	tf::Transform rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec);

	bool fitsPositionalConstraints(tf::Transform w2c);

	tf::Transform runICP(tf::Transform w2c_guess, double& ppe, bool& pass);

	void publishOdometry();

	void publishInsight(cv::Mat src,  bool grid_aligned);
};

#endif /* DIPA_INCLUDE_DIPA_DIPA_H_ */
