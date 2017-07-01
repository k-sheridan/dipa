/*
 * unit_test.cpp
 *
 *  Created on: Jun 27, 2017
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

cv::Mat first;
bool firstSet = false;

static void showDifference(const cv::Mat& image1, const cv::Mat& image2, const char* title)
{
	cv::Mat img1, img2;
	image1.convertTo(img1, CV_32FC3);
	image2.convertTo(img2, CV_32FC3);
	if(img1.channels() != 1)
		cvtColor(img1, img1, CV_RGB2GRAY);
	if(img2.channels() != 1)
		cvtColor(img2, img2, CV_RGB2GRAY);

	cv::Mat imgDiff;
	img1.copyTo(imgDiff);
	imgDiff -= img2;
	imgDiff /= 2.f;
	imgDiff += 128.f;

	cv::Mat imgSh;
	imgDiff.convertTo(imgSh, CV_8UC3);
	imshow(title, imgSh);
}

void imgcb(const sensor_msgs::ImageConstPtr& img)
{
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	cv::resize(temp, temp, cv::Size(320, 240));

	temp.convertTo(temp, CV_64FC1); // convert to double 1 channel

	if(!firstSet)
	{
		first = temp;
		firstSet = true;
		ROS_DEBUG("set first frame");
		return;
	}

	showDifference(first, temp, "before");
	cv::waitKey(30);

	cv::reg::MapperGradProj mapper = cv::reg::MapperGradProj();
	cv::reg::MapperPyramid mappPyr(mapper);
	cv::Ptr<cv::reg::Map> mapPtr;
	mappPyr.calculate(first, temp, mapPtr);

	//get the result
	cv::reg::MapProjec* mapProj = dynamic_cast<cv::reg::MapProjec*>(mapPtr.get());
	mapProj->normalize();

	cv::Mat dest;
	mapProj->inverseWarp(temp, dest);
	showDifference(first, dest, "fixed");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dipa_test");

	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	ros::Subscriber imgsub = nh.subscribe<sensor_msgs::Image>("m7/camera/image_rect", 2, imgcb);


	tf::Transform w2c1;
	w2c1.setRotation(tf::Quaternion(1, 0, 0, 0));
	//w2c1.setRotation(tf::Quaternion(1/sqrt(2), 1/sqrt(2), 0, 0));
	w2c1.setOrigin(tf::Vector3(0, 0, 3));

	cv::Mat_<float> K = (cv::Mat_<float>(3, 3) << 300, 0, 300, 0, 300, 300, 0, 0, 1);

	GridRenderer gr;

	gr.setW2C(w2c1);
	gr.setSize(cv::Size(600, 600));
	gr.setIntrinsic(K);




	while(ros::ok())
	{
		ROS_DEBUG("draw");
		cv::Mat blank = cv::Mat::zeros(cv::Size(600, 600), CV_8UC3);
		blank = gr.drawCorners(blank, gr.renderGridCorners());
		cv::imshow("render", blank);
		cv::waitKey(30);
	}

	ros::spin();

	return 0;
}

