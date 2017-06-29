/*
 * unit_test.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: kevin
 */

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/core/opengl.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapper.hpp"
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

void imgcb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

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

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dipa_test");

	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber quadDetectCameraSub = it.subscribeCamera("m7/camera/image_rect", 2, imgcb);



	ros::spin();

	return 0;
}

