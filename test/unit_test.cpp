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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <dipa/GridRenderer.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "dipa_test");


	tf::Transform w2c1;
	w2c1.setRotation(tf::Quaternion(1/sqrt(3), 1/sqrt(3), 0, 1/sqrt(3)));
	//w2c1.setRotation(tf::Quaternion(1/sqrt(2), 1/sqrt(2), 0, 0));
	w2c1.setOrigin(tf::Vector3(0, 0, 1));

	cv::Mat_<float> K = (cv::Mat_<float>(3, 3) << 300, 0, 300, 0, 300, 300, 0, 0, 1);

	GridRenderer gr;

	gr.setColors(cv::Vec3i(255, 255, 255), cv::Vec3i(0, 255, 0), cv::Vec3i(255, 0, 0));

	gr.grid_size = 10;
	gr.grid_spacing = 0.32;
	gr.inner_line_thickness = 0.04;
	gr.outer_line_thickness = 0.08;

	gr.setW2C(w2c1);
	gr.setSize(cv::Size(600, 600));
	gr.setIntrinsic(K);

	/*
	cv::Mat test = gr.renderGrid();

	cv::imshow("test", test);
	cv::waitKey(30);*/

	while(ros::ok())
	{

	}

	return 0;
}

