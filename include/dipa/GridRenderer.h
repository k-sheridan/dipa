/*
 * GridRenderer.h
 *
 *  Created on: Jun 18, 2017
 *      Author: pauvsi
 */

#ifndef DIPA_INCLUDE_DIPA_GRIDRENDERER_H_
#define DIPA_INCLUDE_DIPA_GRIDRENDERER_H_

#include <ros/ros.h>

#include <GL/glew.h>
#include <GL/glut.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/core/opengl.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#define RENDER_DEBUG = true


class GridRenderer {
public:

	struct Quad{
		cv::Vec3i color;
		cv::Point3d v1;
		cv::Point3d v2;
		cv::Point3d v3;
		cv::Point3d v4;
	};

	std::vector<Quad> grid;

	int grid_size;
	double grid_spacing;
	double inner_line_thickness;
	double outer_line_thickness;

	cv::Vec3i WHITE, RED, GREEN;

	cv::Mat_<float> K;
	cv::Size size;
	cv::Mat_<float> w2c;
	cv::Mat_<float> c2w;

	GridRenderer();
	virtual ~GridRenderer();

	void setIntrinsic(cv::Mat_<float> K);

	void setSize(cv::Size sz){
		size = sz;
	}

	cv::Mat_<float> tf2cv(tf::Transform tf);

	void setC2W(tf::Transform tf)
	{
		c2w = tf2cv(tf);
		w2c = tf2cv(tf.inverse());
	}

	void setW2C(tf::Transform tf)
	{
		w2c = tf2cv(tf);
		c2w = tf2cv(tf.inverse());
	}

	void generateGrid();

	void setColors(cv::Vec3i w, cv::Vec3i g, cv::Vec3i r);

	cv::Mat renderGrid();


};

#endif /* MANTIS_INCLUDE_MANTIS3_GRIDRENDERER_H_ */
