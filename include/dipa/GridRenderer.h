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

#define _WHITE cv::Vec3b(255, 255, 255)
#define _RED cv::Vec3b(0, 0, 255)
#define _GREEN cv::Vec3b(0, 255, 0)

#define _FLOOR cv::Vec3b(255, 0, 0)

#define _BACKGROUND cv::Vec3b(0, 0, 0)


class GridRenderer {
public:

	struct Quad{
		cv::Vec3b color;
		std::vector<cv::Point2f> vertices;

		/*
		 * test if point on xy plane is in this quad
		 */
		bool pointInQuad(tf::Vector3 pt)
		{
			return cv::pointPolygonTest(vertices, cv::Point2f(pt.x(), pt.y()), false) >= 0;
		}
	};

	std::deque<Quad> grid;

	int grid_size;
	double grid_spacing;
	double inner_line_thickness;
	double outer_line_thickness;
	double boundary_padding;

	cv::Vec3b WHITE, RED, GREEN;

	cv::Mat_<float> K;
	cv::Size size;
	tf::Transform w2c;
	tf::Transform c2w;

	GridRenderer();
	virtual ~GridRenderer();

	void setIntrinsic(cv::Mat_<float> K);

	void setSize(cv::Size sz){
		size = sz;
	}

	cv::Mat_<float> tf2cv(tf::Transform tf);

	void setC2W(tf::Transform tf)
	{
		c2w = tf;
		w2c = tf.inverse();
	}

	void setW2C(tf::Transform tf)
	{
		w2c = tf;
		c2w = tf.inverse();
	}

	void generateGrid();

	void setColors(cv::Vec3b w, cv::Vec3b g, cv::Vec3b r);

	tf::Vector3 project2XYPlane(cv::Mat_<float> dir, bool& behind);
	cv::Mat renderGridByProjection();


};

#endif /* MANTIS_INCLUDE_MANTIS3_GRIDRENDERER_H_ */
