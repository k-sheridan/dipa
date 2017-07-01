/*
 * DipaTypes.h
 *
 *  Created on: Jul 1, 2017
 *      Author: kevin
 */

#ifndef DIPA_INCLUDE_DIPA_DIPATYPES_H_
#define DIPA_INCLUDE_DIPA_DIPATYPES_H_

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp> // OpenCV window I/O
#include <opencv2/imgproc.hpp> // OpenCV image transformations
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

struct Match{
	cv::Point3d obj;
	cv::Point2d obj_px;
	cv::Point2d measurement;
	double pixelNorm;

	Match(){
		pixelNorm = -1;
	}

	double getPixelNorm()
	{

		double dx = obj_px.x - measurement.x;
		double dy = obj_px.y - measurement.y;
		pixelNorm = sqrt(dx*dx+dy*dy);
		return pixelNorm;

	}
};

struct Matches{
	std::vector<Match> matches;

	cv::Mat draw(cv::Mat in)
	{
		for(auto e : matches)
		{
			cv::line(in, e.obj_px, e.measurement, cv::Scalar(255, 255, 255));
			cv::drawMarker(in, e.obj_px, cv::Scalar(255, 255, 0), cv::MARKER_SQUARE, 4);
			cv::drawMarker(in, e.measurement, cv::Scalar(0, 255, 0), cv::MARKER_STAR, 4);
		}

		return in;
	}

	std::vector<cv::Point2d> getMeasurementsInOrder(){
		std::vector<cv::Point2d> z;
		for(auto e : matches)
		{
			z.push_back(e.measurement);
		}
		return z;
	}

	std::vector<cv::Point3d> getObjectInOrder(){
		std::vector<cv::Point3d> z;
		for(auto e : matches)
		{
			z.push_back(e.obj);
		}
		return z;
	}

	std::vector<cv::Point2f> getObjectPixelsInOrder(){
		std::vector<cv::Point2f> z;
		for(auto e : matches)
		{
			//ROS_DEBUG_STREAM(e.obj_px.x << " before");
			z.push_back(cv::Point2f(e.obj_px.x, e.obj_px.y));
			//ROS_DEBUG_STREAM(z.back().x << " after");
		}
		return z;
	}
};


#endif /* DIPA_INCLUDE_DIPA_DIPATYPES_H_ */
