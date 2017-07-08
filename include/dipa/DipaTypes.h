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

	double computePixelNorm()
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

	cv::Mat draw(cv::Mat in, std::vector<cv::Point2f> detect)
	{
		for(auto e : detect)
		{
			cv::drawMarker(in, e, cv::Scalar(255, 0, 0), cv::MARKER_DIAMOND, 4);
		}

		for(auto e : matches)
		{

			cv::drawMarker(in, e.obj_px, cv::Scalar(255, 255, 0), cv::MARKER_SQUARE, 4);
			//cv::drawMarker(in, e.measurement, cv::Scalar(0, 255, 0), cv::MARKER_STAR, 4);
			cv::line(in, e.obj_px, e.measurement, cv::Scalar(255, 255, 255));
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

	double sumErrors()
	{
		double error = 0;
		for(auto e : matches){
			error += e.pixelNorm;
		}

		return error;
	}
};

struct DipaState{

private:
	tf::Vector3 omega; // rad/s
	tf::Vector3 vel; // m/s in base frame

	tf::Transform current_best_w2b;
	ros::Time current_best_t;

	tf::Transform last_best_w2b;
	ros::Time last_best_t;

	bool twist_set;
	bool current_pose_set;
	bool last_pose_set;
public:

	DipaState()
{
		twist_set = false;
		current_pose_set = false;
		last_pose_set = false;
}

	void updatePose(tf::Transform trans, ros::Time t)
	{
		if(current_pose_set) // we can now set the last pose
		{
			last_pose_set = true;
			last_best_t = current_best_t;
			last_best_w2b = current_best_w2b;
		}

		current_best_t = t;
		current_best_w2b = trans;

		if(current_pose_set && last_pose_set)
		{
			//numerically derive the velocities
			tf::Transform delta = current_best_w2b * last_best_w2b.inverse();

			double dt = (current_best_t - last_best_t).toSec();

			ROS_ASSERT(dt > 0);

			double r, p, y;
			delta.getBasis().getRPY(r, p, y);
			omega.setX(r);
			omega.setY(p);
			omega.setZ(y);
			omega /= dt;

			vel = (current_best_w2b.getBasis().inverse() * delta.getOrigin()) / dt; // this transforms the delta into the base frame

			twist_set = true;
		}

		current_pose_set = true;
	}

	tf::Transform predict(ros::Time new_t)
	{
		ROS_ASSERT(twist_set && current_pose_set);

		double dt = (new_t - current_best_t).toSec();

		ROS_ASSERT(dt > 0);

		//form update transform
		tf::Transform delta;

		tf::Vector3 dtheta = omega * dt;

		tf::Matrix3x3 rot;
		rot.setRPY(dtheta.x(), dtheta.y(), dtheta.z());

		delta.setBasis(rot);
		delta.setOrigin(current_best_w2b.getBasis() * (vel*dt)); //creates a base frame delta then transforms it into the world frame

		return current_best_w2b * delta; // convolve the current best pose estimate
	}

	bool currentPoseSet(){return current_pose_set;}
	bool twistSet(){return twist_set;}

	tf::Transform getCurrentBestPose(){ROS_ASSERT(current_pose_set); return current_best_w2b;}
	ros::Time getCurrentBestPoseStamp(){ROS_ASSERT(current_pose_set); return current_best_t;}
	tf::Vector3 getBaseFrameVelocity(){ROS_ASSERT(twist_set); return vel;}
	tf::Vector3 getBaseFrameOmega(){ROS_ASSERT(twist_set); return omega;}


};


#endif /* DIPA_INCLUDE_DIPA_DIPATYPES_H_ */
