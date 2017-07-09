/*
 * FeatureTracker.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#include <dipa/planar_odometry/FeatureTracker.h>

FeatureTracker::FeatureTracker() {
	// TODO Auto-generated constructor stub

}

FeatureTracker::~FeatureTracker() {
	// TODO Auto-generated destructor stub
}

void FeatureTracker::updateFeatures(cv::Mat img){

/*
	cv::calcOpticalFlowPyrLK(oldFrame.image, newFrame.image, oldPoints, newPoints, status, error, cv::Size(21, 21), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 0, this->MIN_EIGEN_VALUE);*/

}

tf::Transform FeatureTracker::computePose(double& perPixelError){

}

void FeatureTracker::updatePose(tf::Transform w2c){

}
