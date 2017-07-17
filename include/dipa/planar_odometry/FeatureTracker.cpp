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

void FeatureTracker::updateFeatures(cv::Mat img) {

	std::vector<cv::Point2f> oldPoints = this->state.getPixels2fInOrder();

	ROS_ASSERT(oldPoints.size() > 0);

	//ROS_DEBUG_STREAM_ONCE("got " << oldPoints.size() << " old point2fs from the oldframe which has " << oldFrame.features.size() << " features");
	std::vector<cv::Point2f> newPoints;

	std::vector<uchar> status; // status vector for each point
	cv::Mat error; // error vector for each point

	ROS_DEBUG("before klt");

	cv::calcOpticalFlowPyrLK(this->state.currentImg, img, oldPoints, newPoints,
			status, error, cv::Size(21, 21), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					30, 0.01), 0, KLT_MIN_EIGEN);

	ROS_DEBUG("after klt");

	std::vector<Feature> flowedFeatures;

	int lostFeatures = 0;

	for (int i = 0; i < status.size(); i++) {
		if (status.at(i) == 1) {
			Feature updated_feature = this->state.features.at(i);

			updated_feature.px = newPoints.at(i);

			flowedFeatures.push_back(updated_feature);

		} else {
			lostFeatures++;
		}
	}

	this->state.features = flowedFeatures;

	this->state.currentImg = img; // the new image is now the old image

	ROS_DEBUG_STREAM("VO LOST " << lostFeatures << "FEATURES");

}

bool FeatureTracker::computePose(double& perPixelError) {

	ROS_DEBUG("computing motion");
	ROS_ASSERT(this->state.features.size() >= 4);

	std::vector<cv::Point2d> img_pts;
	std::vector<cv::Point3d> obj_pts;

	img_pts = this->state.getPixelsInOrder();
	obj_pts = this->state.getObjectsInOrder();

	cv::Mat rvec, tvec;

	//set the initial inv w2c guess
	this->tf2rvecAndtvec(this->state.currentPose.inverse(), tvec, rvec);

	cv::solvePnP(obj_pts, img_pts, this->K, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);


	//compute the error for this solution
	std::vector<cv::Point2d> proj;
	cv::projectPoints(obj_pts, rvec, tvec, this->K, cv::noArray(), proj);
	double err = 0;
	for(int i = 0; i < proj.size(); i++)
	{
		double dx = proj[i].x-img_pts[i].x;
		double dy = proj[i].y-img_pts[i].y;
		err += sqrt(dx*dx + dy*dy);
	}

	this->state.ppe = err / (double)proj.size();
	perPixelError = this->state.ppe;

	ROS_DEBUG_STREAM("VO PPE: " << this->state.ppe);

	//get the updated transform back
	this->state.currentPose = this->rvecAndtvec2tf(tvec, rvec).inverse(); // invert back to w2c

	ROS_DEBUG_STREAM("VO w2c: " << this->state.currentPose.getOrigin().x()  << ", " << this->state.currentPose.getOrigin().y() << ", " << this->state.currentPose.getOrigin().z());

	ROS_DEBUG("done computing motion");

	return true;
}

void FeatureTracker::updatePose(tf::Transform w2c, ros::Time t) {
	this->state.currentPose = w2c; // set the new pose

	ROS_INFO("GRID HAS ALIGNED: UPDATING THE VO POSE AND OBJECT POSITIONS TO CORRECT FOR DRIFT" );

	this->state.updateObjectPositions(this->K); // update the object positions to eliminate the drift

	//set the time at this update
	this->state.time_at_last_realignment = t;
}

/*
 * get more features after updating the pose
 */
void FeatureTracker::replenishFeatures(cv::Mat in) {
	//add more features if needed
	cv::Mat img;
	cv::GaussianBlur(in, img, cv::Size(5, 5), FAST_BLUR_SIGMA);

	if (this->state.features.size() < NUM_FEATURES) {
		std::vector<cv::KeyPoint> fast_kp;

		cv::FAST(img, fast_kp, FAST_THRESHOLD, true);

		int needed = NUM_FEATURES - this->state.features.size();

		ROS_DEBUG_STREAM("need " << needed << "more features");

		/*cv::flann::Index tree;

		if (this->state.features.size() > 0) {
			std::vector<cv::Point2f> prev = this->state.getPixels2fInOrder();
			tree = cv::flann::Index(cv::Mat(prev).reshape(1),
					cv::flann::KDTreeIndexParams());
		}*/

		//image which is used to check if a close feature already exists
		cv::Mat checkImg = cv::Mat::zeros(img.size(), CV_8U);
		for(auto e : this->state.features)
		{
			cv::drawMarker(checkImg, e.px, cv::Scalar(255), cv::MARKER_SQUARE, MIN_NEW_FEATURE_DIST * 2, MIN_NEW_FEATURE_DIST);
		}



		for (int i = 0; i < needed && i < fast_kp.size(); i++) {
			/*if (this->state.features.size() > 0) {
				//make sure that this corner is not too close to any old corners
				std::vector<float> query;
				query.push_back(fast_kp.at(i).pt.x);
				query.push_back(fast_kp.at(i).pt.y);

				std::vector<int> indexes;
				std::vector<float> dists;

				tree.knnSearch(query, indexes, dists, 1);

				if (dists.front() < MIN_NEW_FEATURE_DIST) // if this featrue is too close to a already tracked feature skip it
				{
					continue;
				}
			}*/

			//check if there is already a close feature
			if(checkImg.at<unsigned char>(fast_kp.at(i).pt))
			{
				ROS_DEBUG("feature too close to previous feature, not adding");
				needed++; // we need one more now
				continue;
			}

			Feature new_ft;

			new_ft.px = fast_kp.at(i).pt;

			bool valid = new_ft.computeObjectPosition(this->state.currentPose, this->K); // corresponf to a 3d point

			if(valid) // feature is valid add it
			{
				ROS_DEBUG("adding new feature to vo");
				this->state.features.push_back(new_ft);
			}
			else //feature is aff the grid remove it
			{
				needed++; // we still need a new feature
				continue;
			}

		}
	}

	this->state.currentImg = img;

#if SUPER_DEBUG

	cv::Mat copy = img.clone();

	copy = this->draw(copy);

	cv::imshow("vo", copy);
	cv::waitKey(30);

#endif
}


void FeatureTracker::tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec){
	cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(),
			tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(),
			tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z());

	//ROS_DEBUG("setting up tvec and rvec");

	cv::Rodrigues(R, rvec);

	tvec = (cv::Mat_<double>(3, 1) << tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());

	//ROS_DEBUG_STREAM("tvec: " << tvec << "\nrvec: " << rvec);
}

tf::Transform FeatureTracker::rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec){
	//ROS_DEBUG("rvectvec to tf");
	cv::Mat_<double> rot;
	cv::Rodrigues(rvec, rot);
	/*ROS_DEBUG_STREAM("rot: " << rot);
		ROS_DEBUG_STREAM("rvec: " << rvec);*/
	//ROS_DEBUG_STREAM("tvec " << tvec);

	tf::Transform trans;

	trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
	trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

	//ROS_DEBUG_STREAM("rot: " << trans.getRotation().w() << ", " << trans.getRotation().x() << ", " << trans.getRotation().y() << ", " << trans.getRotation().z());
	/*double x, y, z;
		trans.getBasis().getRPY(x, y, z);
		ROS_DEBUG_STREAM("tf rvec " << x <<", "<<y<<", "<<z);*/
	//ROS_DEBUG_STREAM(trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z());

	//ROS_DEBUG("finished");

	return trans;
}
