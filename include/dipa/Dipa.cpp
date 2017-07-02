/*
 * Dipa.cpp
 *
 *  Created on: Jul 1, 2017
 *      Author: kevin
 */

#include <dipa/Dipa.h>

Dipa::Dipa() {
	//ros::NodeHandle nh;


}

Dipa::~Dipa() {
	/*if(kdtree != NULL)
	{
		delete kdtree;
	}*/

	if(tf_listener != NULL)
	{
		delete tf_listener;
	}
}

/*void Dipa::setupKDTree()
{
	if(kdtree != NULL)
	{
		delete kdtree;
		kdtree = 0;
	}
	ROS_DEBUG("setting up kdtree");
	kdtree = new cv::flann::Index(cv::Mat(detected_corners).reshape(1), cv::flann::KDTreeIndexParams());
	ROS_DEBUG("tree setup");
}*/

void Dipa::findClosestPoints(Matches& model)
{
	/*
	 * vector<Point2f> pointsForSearch; //Insert all 2D points to this vector
flann::KDTreeIndexParams indexParams;
flann::Index kdtree(Mat(pointsForSearch).reshape(1), indexParams);
vector<float> query;
query.push_back(pnt.x); //Insert the 2D point we need to find neighbours to the query
query.push_back(pnt.y); //Insert the 2D point we need to find neighbours to the query
vector<int> indices;
vector<float> dists;
kdtree.radiusSearch(query, indices, dists, range, numOfPoints);
	 */

	cv::flann::Index tree(cv::Mat(detected_corners).reshape(1), cv::flann::KDTreeIndexParams());

	ROS_DEBUG("finding nearest neighbors");
	double maxRadius = sqrt(this->image_size.width * this->image_size.width + this->image_size.height * this->image_size.height);

	for(auto& e : model.matches)
	{
		std::vector<float> query;
		query.push_back(e.obj_px.x);
		query.push_back(e.obj_px.y);

		std::vector<int> indexes;
		std::vector<float> dists;

		tree.knnSearch(query, indexes, dists, 4);

		int best = 0;
		double min = DBL_MAX;

		for(auto j : indexes)
		{
			e.measurement = cv::Point2d(detected_corners.at(j).x, detected_corners.at(j).y);

			double temp = e.computePixelNorm();

			if(temp < min)
			{
				best = j;
				min = temp;
			}
		}

		e.measurement = cv::Point2d(detected_corners.at(best).x, detected_corners.at(best).y);
		e.pixelNorm = best;
		//ROS_DEBUG_STREAM("norm: " << e.getPixelNorm());
	}

	ROS_DEBUG("neighbors found");
}

void Dipa::tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec){
	cv::Mat_<float> R = (cv::Mat_<float>(3, 3) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(),
			tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(),
			tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z());

	cv::Rodrigues(R, rvec);

	tvec = (cv::Mat_<float>(3, 1) << tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());
}

tf::Transform Dipa::rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec){
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

	return trans;
}

/*
 * runs iterative closest point algorithm modified to work with 2d to 3d correspondences.
 * takes a tf transform representing the transform from the world coordinate frame to the camera coordinate frame
 * this transform should be the current best guess of the transform
 *
 * returns the optimized pose which fits the corner model the best
 */
void Dipa::runICP(tf::Transform w2c_guess)
{
	//set up the renderer with the current K and size
	this->renderer.setSize(this->image_size);
	this->renderer.setIntrinsic(this->image_K);

	cv::Mat rvec, tvec; // form the rvec and tvec for the error minimization through solvepnp

	this->tf2rvecAndtvec(w2c_guess.inverse(), tvec, rvec); // set the rvec and tvec to the current best guesses inverse (C2W);

	for(int i = 0; i < MAX_ITERATIONS; i++)
	{
		this->renderer.setC2W(this->rvecAndtvec2tf(tvec, rvec)); // the the renderer's current pose
		Matches matches = this->renderer.renderGridCorners(); // render the corners into this frame given our current guess

		this->findClosestPoints(matches); // find the closest points between the model and the observation corners

		// now we minimize the photometric error between our known model and our observations using the correspondences we have just guessed
		cv::solvePnP(matches.getObjectInOrder(), matches.getMeasurementsInOrder(), this->image_K, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE); // use the current guess to help convergence


		//TODO determine if converged

#if SUPER_DEBUG
		cv::Mat blank = cv::Mat::zeros(this->image_size, CV_8UC3);
		blank = matches.draw(blank, this->detected_corners);
		cv::imshow("render", blank);
		ros::Duration dur(1);
		dur.sleep();
#endif

	}


}
