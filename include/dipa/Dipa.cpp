/*
 * Dipa.cpp
 *
 *  Created on: Jul 1, 2017
 *      Author: kevin
 */

#include <dipa/Dipa.h>

Dipa::Dipa(tf::Transform initial_world_to_base_transform) {
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	//TODO make a camera sub when I have a properly recorded dataset
	image_transport::Subscriber bottom_cam_sub = it.subscribe(BOTTOM_CAMERA_TOPIC, 2, &Dipa::bottomCamCb, this);


	ros::spin(); // go into the main loop;

}

Dipa::~Dipa() {

}
//void Dipa::bottomCamCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr cam)
void Dipa::bottomCamCb(const sensor_msgs::ImageConstPtr& img)
{
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	this->detectFeatures(temp);
}

void Dipa::detectFeatures(cv::Mat img)
{
	cv::Mat canny, blur;
	cv::GaussianBlur(img, blur, CANNY_BLUR_KERNEL, CANNY_BLUR_SIGMA);
	//cv::Canny(blur, canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);

	std::vector<cv::KeyPoint> kp;
	cv::FAST(blur, kp, FAST_THRESHOLD);

	cv::drawKeypoints(blur, kp, img);

	cv::imshow("kp", blur);
	cv::waitKey(30);
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

	//ROS_DEBUG("finding nearest neighbors");
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

	//ROS_DEBUG("neighbors found");
}

void Dipa::tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec){
	cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(),
			tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(),
			tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z());

	//ROS_DEBUG("setting up tvec and rvec");

	cv::Rodrigues(R, rvec);

	tvec = (cv::Mat_<double>(3, 1) << tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());

	//ROS_DEBUG_STREAM("tvec: " << tvec << "\nrvec: " << rvec);
}

tf::Transform Dipa::rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec){
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

/*
 * runs iterative closest point algorithm modified to work with 2d to 3d correspondences.
 * takes a tf transform representing the transform from the world coordinate frame to the camera coordinate frame
 * this transform should be the current best guess of the transform
 *
 * returns the optimized pose which fits the corner model the best
 */
tf::Transform Dipa::runICP(tf::Transform w2c_guess)
{
	//set up the renderer with the current K and size
	this->renderer.setSize(this->image_size);
	this->renderer.setIntrinsic(this->image_K);

	cv::Mat rvec, tvec; // form the rvec and tvec for the error minimization through solvepnp

	this->tf2rvecAndtvec(w2c_guess.inverse(), tvec, rvec); // set the rvec and tvec to the current best guesses inverse (C2W);

	ROS_DEBUG("begining optim");

	//initial setup and sse calculation
	this->renderer.setC2W(this->rvecAndtvec2tf(tvec, rvec)); // the the renderer's current pose
	Matches matches = this->renderer.renderGridCorners(); // render the corners into this frame given our current guess

	this->findClosestPoints(matches); // find the closest points between the model and the observation corners
	double last_sse = matches.sumErrors();
	double current_sse = last_sse;

	for(int i = 0; i < MAX_ITERATIONS; i++)
	{
		// now we minimize the photometric error between our known model and our observations using the correspondences we have just guessed
		cv::solvePnP(matches.getObjectInOrder(), matches.getMeasurementsInOrder(), this->image_K, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE); // use the current guess to help convergence

		// recalculate correspondences and sse
		this->renderer.setC2W(this->rvecAndtvec2tf(tvec, rvec)); // the the renderer's current pose
		matches = this->renderer.renderGridCorners(); // render the corners into this frame given our current guess

		this->findClosestPoints(matches); // find the closest points between the model and the observation corners
		current_sse = matches.sumErrors(); // compute current sse

		ROS_DEBUG_STREAM("current error: " << current_sse);

		if(fabs(current_sse - last_sse) < CONVERGENCE_DELTA){
			ROS_DEBUG("PNP-ICP Converged");
			break;
		}
		else
		{
			last_sse = current_sse; // set the new last sse
		}

#if SUPER_DEBUG
		cv::Mat blank = cv::Mat::zeros(this->image_size, CV_8UC3);
		blank = matches.draw(blank, this->detected_corners);
		cv::imshow("render", blank);
		cv::waitKey(30);
		ros::Duration dur(1);
		dur.sleep();
#endif

	}

	ROS_DEBUG("end optim");

	return this->rvecAndtvec2tf(tvec, rvec).inverse(); // return the w2c guess

}
