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

		tree.knnSearch(query, indexes, dists, 1);

		e.measurement = cv::Point2d(detected_corners.at(indexes.front()).x, detected_corners.at(indexes.front()).y);
		e.pixelNorm = dists.front();
		//ROS_DEBUG_STREAM("norm: " << e.pixelNorm);
	}

	ROS_DEBUG("neighbors found");
}
