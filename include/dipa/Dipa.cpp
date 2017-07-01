/*
 * Dipa.cpp
 *
 *  Created on: Jul 1, 2017
 *      Author: kevin
 */

#include <dipa/Dipa.h>

Dipa::Dipa() {
	ros::NodeHandle nh;


}

Dipa::~Dipa() {
	// TODO Auto-generated destructor stub
}

void Dipa::setupKDTree()
{
	indexParams = cv::flann::KDTreeIndexParams();
	kdtree = cv::flann::Index(cv::Mat(detected_corners).reshape(1), indexParams);
}

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
}
