/*
 * GridRenderer.cpp
 *
 *  Created on: Jun 18, 2017
 *      Author: pauvsi
 */

#include <dipa/GridRenderer.h>

GridRenderer::GridRenderer() {

}

GridRenderer::~GridRenderer() {

}

void GridRenderer::setColors(cv::Vec3i w, cv::Vec3i g, cv::Vec3i r)
{
	WHITE = w;
	GREEN = g;
	RED = r;
}

void GridRenderer::setIntrinsic(cv::Mat_<float> K)
{
	this->K = K;
}


void GridRenderer::generateGrid()
{

	double grid_size = 9;
	double ilt=0.02;
	double olt=0.04;
	double grid_spacing = 0.3;
	double min = -(grid_size * grid_spacing / 2);
	double max = (grid_size * grid_spacing / 2);

	int line = 0;
	for(double x = min; x < max + grid_spacing; x += grid_spacing)
	{
		//ROS_DEBUG_STREAM("x: " << x);

		Quad q;
		q.color = WHITE;

		if(line != 0 && line != grid_size)
		{
			q.v1 = cv::Point3d( x - ilt, max, 0);
			q.v2 = cv::Point3d( x + ilt, max, 0);
			q.v3 = cv::Point3d( x + ilt, min, 0);
			q.v4 = cv::Point3d( x - ilt, min, 0);
		}
		else
		{
			q.v1 = cv::Point3d( x - olt, max, 0);
			q.v2 = cv::Point3d( x + olt, max, 0);
			q.v3 = cv::Point3d( x + olt, min, 0);
			q.v4 = cv::Point3d( x - olt, min, 0);
		}

		grid.push_back(q);

		line++;
	}

	line = 0;
	for(double y = min; y < max + grid_spacing; y += grid_spacing)
	{
		//ROS_DEBUG_STREAM("x: " << y);

		Quad q;
		q.color = WHITE;

		if(line != 0 && line != grid_size)
		{
			q.v1 = cv::Point3d(max, y - ilt, 0);
			q.v1 = cv::Point3d(max, y + ilt, 0);
			q.v1 = cv::Point3d(min, y + ilt, 0);
			q.v1 = cv::Point3d(min, y - ilt, 0);
		}

		grid.push_back(q);

		line++;
	}


}

cv::Mat GridRenderer::renderGrid()
{
	cv::Mat result = cv::Mat(size, CV_8UC3);

	return result;
}



