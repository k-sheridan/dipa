/*
 * GridRenderer.cpp
 *
 *  Created on: Jun 18, 2017
 *      Author: pauvsi
 */

#include <dipa/GridRenderer.h>

GridRenderer::GridRenderer() {
	WHITE = _WHITE;
	GREEN = _GREEN;
	RED = _RED;

	generateGrid();
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

cv::Mat_<float> GridRenderer::tf2cv(tf::Transform tf)
{
	cv::Mat_<float> out = (cv::Mat_<float>(3, 4) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(), tf.getOrigin().x(),
			tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(), tf.getOrigin().y(),
			tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z(), tf.getOrigin().z());

	return out;
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
			q.vertices.push_back(cv::Point2f( x - ilt, max));
			q.vertices.push_back(cv::Point2f( x + ilt, max));
			q.vertices.push_back(cv::Point2f( x + ilt, min));
			q.vertices.push_back(cv::Point2f( x - ilt, min));
		}
		else
		{
			q.vertices.push_back(cv::Point2f( x - olt, max));
			q.vertices.push_back(cv::Point2f( x + olt, max));
			q.vertices.push_back(cv::Point2f( x + olt, min));
			q.vertices.push_back(cv::Point2f( x - olt, min));
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
			q.vertices.push_back(cv::Point2f(max, y - ilt));
			q.vertices.push_back(cv::Point2f(max, y + ilt));
			q.vertices.push_back(cv::Point2f(min, y + ilt));
			q.vertices.push_back(cv::Point2f(min, y - ilt));
		}

		grid.push_back(q);

		line++;
	}


}

cv::Mat GridRenderer::renderGridByProjection()
{
	cv::Mat result = cv::Mat(size, CV_8UC3);

	cv::Mat_<float> Kinv = K.inv();



	return result;
}



