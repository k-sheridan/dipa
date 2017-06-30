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

	boundary_padding = 1;

	generateGrid();
}

GridRenderer::~GridRenderer() {

}

void GridRenderer::setColors(cv::Vec3b w, cv::Vec3b g, cv::Vec3b r)
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

	Quad floor;
	floor.color = _FLOOR;
	floor.vertices.push_back(cv::Point2f( max + boundary_padding, max + boundary_padding));
	floor.vertices.push_back(cv::Point2f( min - boundary_padding, max + boundary_padding));
	floor.vertices.push_back(cv::Point2f( min - boundary_padding, min - boundary_padding));
	floor.vertices.push_back(cv::Point2f( max + boundary_padding, min - boundary_padding));

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

		grid.push_front(q);

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

		grid.push_front(q);

		line++;
	}


}

tf::Vector3 GridRenderer::project2XYPlane(cv::Mat_<float> dir, bool& behind)
{
	tf::Vector3 dir_world = (w2c * tf::Vector3(dir(0), dir(1), dir(2))) - w2c.getOrigin();

	// z = z0 + dz*dt;
	// 0 = z0 + dz*dt;
	// -z0/dz = dt;

	double dt = (-w2c.getOrigin().z() / dir_world.z());

	tf::Vector3 proj = w2c.getOrigin() + dir_world * (-w2c.getOrigin().z() / dir_world.z()) * dt;

	if(dt <= 0)
	{
		behind = true;
	}
	else
	{
		behind = false;
	}

	ROS_ASSERT(proj.z() == 0);

	return proj;
}

cv::Mat GridRenderer::renderGridByProjection()
{
	cv::Mat result = cv::Mat(size, CV_8UC3);

	cv::Mat_<float> Kinv = K.inv();

	for(int i = 0; i < result.rows; i++)
	{
		for(int j = 0; j < result.cols; j++)
		{
			cv::Mat_<float> dir = Kinv * (cv::Mat_<float>(3, 1) << j, i, 1);

			bool behind = false;

			tf::Vector3 proj = project2XYPlane(dir, behind);

			if(behind) // break and set the color to background
			{
				continue;
			}

			bool colorSet = false;

			for(auto e : grid)
			{
				if(e.pointInQuad(proj))
				{
					colorSet = true;
					result.at<cv::Vec3b>(i, j) = e.color;
					break;
				}
			}

			if(!colorSet)
			{
				//set the color to background
				result.at<cv::Vec3b>(i, j) = _BACKGROUND;
			}
		}
	}

	return result;
}



