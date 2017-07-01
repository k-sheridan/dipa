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

	boundary_padding = BOUNDARY_PADDING;
	inner_line_thickness = INNER_LINE_THICKNESS;
	outer_line_thickness = OUTER_LINE_THICKNESS;
	grid_size = GRID_SIZE;
	grid_spacing = GRID_SPACING;


	generateGrid();
	//renderSourceImage();
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

	double grid_size = this->grid_size;
	double ilt=inner_line_thickness;
	double olt=outer_line_thickness;
	double grid_spacing = this->grid_spacing;
	double min = -(grid_size * grid_spacing / 2);
	double max = (grid_size * grid_spacing / 2);

	Quad floor;
	floor.color = _FLOOR;
	floor.vertices.push_back(cv::Point2f( max + boundary_padding, max + boundary_padding));
	floor.vertices.push_back(cv::Point2f( min - boundary_padding, max + boundary_padding));
	floor.vertices.push_back(cv::Point2f( min - boundary_padding, min - boundary_padding));
	floor.vertices.push_back(cv::Point2f( max + boundary_padding, min - boundary_padding));
	grid.push_front(floor);

	ROS_DEBUG_STREAM("front size: " << grid.front().vertices.size());

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

		ROS_DEBUG_STREAM("front size: " << grid.front().vertices.size());

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
		else
		{
			q.vertices.push_back(cv::Point2f(max, y - olt));
			q.vertices.push_back(cv::Point2f(max, y + olt));
			q.vertices.push_back(cv::Point2f(min, y + olt));
			q.vertices.push_back(cv::Point2f(min, y - olt));
		}

		grid.push_front(q);

		ROS_DEBUG_STREAM("front size: " << grid.front().vertices.size());

		line++;
	}


	//generate the grid corners

	int x_line = 0;
	for(double x = min; x < max + grid_spacing; x += grid_spacing)
	{
		int y_line = 0;
		for(double y = min; y < max + grid_spacing; y += grid_spacing)
		{
			if(y_line == 0 || y_line == grid_size)
			{
				if(x_line == 0 || x_line == grid_size)
				{
					if(x_line == 0 && y_line == 0)
					{
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y - olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y + olt / 2.0, 0));
					}
					else if(x_line == grid_size && y_line == 0)
					{
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y - olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y + olt / 2.0, 0));
					}
					else if(x_line == grid_size && y_line == grid_size)
					{
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y + olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y - olt / 2.0, 0));
					}
					else{
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y + olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y - olt / 2.0, 0));
					}

				}
				else
				{
					if(x_line == 0) // two up
					{
						grid_corners.push_back(tf::Vector3(x - ilt / 2.0, y + olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x + ilt / 2.0, y + olt / 2.0, 0));
					}
					else // two down
					{
						grid_corners.push_back(tf::Vector3(x - ilt / 2.0, y - olt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x + ilt / 2.0, y - olt / 2.0, 0));
					}
				}
			}
			else
			{
				if(x_line == 0 || x_line == grid_size)
				{
					if(x_line == 0) // two to the right
					{
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y + ilt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x + olt / 2.0, y - ilt / 2.0, 0));
					}
					else // two to the left
					{
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y + ilt / 2.0, 0));
						grid_corners.push_back(tf::Vector3(x - olt / 2.0, y - ilt / 2.0, 0));
					}

				}
				else
				{
					//this is the case that we are in the center
					grid_corners.push_back(tf::Vector3(x + ilt / 2.0, y + ilt / 2.0, 0));
					grid_corners.push_back(tf::Vector3(x - ilt / 2.0, y + ilt / 2.0, 0));
					grid_corners.push_back(tf::Vector3(x - ilt / 2.0, y - ilt / 2.0, 0));
					grid_corners.push_back(tf::Vector3(x + ilt / 2.0, y - ilt / 2.0, 0));

					ROS_DEBUG_STREAM("drew center about: " << x << ", " << y);
				}
			}


			y_line++;
		}
		x_line++;
	}

}

cv::Mat GridRenderer::computeHomography()
{
	cv::Mat_<float> R = (cv::Mat_<float>(3, 3) << w2c.getBasis().getRow(0).x(), w2c.getBasis().getRow(0).y(), w2c.getBasis().getRow(0).z(),
			w2c.getBasis().getRow(1).x(), w2c.getBasis().getRow(1).y(), w2c.getBasis().getRow(1).z(),
			w2c.getBasis().getRow(2).x(), w2c.getBasis().getRow(2).y(), w2c.getBasis().getRow(2).z());

	cv::Mat_<float> t = (cv::Mat_<float>(3, 1) << w2c.getOrigin().x(), w2c.getOrigin().y(), w2c.getOrigin().z());
	cv::Mat_<float> n = (cv::Mat_<float>(1, 3) << 0, 0, 1);

	cv::Mat H = R - (t * n) * (1 / fabs(w2c.getOrigin().z()));

	return H;
}

tf::Vector3 GridRenderer::project2XYPlane(cv::Mat_<float> dir, bool& behind)
{
	tf::Vector3 dir_world = (w2c * tf::Vector3(dir(0), dir(1), dir(2))) - w2c.getOrigin();

	//ROS_DEBUG_STREAM("dir cam " << dir);
	//ROS_DEBUG_STREAM("z dir world: " << dir_world.z());
	// z = z0 + dz*dt;
	// 0 = z0 + dz*dt;
	// -z0/dz = dt;

	double dt = (-w2c.getOrigin().z() / dir_world.z());

	//ROS_DEBUG_STREAM("dt: " << dt);

	tf::Vector3 proj = w2c.getOrigin() + dir_world * dt;

	if(dt <= 0)
	{
		behind = true;
	}
	else
	{
		behind = false;
	}
	//ROS_DEBUG_STREAM("z height: " << proj.z());
	//ROS_ASSERT(proj.z() == 0);

	return proj;
}

void GridRenderer::renderSourceImage()
{
	double width = (grid_size * grid_spacing + 2*boundary_padding) / METRIC_RESOLUTION;
	double height = width;

	ROS_DEBUG_STREAM("width: " << width);

	cv::Mat_<float> tempK = (cv::Mat_<float>(3, 3) << 1/METRIC_RESOLUTION, 0, width/2,
			0, 1/METRIC_RESOLUTION, height/2,
			0, 0, 1);
	this->setIntrinsic(tempK);
	this->setSize(cv::Size(width, height));

	source_trans.setRotation(tf::Quaternion(1/sqrt(2), 0, 0, 0));
	source_trans.setOrigin(tf::Vector3(0, 0, 1));

	this->setW2C(source_trans);

	sourceRender = this->renderGridByProjection();


}

cv::Point2f GridRenderer::projectPoint(tf::Vector3 in, bool& good)
{
	tf::Vector3 proj = this->c2w * in;

	if(proj.z() <= 0)
	{
		good = false;
		return cv::Point2f(-1, -1);
	}

	cv::Point2f px = cv::Point2f(this->K(0) * (proj.x()/proj.z()) + this->K(2), this->K(4) * (proj.y()/proj.z()) + this->K(5));

	if(px.x < 0 || px.y < 0 || px.x > this->size.width || px.y > this->size.height)
	{
		good = false;
		return cv::Point2f(-1, -1);
	}

	good = true;
	return px;
}

cv::Mat GridRenderer::drawCorners(cv::Mat in, std::vector<cv::Point2f> corners)
{
	for(auto e : corners)
	{
		cv::drawMarker(in, e, cv::Scalar(255, 255, 0), cv::MARKER_SQUARE, 4);
	}

	return in;
}

std::vector<cv::Point2f> GridRenderer::renderGridCorners()
{
	std::vector<cv::Point2f> corners;
	for(auto e : grid_corners)
	{
		bool good = false;
		cv::Point2f px = this->projectPoint(e, good);
		if(good)
		{
			corners.push_back(px);
		}
	}

	return corners;
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



