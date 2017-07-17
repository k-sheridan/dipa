/*
 * DipaParams.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef DIPA_INCLUDE_DIPA_DIPAPARAMS_H_
#define DIPA_INCLUDE_DIPA_DIPAPARAMS_H_



#define SUPER_DEBUG false

//GRID RENDERER
#define GRID_WIDTH 20
#define GRID_HEIGHT 20
#define GRID_SPACING 1
#define INNER_LINE_THICKNESS 0.04
#define OUTER_LINE_THICKNESS 0.04

#define ABSOLUTE_MIN_Z 0.001
#define ABSOLUTE_MAX_Z 5.0

//ICP
#define MAX_ITERATIONS 20

#define CONVERGENCE_DELTA 0.1

//OUTLIER DETECTION
//maximum normal for a correspondence in pixels
#define USE_MAX_NORM true
#define MAX_NORM 25

//OUTLIER DETECTION
//minimum initial matches after huber max norm
#define MINIMUM_INITIAL_MATCHES 4
//minimum huber matches after icp converge
#define MINIMUM_FINAL_MATCHES 10
#define MINIMUM_HUBER_RATIO 0.5

//OUTLIER DETECTION
//position constraints on grid alignments
//IMPORTANT change these constraints based on the grid
//x and y constraints are determined by the grid size specified
#define ICP_MAX_Z 5
#define ICP_MIN_Z 0.5

//OUTLIER DETECTION
// maximium per pixel error to be unti deemed outlier
#define MAX_ICP_ERROR 1.5

//END ICP

//CORNER DETECTION
#define WHITE_THRESH 140
//lower this if you have easy corners to detect
#define INVERSE_IMAGE_SCALE 4.0

#define CANNY_BLUR_SIGMA 2.0
#define CANNY_THRESH_1 50
#define CANNY_THRESH_2 200

#define HARRIS_SIZE 2
#define HARRIS_APERTURE 3
#define HARRIS_K 0.04

#define HOUGH_THRESH 75

//#define MIN_D_THETA 10 * CV_PI/180
#define PARALLEL_THRESH 0.1

//END GRID CORNER DETECTION

//PLANAR ODOM
//fast corner detector for planar odometry
#define FAST_THRESHOLD 100
#define FAST_BLUR_SIGMA 0.5

#define KLT_MIN_EIGEN 1e-4

#define MIN_NEW_FEATURE_DIST 30

#define NUM_FEATURES 40

#define MINIMUM_TRACKABLE_FEATURES 4

//OUTLIER DETECTION
// if icp has not realigned vo since this time, we have lost tracking
#define MAXIMUM_TIME_SINCE_REALIGNMENT 5

//if the ppe of our planar odometry exceeds this value we have lost odometry
#define MAXIMUM_VO_PPE 7.0

//END PLANAR ODOM

#define ODOM_TOPIC "dipa/odom"

// this topic will serve as a last resort for realignment
#define REALIGNMENT_TOPIC "state/pose"

#define BOTTOM_CAMERA_TOPIC "/bottom_camera/image_rect"
#define CAMERA_FRAME "bottom_camera"

#define BASE_FRAME "base_link"

#define WORLD_FRAME "world"

// insight is an image representing the describing the current state of DIPA
#define PUBLISH_INSIGHT true
#define INSIGHT_TOPIC "dipa/insight"

#endif /* DIPA_INCLUDE_DIPA_DIPAPARAMS_H_ */
