/*
 * DipaParams.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef DIPA_INCLUDE_DIPA_DIPAPARAMS_H_
#define DIPA_INCLUDE_DIPA_DIPAPARAMS_H_



#define SUPER_DEBUG true

#define MAX_ITERATIONS 20

#define CONVERGENCE_DELTA 0.1

//maximum normal for a correspondence in pixels
#define USE_MAX_NORM true
#define MAX_NORM 25

//CORNER DETECTION
#define WHITE_THRESH 140
//lower this if you have easy corners to detect
#define INVERSE_IMAGE_SCALE 4.0

//uses fast corner detctor in addition to the line corners
#define USE_FAST_CORNERS false
#define FAST_BLUR_SIGMA 2
#define FAST_THRESHOLD 100

#define CANNY_BLUR_SIGMA 1.2
#define CANNY_THRESH_1 50
#define CANNY_THRESH_2 200

#define HARRIS_SIZE 2
#define HARRIS_APERTURE 3
#define HARRIS_K 0.04

#define HOUGH_THRESH 75

//#define MIN_D_THETA 10 * CV_PI/180
#define PARALLEL_THRESH 0.1

//END CORNER DETECTION

#define BOTTOM_CAMERA_TOPIC "/m7/camera/image_rect"
#define BOTTOM_CAMERA_FRAME "bottomCamera"

#define BASE_FRAME "base_link"

#define WORLD_FRAME "world"

#endif /* DIPA_INCLUDE_DIPA_DIPAPARAMS_H_ */
