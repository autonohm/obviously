#ifndef _TSD_SPACE_UTILS_H
#define _TSD_SPACE_UTILS_H

#include "obcore/math/Matrix.h"

namespace obvious
{

/**
* Struct for Class Tsd_space
* Contains data that is saved in a voxel
* @param tsdf value of the truncated signed distance function
* @param weight used to calulate mean of all points in a voxel
*/
struct TsdVoxel
	{
	double tsdf;
	double weight;
	};

/**
* Struct for Class Tsd_space
* Contains Carthesian Point Coordinates
* @param x,y,z carthesian coords
*/
struct Point
	{
  double x;
  double y;
  double z;
	};

/**
* Enum Variable type for return-values
* OK Function returned with no error
* ERROR Function terminated with error
* EDGE Ray-tracer reached Edge of space without hitting an object
*/
enum MSG
	{
  OK,
  ERROR,
  EDGE
	};

/**
* abs3D
* Function to calculate the value of a threedimensional vector
* @param foot point pointer to foot point
* @param ref point pointer to reference point; set to NULL if foot point is vector
* @return value value of the vector
*/
double abs3D(obvious::Matrix *foot_point,obvious::Matrix *ref_point);

/**
* z_img_to_gysc
* Function to generate a 8-Bit grayscale picture out of a depth image
* @param max_depth defines the 255st grayvalue
* @param path pointer to the path where the image will be stored
* @param z_bfr pointer to the depth image
* @param width,height dimensions of the image
* @param trace_on set to true to get trace outputs
*/
void z_img_to_gysc(const double max_depth,const char *path,const double *z_bfr,const unsigned int width,const unsigned int height,bool trace_on);


}

#endif
