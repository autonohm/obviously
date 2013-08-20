#ifndef TSDSPACE_H
#define TSDSPACE_H

#include "obcore/math/Matrix.h"
#include "Sensor.h"

namespace obvious
{

/**
* Struct for Class Tsd_space
* Contains data that is saved in a voxel
* @param tsdf value of the truncated signed distance function
* @param weight used to calculate mean of all points in a voxel
*/
struct TsdVoxel
{
  double tsdf;
  double weight;
  unsigned char rgb[3];
};

/**
* Struct for Class Tsd_space
* Contains Cartesian Point Coordinates
* @param x,y,z Cartesian coords
*/
struct Point
{
  double x;
  double y;
  double z;
};

/**
 * @class TsdSpace
 * @brief Space representing a true signed distance function
 * @author Philipp Koch, Stefan May
 */
class TsdSpace
{
public:

 /**
  * Standard constructor
  * Allocates and initializes space and matrices
  * @param height,width,depth dimension of the allocated space in meters
  * @param cols
  * @param rows
  * @param voxelDim edge length of the Tsd_voxels
  * @param P projection matrix
  */
  TsdSpace(const unsigned int height, const unsigned int width, const unsigned int depth, const double voxelDim);

 /**
  * Destructor
  */
 virtual ~TsdSpace();

 /**
  * Get number of voxels in x-direction
  */
 unsigned int getXDimension();

 /**
  * Get number of voxels in y-direction
  */
 unsigned int getYDimension();

 /**
  * Get number of voxels in z-direction
  */
 unsigned int getZDimension();

 /**
  * Get edge length of voxels
  */
 double getVoxelSize();

 /**
  * Set maximum truncation radius
  * Function to set the max truncation
  * @param val new truncation radius
  */
 void setMaxTruncation(const double val);

 /**
  * Get maximum truncation radius
  * @return truncation radius
  */
 double getMaxTruncation();

 /**
  * Function to set the current transformation matrix
  * @param TData content of the new transformation matrix
  */
 //void setTransformation(double *TData);

 /**
  * Get current transformation matrix
  * @return current transformation matrix
  */
 //double* getTransformation();


 //Matrix *getMTransformation(){return (_T);}

 /**
  *
  */
 void push(Sensor* sensor);//double *depthImage, bool* mask, unsigned char* rgbImage);

 /**
  * interpolate_trilineary
  * Function to interpolate TSDF trilineary
  * @param coordinates pointer to coordinates of intersection
  * @param[out] tsdf interpolated TSD value
  */
 bool interpolateTrilinear(double coord[3], double* tsdf);

 /**
  *
  */
 bool interpolateTrilinearRGB(double coord[3], unsigned char rgb[3]);

 /**
  *
  * Calculates normal to a hit plain element
  * @param normal Variable to store the components in. Has to be allocated by calling function (3 coordinates)
  */
 bool interpolateNormal(const double* coord, double* normal);

 bool buildSliceImage(const unsigned int depthIndex, unsigned char* image);

 //TsdVoxel*** getSpace(void){return(_space);}

private:

 /**
  *
  */
 void addTsdfValue(const unsigned int col, const unsigned int row, const unsigned int z, double sdf);

 /**
  *
  */
 bool coord2Voxel(double coord[3], int* x, int* y, int* z, Point* p);

 int _xDim;

 int _yDim;

 int _zDim;

 int _sizeOfSpace;

 int _height;

 int _width;

 int _depth;

 TsdVoxel*** _space;

 Matrix* _voxelCoordsHom;

 double _voxelSize;

 double _invVoxelSize;

 double _maxTruncation;

 //unsigned char* _rgbImage;

 Matrix*** _rays;
};

}

#endif
