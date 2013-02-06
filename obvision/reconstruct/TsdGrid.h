#ifndef TSDGRID_H
#define TSDGRID_H

#include "obcore/math/Matrix.h"
#include "Projection.h"

namespace obvious
{

/**
* Cell structure for TsdGrid
* @param tsdf value of the truncated signed distance function
* @param weight used to calculate mean of all points in a voxel
*/
struct TsdCell
{
  double tsdf;
  double weight;
};

/**
 * @class TsdGrid
 * @brief Grid on the basis of true signed distance functions
 * @author Stefan May, Philipp Koch
 */
class TsdGrid
{
public:

 /**
  * Standard constructor
  * Allocates and initializes space and matrices
  * @param height,width,depth dimension of the allocated space in meters
  * @param cols
  * @param rows
  * @param cellSize
  * @param P projection matrix
  */
  TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize, const unsigned int beams, double angularRes);

 /**
  * Destructor
  */
 virtual ~TsdGrid();

 /**
  *
  */
 unsigned int getCellsX();

 /**
  *
  */
 unsigned int getCellsY();

 /**
  *
  */
 double getCellSize();

 /**
  * setMaxTruncation
  * Function to set the max truncation
  * @param new value new  max_truncation
  */
 void setMaxTruncation(const double val);

 /**
  *
  */
 double getMaxTruncation();

 /**
  * set current transformation
  * @param Tdata 3x3 transformation matrix
  */
 void setTransformation(double* Tdata);

 /**
  *
  */
 double* getTransformation();

 /**
  *
  */
 void push(double* depthArray, bool* mask);

 /**
  *
  * Calculates normal of plain element hit by a ray caster
  * @param[out] coordinates
  * @param[out] normal vector
  */
 bool interpolateNormal(const double coord[2], double normal[2]);

 /**
  * interpolate bilinear
  * @param coordinates pointer to coordinates of intersection
  * @param[out] tsdf interpolated TSD value
  */
 bool interpolateBilinear(double coord[2], double* tsdf);

 TsdCell** getGrid(void){return(_grid);}

private:

 /**
  *
  */
 void addTsdfValue(const double cellCoords[2], const unsigned int x, const unsigned int y, const double depth);

 unsigned int TsdGrid::phi2Index(double phi);

 /**
  *
  */
 bool coord2Cell(double coord[2], int* x, int* y, double* dx, double* dy);

 int _cellsX;

 int _cellsY;

 int _sizeOfGrid;

 int _dimX;

 int _dimY;

 TsdCell** _grid;

 Matrix* _cellCoords;

 double _cellSize;

 double _invCellSize;

 Matrix *_T;

 Matrix *_Tinv;

 double _tr[2];

 double _maxTruncation;

 double _phiMin;

 double _phiMax;

 double _angularRes;
};

}

#endif
