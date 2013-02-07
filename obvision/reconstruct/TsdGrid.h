#ifndef TSDGRID_H
#define TSDGRID_H

#include "obcore/math/Matrix.h"
#include "obvision/reconstruct/SensorPolar2D.h"

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
    * @param[in] dimX Number of cells in x-dimension
    * @param[in] dimY Number of cells in y-dimension
    * @param[in] cellSize Size of cell in meters
    */
   TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize);

   /**
    * Destructor
    */
   virtual ~TsdGrid();

   /**
    * Get number of cells in x-dimension
    * @return number of cells
    */
   unsigned int getCellsX();

   /**
    * Get number of cells in y-dimension
    * @return number of cells
    */
   unsigned int getCellsY();

   /**
    * Get size of cell in meters
    * @return size
    */
   double getCellSize();

   /**
    * Set maximum truncation radius
    * @param[in] val truncation radius
    */
   void setMaxTruncation(const double val);

   /**
    * Get maximum truncation radius
    * @return truncation radius
    */
   double getMaxTruncation();

   /**
    * Push current measurement from sensor
    * @param[in] virtual 2D measurement unit
    */
   void push(SensorPolar2D* sensor);

   /**
    * Create grayscale image from tsdf grid
    * @param[out] grayscale image
    */
   void grid2GrayscaleImage(unsigned char* image);

   /**
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

private:

   /**
    * Add tsdf value to grid
    * @param[in] x index in x-dimension
    * @param[in] y index in y-dimension
    * @param[in] sdf signed distance function value
    */
   void addTsdfValue(const unsigned int x, const unsigned int y, const double sdf);

   /**
    * Convert arbitrary coordinate to grid coordinates
    * @param[in] coord 2D query coordinates
    * @param[out] x x-index
    * @param[out] y y-index
    * @param[out] dx x-coordinate of cell-center in metric space
    * @param[out] dy y-coordinate of cell-center in metric space
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

   double _maxTruncation;
};

}

#endif
