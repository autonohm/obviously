#ifndef TSDSPACE_H
#define TSDSPACE_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/Sensor.h"
#include "TsdSpacePartition.h"

namespace obvious
{

enum EnumTsdSpaceLayout { LAYOUT_1x1x1=0,
                          LAYOUT_2x2x2=1,
                          LAYOUT_4x4x4=2,
                          LAYOUT_8x8x8=3,
                          LAYOUT_16x16x16=4,
                          LAYOUT_32x32x32=5,
                          LAYOUT_64x64x64=6,
                          LAYOUT_128x128x128=7,
                          LAYOUT_256x256x256=8,
                          LAYOUT_512x512x512=9,
                          LAYOUT_1024x1024x1024=10};


enum EnumTsdSpaceInterpolate { INTERPOLATE_SUCCESS=0,
                               INTERPOLATE_INVALIDINDEX=1,
                               INTERPOLATE_EMPTYPARTITION=2,
                               INTERPOLATE_ISNAN=3};

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
  * @param[in] voxelSize edge length of voxels in meters
  * @param[in] layoutPartition Partition layout, i.e., voxels in partition
  * @param[in] layoutSpace Space layout, i.e., partitions in space
  */
  TsdSpace(const double voxelSize, const EnumTsdSpaceLayout layoutPartition, const EnumTsdSpaceLayout layoutSpace);

 /**
  * Destructor
  */
 virtual ~TsdSpace();

 /**
  * Reset space to initial state
  */
 void reset();

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
  * Get number of cells along edge
  * @return number of cells
  */
 unsigned int getPartitionSize();

 /**
  * Get minimum for x-coordinate
  * @return x-coordinate
  */
 double getMinX();

 /**
  * Get maximum for x-coordinate
  * @return x-coordinate
  */
 double getMaxX();

 /**
  * Get minimum for y-coordinate
  * @return y-coordinate
  */
 double getMinY();

 /**
  * Get maximum for y-coordinate
  * @return y-coordinate
  */
 double getMaxY();

 /**
  * Get minimum for z-coordinate
  * @return z-coordinate
  */
 double getMinZ();

 /**
  * Get maximum for z-coordinate
  * @return z-coordinate
  */
 double getMaxZ();

 /**
  * Get centroid of space
  * @param[out] centroid centroid coordinates
  */
 void getCentroid(double centroid[3]);

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
  * Get pointer to internal partition space
  * @return pointer to 3D partition space
  */
 TsdSpacePartition**** getPartitions();

 /**
  * Check, if partition belonging to coordinate is initialized
  * @param coord query coordinate
  * @return initialization state
  */
 bool isPartitionInitialized(double coord[3]);

 /**
  * Determine whether sensor is inside space
  * @param sensor
  */
 bool isInsideSpace(Sensor* sensor);

 /**
  * Push sensor data to space
  * @param[in] sensor abstract sensor instance holding current data
  */
 void push(Sensor* sensor);

 /**
  * Push sensor data to space using octree insertion
  * @param[in] sensor abstract sensor instance holding current data
  */
 void pushTree(Sensor* sensor);

 /**
  * interpolate_trilineary
  * Function to interpolate TSDF trilineary
  * @param coordinates pointer to coordinates of intersection
  * @param[out] tsd interpolated TSD value
  */
 EnumTsdSpaceInterpolate interpolateTrilinear(double coord[3], double* tsd);

 /**
  *
  */
 bool interpolateTrilinearRGB(double coord[3], unsigned char rgb[3]);

 /**
  *
  * Calculates normal of crossed surface
  * @param normal Variable to store the components in. Has to be allocated by calling function (3 coordinates)
  */
 bool interpolateNormal(const double* coord, double* normal);

 //bool buildSliceImage(const unsigned int depthIndex, unsigned char* image);
 /**
  * Method to store the content of the grid in a file
  * @param filename
  */
 //void serialize(const char* filename);

 /**
  * Method to load values out of a file into the grid
  * @param filename
  */
 //void load(const char* filename);
private:

 void pushRecursion(Sensor* sensor, double pos[3], TsdSpaceComponent* comp, vector<TsdSpacePartition*> &partitionsToCheck);

 void propagateBorders();

 void addTsdValue(const unsigned int col, const unsigned int row, const unsigned int z, double sd, unsigned char* rgb);

 bool coord2Index(double coord[3], int* x, int* y, int* z, double* dx, double* dy, double* dz);

 TsdSpaceComponent* _tree;

 unsigned int _cellsX;

 unsigned int _cellsY;

 unsigned int _cellsZ;

 double _voxelSize;

 double _invVoxelSize;

 double _maxTruncation;

 double _minX;

 double _maxX;

 double _minY;

 double _maxY;

 double _minZ;

 double _maxZ;

 TsdSpacePartition**** _partitions;

 int* _lutIndex2Partition;
 int* _lutIndex2Cell;

 int _partitionsInX;

 int _partitionsInY;

 int _partitionsInZ;
};

}

#endif
