#ifndef TSDSPACE_H
#define TSDSPACE_H

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Point.h"
#include "obvision/reconstruct/reconstruct_defs.h"
#include "obvision/reconstruct/Sensor.h"
#include "TsdSpacePartition.h"

#include <string>
#include <Eigen/Dense>

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > stdVecEig3f;

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

enum EnumSpaceAxis
{
  X = 0,
  Y,
  Z
};

struct PointWithAngle
  {
  PointWithAngle(const Eigen::Vector3f& point, const double angle):
    point(point),
    angle(angle){}
    Eigen::Vector3f point;
    double angle;
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
   * @param[in] voxelSize edge length of voxels in meters
   * @param[in] layoutPartition Partition layout, i.e., voxels in partition
   * @param[in] layoutSpace Space layout, i.e., partitions in space
   */
  TsdSpace(const double voxelSize, const EnumTsdSpaceLayout layoutPartition, const EnumTsdSpaceLayout layoutSpace);
  TsdSpace(const double voxelSize, const EnumTsdSpaceLayout layoutPartition, const unsigned int cellsX, const unsigned int cellsY, const unsigned int cellsZ);

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
  unsigned int getXDimension() const { return _cellsX; }

  /**
   * Get number of voxels in y-direction
   */
  unsigned int getYDimension() const { return _cellsY; }

  /**
   * Get number of voxels in z-direction
   */
  unsigned int getZDimension() const { return _cellsZ; }

  /**
   * Get number of partitions in x-direction
   */
  int getPartitionsInX() const { return _partitionsInX; }

  /**
   * Get number of partitions in y-direction
   */
  int getPartitionsInY() const { return _partitionsInY; }

  /**
   * Get number of partitions in z-direction
   */
  int getPartitionsInZ() const { return _partitionsInZ; }

  /**
   * Get edge length of voxels
   */
  obfloat getVoxelSize() const { return _voxelSize; }

  /**
   * Get number of cells along edge
   * @return number of cells
   */
  unsigned int getPartitionSize();

  /**
   * Get minimum for x-coordinate
   * @return x-coordinate
   */
  obfloat getMinX() const { return _minX; }

  /**
   * Get maximum for x-coordinate
   * @return x-coordinate
   */
  obfloat getMaxX() const { return _maxX; }

  /**
   * Get minimum for y-coordinate
   * @return y-coordinate
   */
  obfloat getMinY() const { return _minY; }

  /**
   * Get maximum for y-coordinate
   * @return y-coordinate
   */
  obfloat getMaxY() const { return _maxY; }

  /**
   * Get minimum for z-coordinate
   * @return z-coordinate
   */
  obfloat getMinZ() const { return _minZ; }

  /**
   * Get maximum for z-coordinate
   * @return z-coordinate
   */
  obfloat getMaxZ() const { return _maxZ; }

  /**
   * Get centroid of space
   * @param[out] centroid centroid coordinates
   */
  void getCentroid(obfloat centroid[3]);

  /**
   * Set maximum truncation radius
   * Function to set the max truncation
   * @param val new truncation radius
   */
  void setMaxTruncation(const obfloat val);

  /**
   * Get maximum truncation radius
   * @return truncation radius
   */
  double getMaxTruncation() const { return _maxTruncation; }

  /**
   * Get pointer to internal partition space
   * @return pointer to 3D partition space
   */
  TsdSpacePartition**** getPartitions() const { return _partitions; }

  /**
   * Check, if partition belonging to coordinate is initialized
   * @param coord query coordinate
   * @return initialization state
   */
  bool isPartitionInitialized(obfloat coord[3]);

  /**
   * Determine whether sensor is inside space
   * @param sensor
   */
  bool isInsideSpace(Sensor* sensor);
  bool isInsideSpace(const Eigen::Vector3f& pos);

  /**
   * Push sensor data to space
   * @param[in] sensor abstract sensor instance holding current data
   */
  void push(Sensor* sensor);

  void push(stdVecEig3f& points);

  void push(const std::vector<stdVecEig3f>& data, const unsigned int width, const unsigned int height, const Eigen::Vector3f& t, const double resDepth,
      const double resHor);

  /**
   * Push sensor data to space using forward raycast
   * @param[in] sensor abstract sensor instance holding current data
   */
  void pushForward(Sensor* const sensor);

  /**
   * @brief Push pointcloud forward in without usage of a sensor.
   * Every point in the vector is treated as a 2D Laser measurement from (0.0, 0.0, z) to (x, y, z) measurement.
   * This method violates the sensor concept in this framework but is necessary to treat pointclouds
   * from unknown poses.
   */
  void pushForward(const stdVecEig3f& points);

  /**
   * Push sensor data to space using octree insertion
   * @param[in] sensor abstract sensor instance holding current data
   */
  void pushTree(Sensor* sensor);

  /**
   * interpolate_trilineary
   * Method to interpolate TSDF trilineary
   * @param coord pointer to coordinates of intersection
   * @param[out] tsd interpolated TSD value
   */
  EnumTsdSpaceInterpolate interpolateTrilinear(obfloat coord[3], obfloat* tsd);

  /**
   * interpolate_trilineary
   * Method to interpolate RGB data trilineary
   * @param coord pointer to coordinates of intersection
   * @param[out] rgb interpolated RGB vector
   */
  EnumTsdSpaceInterpolate interpolateTrilinearRGB(obfloat coord[3], unsigned char rgb[3]);

  /**
   *
   * Calculates normal of crossed surface
   * @param normal Variable to store the components in. Has to be allocated by calling function (3 coordinates)
   */
  bool interpolateNormal(const obfloat* coord, obfloat* normal);

  EnumTsdSpaceInterpolate getTsd(obfloat coord[3], obfloat* tsd);

  //bool buildSliceImage(const unsigned int depthIndex, unsigned char* image);
  /**
   * Method to store the content of the grid in a file
   * @param filename
   */
  void serialize(const char* filename);

  /**
   * Method to load values out of a file into the grid
   * @param filename
   */
  TsdSpace* load(const char* filename);

  bool sliceImage(const unsigned int idx, const EnumSpaceAxis& axis, std::vector<unsigned char>* const rgb);

  void serializeSliceImages(const EnumSpaceAxis& axis, const std::string& path = "");

private:

  void pushRecursion(Sensor* sensor, obfloat pos[3], TsdSpaceComponent* comp, vector<TsdSpacePartition*> &partitionsToCheck);

  void propagateBorders();

  void addTsdValue(const unsigned int col, const unsigned int row, const unsigned int z, double sd, unsigned char* rgb);

  bool coord2Index(obfloat coord[3], int* x, int* y, int* z, obfloat* dx, obfloat* dy, obfloat* dz);

  TsdSpaceComponent* _tree;

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _cellsZ;

  obfloat _voxelSize;

  obfloat _invVoxelSize;

  obfloat _maxTruncation;

  obfloat _minX;

  obfloat _maxX;

  obfloat _minY;

  obfloat _maxY;

  obfloat _minZ;

  obfloat _maxZ;

  TsdSpacePartition**** _partitions;

  int* _lutIndex2Partition;
  int* _lutIndex2Cell;

  int* _lutIndex2PartitionX;
  int* _lutIndex2CellX;
  int* _lutIndex2PartitionY;
  int* _lutIndex2CellY;
  int* _lutIndex2PartitionZ;
  int* _lutIndex2CellZ;

  int _partitionsInX;

  int _partitionsInY;

  int _partitionsInZ;

  EnumTsdSpaceLayout _layoutPartition;

  EnumTsdSpaceLayout _layoutSpace;

};

bool compareZ(const Eigen::Vector3f& var1, const Eigen::Vector3f& var2);
bool compareAngle(const PointWithAngle& var1, const PointWithAngle& var2);

}

#endif
