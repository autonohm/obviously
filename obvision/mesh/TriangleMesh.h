#ifndef TRIANGLE_MESH_H_
#define TRIANGLE_MESH_H_

#include <iostream>
#include <vector>

using namespace std;

namespace obvious
{

/**
 * @class TriangleMesh
 * @brief Meshing operations for point clouds
 * @author Stefan May
 */
class TriangleMesh
{
public:

  /**
   * Standard constructur
   * @param maxsize Maximum number of triangles, e.g. for clouds in grid layout 2 times the number of points
   * @param maxDiscontinuity Maximum depth discontinuity of neighboring points. Points with a larger difference are discarded.
   */
  TriangleMesh(unsigned int maxsize, double maxDiscontinuity = 0.03);

  /**
   * Destructor
   */
  ~TriangleMesh();

  /**
   * Compute surface of created mesh succeeding a call of createMeshxxx
   * @return surface in m^2
   */
  double computeSurface();

  /**
   * Create mesh from local neighborhood, i. e. a grid layout of input data is expected
   * @param[in] coords Coordinates of a point cloud in grid layout
   * @param[in] rows Number of rows of input data
   * @param[in] cols Number of columns of input data
   * @param[in] rgb RGB data of a point cloud in grid layout
   * @param[in] mask Validity mask of a point cloud in grid layout
   */
  void createMeshFromOrganizedCloud(double* coords, unsigned int rows, unsigned cols, unsigned char* rgb=NULL, bool* mask=NULL, double* normals=NULL);

  /**
   * Accessor to coordinates of valid points (grid layout is not given anymore)
   */
  double** getCoords();

  double** getNormals();

  /**
   * Accessor to RGB data of valid points (grid layout is not given anymore)
   */
  unsigned char** getRGB();

  /**
   * Accessor to array of indices, i. e. point indices related to filtered data
   * @return index array (dimension: nx3, with 3 points per triangle)
   */
  unsigned int** getIndices();

  /**
   * Accessor to array of input indices, i. e. point indices related to input data
   * @return index array (dimension: nx3, with 3 points per triangle)
   */
  unsigned int** getInputIndices();

  /**
   * Accessor to size of valid input data
   * @return number of valid points
   */
  unsigned int getNumberOfPoints();

  /**
   * Accessor to size of triangle mesh
   * @return number of triangles in surface mesh
   */
  unsigned int getNumberOfTriangles();

private:

  /**
   * Coordinate array of only valid points (grid layout is not given anymore)
   */
  double**        _coords;

  double**        _normals;

  /**
   * RGB array of only valid points (grid layout is not given anymore)
   */
  unsigned char** _rgb;

  /**
   * Number of valid input points
   */
  unsigned int    _size;

  /**
   * Array of indices of surface mesh of filtered input data, i.e. having invalid points removed
   */
  unsigned int**  _indices;

  /**
   * Array of indices of surface mesh related to the input data
   */
  unsigned int**  _inputIndices;

  /**
   * Number of triangles in created mesh
   */
  unsigned int    _triangles;

  /**
   * Hash table for valid points
   */
  unsigned int*   _validIndices;

  /**
   * Maximum discontinuity in depth between neighboring points
   */
  double _maxDiscontinuitySqr;
};

}

#endif /*TRIANGLE_MESH_H_*/
