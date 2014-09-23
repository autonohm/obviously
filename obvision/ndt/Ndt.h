#ifndef NDT_H_
#define NDT_H_

#include <iostream>
#include <string.h>
using namespace std;

#include "obcore/base/CartesianCloud.h"
#include "obcore/base/System.h"

#include "obcore/math/linalg/linalg.h"

using namespace obvious;

namespace obvious
{

/**
 * NDT return states
 */
enum EnumNdtState { NDT_IDLE 			= 0,
  NDT_PROCESSING 		= 1,
  NDT_NOTMATCHABLE 	= 2,
  NDT_MAXITERATIONS 	= 3,
  NDT_TIMEELAPSED 	= 4,
  NDT_SUCCESS 		= 5,
  NDT_CONVERGED   = 6,
  NDT_ERROR			= 7 };


struct NdtCell
{
  vector<double*> coords;
  double* centroid;
  Matrix* cov;
  Matrix* cov_inv;
  bool isOccupied(){ return (coords.size()>=5); };
};

/**
 * @class Ndt
 * @brief Represents the normal distribution transform
 * @author Stefan May
 **/
class Ndt
{
public:
  /**
   * Standard constructor
   */
  Ndt(int minX, int maxX, int minY, int maxY);

  /**
   * Destructor
   */
  ~Ndt();

  /**
   * convert enumeration to char*
   * @param eState state enumeration
   * @return state string
   */
  static const char* state2char(EnumNdtState eState);

  /**
   * Sample model point cloud to NDT space
   * @param coords model coordinates
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
   */
  void setModel(Matrix* coords, double probability=1.0);

  /**
   * Copy scene to internal buffer
   * @param coords scene coordinates
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
   */
  void setScene(Matrix* coords, double probability=1.0);

  /**
   * Reset state of NDT algorithm
   */
  void reset();

  /**
   * Set maximum number of iteration steps
   * @param iterations maximum number of iteration steps
   */
  void setMaxIterations(unsigned int iterations);

  /**
   * Get maximum number of iteration steps
   * @return maximum number of iteration steps
   */
  unsigned int getMaxIterations();

  /**
   * Start iteration
   * @param rms return value of RMS error
   * @param iterations return value of performed iterations
   * @param Tinit apply initial transformation before iteration
   * @return  processing state
   */
  EnumNdtState iterate(double* rms, unsigned int* iterations, Matrix* Tinit=NULL);

  /**
   * Get final 4x4 rotation matrix determined through iteration
   * @return final transformation matrix
   */
  Matrix getFinalTransformation4x4();

  /**
   * Get final rotation matrix determined through iteration
   * @return final transformation matrix
   */
  Matrix getFinalTransformation();

  /**
   * Get last rotation matrix determined within the last iteration step
   * @return last transformation matrix
   */
  Matrix getLastTransformation();

private:

  /**
   * apply transformation to data array
   * @param data 2D or 3D coordinates
   * @param size number of points
   * @param dim dimensionality
   * @param T transformation matrix
   */
  void applyTransformation(double** data, unsigned int size, unsigned int dim, Matrix* T);

  /**
   * internal memory check routine
   * @param rows row size of needed memory
   * @param memsize row size of target memory
   * @param mem target memory
   */
  void checkMemory(unsigned int rows, unsigned int cols, unsigned int &memsize, double** &mem);

  int _minX;
  int _maxX;
  int _minY;
  int _maxY;

  /**
   * maximum number of iterations
   */
  unsigned int _maxIterations;

  /**
   * size of internal scene buffer
   */
  unsigned int _sizeSceneBuf;

  NdtCell** _model;

  /**
   * the scene
   */
  double** _scene;

  /**
   * local copy of scene
   */
  double** _sceneTmp;

  /**
   * size of scene
   */
  unsigned int _sizeScene;

  /**
   * final transformation matrix, found after iteration (fixed dimensions for 2D and 3D case)
   */
  Matrix* _Tfinal4x4;

  /**
   * transformation matrix of last step
   */
  Matrix* _Tlast;

  /**
   * dimension of space
   */
  int _dim;

  double _d1;

  double _d2;

};

}

#endif /*NDT_H_*/
