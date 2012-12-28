/**
* @file Xtion.h
* @autor christian
* @date  23.12.2012
*
*
*/

#ifndef XTION_H_
#define XTION_H_

#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

#include <iostream>
#include <fstream>

#include "obcore/math/MatRGB.h"
#include "obcore/math/MatD.h"

using namespace xn;
using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Kinect
 * @brief Kinect interface
 * @author Stefan May
 **/
class Xtion
{
public:
  /**
   * Standard constructor;
   */
    Xtion(const char* path);
  /**
   * Standard destructor
   */
  ~Xtion();
  /**
   * Grab new image
   * @return success
   */
  bool grab();
  /**
   * Start serializing the data stream to file
   * @param filename name of file
   */
  void startRecording(char* filename);
  /**
   * Stop previously started recording
   */
  void stopRecording();
  /**
   * Get number of rows of images
   * @return rows
   */
  unsigned int getRows();
  /**
   * Get number of columns of images
   * @return columns
   */
  unsigned int getCols();
  /**
   * Accessor to pointer of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords();
  /**
   * Accessor to mask of valid points
   * @return pointer to mask
   */
  bool* getMask();
  /**
   * Get pointer to z-Buffer
   */
  double* getZ();

    /**
     * Get Z buffer
     * @return MatD
     */
    MatD getMatZ(void) const;
private:
  /**
   * Grab new color image
   * @return success
   */
  bool grabDistance();
  /**
   * Grab new "night vision" image
   * @return success
   */
  bool grabIR();

  Context _context;

  DepthGenerator _depth;
  IRGenerator    _ir;
  bool          _useIR;

  XnPoint3D* _proj;
  XnPoint3D* _wrld;

  double* _coords;
  double* _z;
  bool*   _mask;

  unsigned int _rows;
  unsigned int _cols;

  bool      _init;
  bool      _record;
  ofstream  _recfile;
};

} // end namespace

#endif /* XTION_H_ */
