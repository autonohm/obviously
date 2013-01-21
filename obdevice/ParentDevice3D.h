/**
* @file   ParentDevice3D.h
* @author Christian Pfitzner
* @date   07.01.2013
*
*
*/

#ifndef PARENTDEVICE3D_H_
#define PARENTDEVICE3D_H_

#include "obcore/math/MatD.h"
#include "obcore/math/MatRGB.h"
#include "obcore/base/Timer.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace obvious {

class ParentDevice3D
{
public:
  /**
   * Standard constructor
   */
  ParentDevice3D(unsigned int cols = 0, unsigned int rows = 0);

  /**
   * Default destructor
   */
  virtual ~ParentDevice3D(void) = 0;
  /**
   * Get number of rows of images
   * @return rows
   */
  unsigned int getRows() const  { return _rows; }
  /**
   * Get number of columns of images
   * @return columns
   */
  unsigned int getCols() const  { return _cols; }
  /**
   * Accessor to pointer of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords() const     { return _coords; }
  /**
   * Accessor to mask of valid points
   * @return pointer to mask
   */
  bool* getMask() const         { return _mask; }
  /**
   * Get pointer to z-Buffer
   */
  double* getZ() const          { return _z; }
  /**
   * Get Z buffer
   * @return MatD
   */
  MatD getMatZ(void) const;
  /**
   * Accessor to pointer of color data
   * @return pointer to color buffer (layout r1g1b1r2...)
   */
  unsigned char* getRGB()     { return _rgb; }
  /**
   * Get Matrix containing RGB data
   * @return MatRGB
   */
  MatRGB getMatRGB(void) const;
  /**
   * Function to get the frame rate of sensor
   * @return   frame rate in pictures per second
   */
  float getFrameRate(void) const { return _frameRate; }
  /**
   * Start serializing the data stream to file
   * @param filename name of file
   */
  void startRecording(char* filename);
  /**
   * Stop previously started recording
   */
  void stopRecording(void);
protected:
  virtual void record(void) = 0;
  /**
   * Function to estimate frame rate of grabbing
   */
  void estimateFrameRate(void);

  unsigned int    _rows;
  unsigned int    _cols;
  double*          _coords;
  double*          _z;
  bool*            _mask;
  unsigned char*  _rgb;

  Timer            _time;          ///< timer for estimation of frame rate
  float           _frameRate;     ///< frame rate of grabbing

  bool            _record;
  std::ofstream   _recfile;
};

}; // namespace



#endif /* PARENTDEVICE3D_H_ */
