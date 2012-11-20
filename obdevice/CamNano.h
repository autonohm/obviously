/**
* @file  CamNano.h
* @autor Christian Pfitzner
* @date  08.11.2012
*
*
*/

#ifndef __CAMNANO__
#define __CAMNANO__

#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/PID_Controller.h"

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cstring>

#include <pmdsdk2.h>

/// @def source plugin for cam board
#define SOURCE_PLUGIN "camboardnano"
/// @def source param for cam board
#define SOURCE_PARAM  ""
/// @def proc plugin for cam board
#define PROC_PLUGIN   "camboardnanoproc"
/// @def proc param for cam board
#define PROC_PARAM    ""

/**
 * @namespace namespace of obviously library
 */
namespace obvious {

/// @def maximum threshold for distance filter
#define DIST_THRESHOLD_MAX  2.0
/// @def minimum threshold for distance filter
#define DIST_THRESHOLD_MIN  0.05
/// @def threshold for amplitude
#define AMP_THRESHOLD       100
/// @def maximal integration time
#define MAX_INTEGRATIONTIME 2000
/// @def minimal integration time
#define MIN_INTEGRATIONTIME 100

/**
 * @class CamNano
 * Driver for PMD time of flight camera
 *
 * This driver contains common functions needed to work with the pmd
 * camboard nano. The specifications of this sensor can be seen on
 * http://www.pmdtec.com/fileadmin/pmdtec/media/PMDvision-CamBoard-nano.pdf
 */
class CamNano
{
public:
  /**
   * Standard constructors
   */
  CamNano(void);
  /**
   * Standard destructor
   */
  ~CamNano(void);
  /**
   * Get number of rows of images
   * @return rows
   */
  unsigned int getRows(void ) const;
  /**
   * Get number of columns of images
   * @return columns
   */
  unsigned int getCols(void) const;
  /**
   * Function to get size of coords array
   * @return size
   */
  unsigned int getSize(void) const;
  /**
   * Accessor to pointer of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords(void) const;
  /**
   * Function to get the frame rate of sensor
   * @return   frame rate in pictures per second
   */
  float getFrameRate(void) const ;
  /**
   * Grab new image with filtering and validation
   * @return success of grabbing
   */
  bool grab(void);
  /**
   * Function to grab new raw data from sensor (without filtering or validation)
   * @param[in]     raw     TRUE to get raw values with grab
   */
  void setRaw(bool raw = true);
  /**
   * Function to set integration time of sensor manually
   * @param[in]   value     value in microseconds
   */
  void setIntegrationTime(const unsigned value);
  /**
   * Function to call integration time automatically
   * @see setIntegrationTime
   *
   * @param[in]     auto    TRUE to set integration automatically (default: TRUE)
   */
  void setIntegrationAuto(bool autom = true);
  /**
   * Function to show parameters of TOF camera in terminal
   */
  void showParameters(void);
  /**
   * Function to set up debug
   * @param[in]     debug    TRUE to set debug mode (default: TRUE)
   */
  void setDebug(bool debug = true);

private:
  /*
   * Private functions
   */
  /**
   * Function to filter points
   * @param     points
   * @param[in] distance    distance values of sensor
   * @param[in] amplitudes  amplitude values of sensor
   */
  void filterPoints(const float* points, const float* distances, const float* amplitudes);
  /**
   * Function to write points from sensor to member variable _coords
   * @param   points
   */
  void noFilterPoints(const float* points);
  /**
   * Function to estimate frame rate of grabbing
   */
  void estimateFrameRate(void);
  /**
   * Function to set integration value automatically
   */
  void setAutoIntegration(void);
  /*
   * Private members
   */
  PMDHandle           _hnd;           ///< Handler of PMD device
  PMDDataDescription  _dd;            ///< description of PMD device
  Timer               _time;          ///< timer for estimation of frame rate
  PID_Controller      _ctrl;          ///< pid controller for automatic integration set up

  int                 _res;           ///< error buffer of PMD devices
  unsigned int        _rows;          ///< number of rows
  unsigned int        _cols;          ///< number of columns
  unsigned int        _size;          ///< size of picture
  unsigned int        _points;        ///< number of points 2d

  float               _meanAmp;       ///< mean amplitude for controller

  float               _frameRate;     ///< frame rate of grabbing
  double*             _coords;        ///< coords save as x1,y1,z1; x2, ...
  bool                _intrinsic;     ///< object gives back nondistrubed point cloud @see grub
  bool                _rawSet;        ///< TRUE for output of raw unfiltered data from sensor
  bool                _init;          ///< TRUE if camera initialized
  bool                _autoIntegrat;  ///< TRUE if auto integration timer
  bool                _debug;         ///< TRUE if debug messages should be shown in terminal
};

} // end namespace obvious

#endif /* __CAMNANO__ */

