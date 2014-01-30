/**
* @file  CamNano.h
* @autor Christian Pfitzner
* @date  08.11.2012
*
*
*/

#ifndef __CAMNANO__
#define __CAMNANO__



#include "obcore/math/PID_Controller.h"
#include "obdevice/ParentDevice3D.h"

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
#define DIST_THRESHOLD_MAX  1.0
/// @def minimum threshold for distance filter
#define DIST_THRESHOLD_MIN  0.05
/// @def threshold for amplitude
/// @def maximal integration time
#define AMP_THRESHOLD       320
#define MAX_INTEGRATIONTIME 2000
/// @def minimal integration time
#define MIN_INTEGRATIONTIME 50

/**
 * @class CamNano
 * @brief Driver for PMD time of flight camera
 * @see   Device3D
 *
 * This driver contains common functions needed to work with the pmd
 * camboard nano. The specifications of this sensor can be seen on
 * http://www.pmdtec.com/fileadmin/pmdtec/media/PMDvision-CamBoard-nano.pdf
 */
class CamNano : public ParentDevice3D
{
public:
  /**
   * Standard constructors
   */
  CamNano(void);
  /**
   * Standard destructor
   */
  virtual ~CamNano(void);

  // ---> GETTER
  /**
   * Function to return valid size of coords
   * @return  valid size
   */
  unsigned int getValidSize(void) const;
  /**
   * Function to return valid points from mask
   * @return
   */
  double* getValidCoords(void);
  /**
   * Function to return distance image from camera
   * @return    distance image in float with unit meter
   */
  double* getDistImage(void);
  /**
   * Function to return image of tof camera.
   * @return image
   */
  unsigned char* getImage(void) const;
  /**
   * Function to get integration time of camera
   */
  float getIntegrationTime(void) const;

  // ---> SETTER
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
   * Function to set threshold for jumping edge filter
   * @param[in] 			threshold		threshold in degrees
   */
  void setThresholdJumpingEdge(double threshold = 160.0);
  /**
   * Function to set up debug
   * @param[in]     debug    TRUE to set debug mode (default: TRUE)
   */
  void setDebug(bool debug = true);
  /**
   * Function to activate bilinear filter
   * @param[in] 		activate		default true
   */
  void activeBilinearFilter(bool activate = true);
  /**
   * Grab new image with filtering and validation
   * @return success of grabbing
   */
  bool grab(void);
  /**
   * Function to show parameters of TOF camera in terminal
   */
  void showParameters(void);
private:
  void record(void) {};
  /**
   * Function to filter points
   * @param     points
   * @param[in] distance    distance values of sensor
   * @param[in] amplitudes  amplitude values of sensor
   */
  void filterPoints(const float* points, const float* distances, const float* amplitudes);

  void filterBilinear(bool* mask, double* z_filtered);
  /**
   * Function to set integration value automatically
   */
  void setAutoIntegration(void);
  /*
   * Private members
   */
  PMDHandle           _hnd;           ///< Handler of PMD device
  PMDDataDescription  _dd;            ///< description of PMD device

  PID_Controller      _ctrl;          ///< pid controller for automatic integration set up

  int                 _res;           ///< error buffer of PMD devices
  unsigned int        _points;        ///< number of points 2d

  float               _meanAmp;       ///< mean amplitude for controller
  float               _intTime;
  float*              _dist;          //!< distance image
  float*              _amp;           //!< amplitude image

  float*              _coordsF;
  double*             _coordsV;

  double							_jumpingEdgeTH; //!< threshold for jumping edges in degrees

  unsigned char*      _image;         ///< 2d image
  float*              _imageF;

  // ---> FLAGS
  bool                _intrinsic;     		///< object gives back nondistrubed point cloud @see grub
  bool                _rawSet;        		///< TRUE for output of raw unfiltered data from sensor
  bool                _init;          		///< TRUE if camera initialized
  bool                _autoIntegrat;  		///< TRUE if auto integration timer
  bool                _debug;         		///< TRUE if debug messages should be shown in terminal
  bool 								_useBilinearFilter; //
};

} // end namespace obvious

#endif /* __CAMNANO__ */

