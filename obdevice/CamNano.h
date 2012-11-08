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
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <pmdsdk2.h>

#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""
#define PROC_PLUGIN "camboardnanoproc"
#define PROC_PARAM ""

namespace obvious {

/**
 * @class CamNano
 * Driver for PMD time of flight camera
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
  unsigned int getRows() const;
  /**
   * Get number of columns of images
   * @return columns
   */
  unsigned int getCols() const;
  /**
   * Accessor to pointer of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords(void);
  /**
   * Grab new image with filtering and validation
   * @return success of grabbing
   */
  bool grab(void);
  /**
   * Function to grab new raw data from sensor (without filtering or validation)
   * @return success of grabbing
   */
  bool grabRaw(void);
  /**
   * Function to set integration time of sensor manually
   * @param[in]   value     value in microsecounds
   */
  void setIntegrationTime(const unsigned value);
  /**
   * Function to show parameters of TOF camera in terminal
   */
  void showParameters(void);


private:
  /**
   * Function to filter points
   * @param     points
   */
  void filterPoints(double& points);

  /**
   * Funtion to call integration time automaticaly
   * @see setIntegrationTime
   */
  void setIntegrationAuto(void);

  PMDHandle           hnd;            ///< Handler of PMD device
  PMDDataDescription  dd;             ///< description of PMD device

  int                 _res;           ///< error buffer of PMD devices
  unsigned int        _rows;          ///< number of rows
  unsigned int        _cols;          ///< number of columns
  unsigned int        _size;          ///< size of picture

  double*             _coords;        ///< coords save as x1,y1,z1; x2, ...
  bool                _intrinsic;     ///< object gives back nondistrubed point cloud @see grub
  bool                _init;          ///< true if camera initialized


};

} // end namespace obvious

#endif /* __CAMNANO__ */

