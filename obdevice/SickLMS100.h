/*
 * SickLMS100.h
 *
 *  Created on: 11.03.2013
 *      Author: christian
 */



#ifndef SICKLMS100_H_
#define SICKLMS100_H_

#include "obdevice/LaserDevice.h"
#include <LMS1xx.h>

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class LaserDevice
 */
class SickLMS100 : public LaserDevice
{
public:
  /**
   * Standard Constructor
   */
  SickLMS100(double minAngle=-135.0, double maxAngle=135.0, unsigned int rays = 1);
  /**
   * Default Destructor
   */
  virtual   ~SickLMS100();
  /**
   * Function to grab new data
   * @return  TRUE if success
   */
  virtual bool      grab(void);
private:
  void estimateAngularRes(void);
  /**
   * Function to estimate ranges in scan
   */
  void estimateRanges(void);
  /**
   * Function to estimate intensities in scan
   */
  virtual void estimateIntensities(void);
  /**
   * Function to estimate single angles for every ray
   */
  virtual void estimateAngles(void);
  /**
   * Function to estimate 2D coords
   */
  virtual void estimateCoords2D(void);
  /**
   * Function to estimate 3D coords
   */
  virtual void estimateCoords3D(void);
  /**
   * Function to estimate normals
   */
  virtual void estimateNormals(void);
  /**
   * Function to estimate mask
   */
  virtual void estimateMask(void);

  LMS1xx      _laser;
  scanCfg     _cfg;
  scanDataCfg _dataCfg;
  scanData    _data;


};

};  //namespace


#endif /* SICKLMS100_H_ */
