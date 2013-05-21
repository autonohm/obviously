/*
 * SickLMS100.h
 *
 *  Created on: 11.03.2013
 *      Author: christian
 */



#ifndef SICKLMS100_H_
#define SICKLMS100_H_

#include <LMS1xx.h>

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class LaserDevice
 */
class SickLMS100
{
public:
   /**
    * Standard Constructor
    */
   SickLMS100();

   /**
    * Default Destructor
    */
   virtual   ~SickLMS100();

   double getStartAngle();

   double getStopAngle();

   unsigned int getNumberOfRays();

   double getAngularRes(void);

   double* getRanges();

   double* getCoords();

   /**
    * Function to grab new data
    * @return  TRUE if success
    */
   virtual bool      grab(void);

   void schedule();

private:

   /**
    * Function to estimate ranges in scan
    */
   void calculateRanges(void);

   /**
    * Function to estimate intensities in scan
    */
   void calculateIntensities(void);

   /**
    * Function to estimate single angles for every ray
    */
   void calculateAngles(void);

   /**
    * Function to estimate 2D coords
    */
   void calculateCoords2D(void);

   LMS1xx      _laser;
   scanCfg     _cfg;
   scanDataCfg _dataCfg;
   scanData    _data;
   scanData    _dataBuffer;
   unsigned int _nrOfRays;

   double*   _ranges;            //!< Distance in meters
   double*   _intensities;       //!< Intensities
   double*   _coords2D;          //!< 2D coords
   double*   _normals;           //!< normals
   double*   _angles;            //!< Angles in rad
   bool*     _mask;              //!< mask for valid or invalid points

};

};  //namespace


#endif /* SICKLMS100_H_ */
