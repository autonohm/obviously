/**
* @file  CamNano.cpp
* @autor Christian Pfitzner
* @date  08.11.2012
*
* @todo   set up matrix based filtering
* @todo   set up controller for integration time according to amplitudes
*/


#include "CamNano.h"

using namespace obvious;

/*
 * Standard constructor of class CamNano
 */
CamNano::CamNano()
{
   _res = pmdOpen (&_hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);

   if (_res != PMD_OK)
   {
      LOGMSG(DBG_ERROR, "Error openning sensor");
   }
   else
     std::cout << "Opened sensor" << std::endl;

   /*
    * config pmd processing
    */
   char lens[128];
   pmdProcessingCommand(_hnd, lens, 128,  "GetLensParameters");
   pmdProcessingCommand(_hnd, 0, 0,       "SetAveraging On");
   pmdProcessingCommand(_hnd, 0, 0,       "SetAveragingFrames 2");
   pmdProcessingCommand(_hnd, 0, 0,       "SetConsistencyThreshold 0.5");
   pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterKernelSize 10");
   pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilter on");
   pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFitlerSigmaRange 15");
   pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterenhanceImage off");
   pmdProcessingCommand(_hnd, 0, 0,       "SetSignalStrengthCheck On");

   /*
    * config logging messages
    */
   LOGMSG_CONF("CamNano.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_ERROR);

	_rows   = 120;
	_cols   = 165;
	_size   = _rows * _cols;
	_points = _size;

	_coords    = new double[_size*3];

	_meanAmp      = 0.0f;
	_autoIntegrat = true;
	_frameRate    = 0.0f;
	_rawSet       = false;
	_intrinsic    = true;
	_init         = true;
	_debug        = false;

	// init of pid controller
	_ctrl.setDebug(_debug);
	_ctrl.setP(3.0f);
	_ctrl.setI(0.5f);
	_ctrl.setD(0.0f);
	_ctrl.setAWU(30.f);
	_ctrl.setSetValue(250);
	_ctrl.setMinValue(MIN_INTEGRATIONTIME);
	_ctrl.setMaxValue(MAX_INTEGRATIONTIME);
}

/*
 * Standard destructor of class CamNano
 */
CamNano::~CamNano()
{
	delete [] _coords;
	pmdClose(_hnd);
}

/*
 * Function to grab raw data from camera
 */
void CamNano::setRaw(bool raw)
{
  _rawSet = raw;
}

/*
 * Function to get number of columns
 */
unsigned int CamNano::getCols(void) const
{
	return _cols;
}

/*
 * Function to get number of rows
 */
unsigned int CamNano::getRows(void) const
{
	return _rows;
}

unsigned int CamNano::getSize(void) const
{
  return _points;
}
/*
 * Function to grab data from camera
 */
bool CamNano::grab()
{
  if (!_init)
  {
    LOGMSG(DBG_ERROR, "Sensor uninitialized");
    pmdClose(_hnd);
    return(false);
  }

  this->estimateFrameRate();

  //_coords  = new double[_size*3];

  _res = pmdUpdate(_hnd);
  if (_res != PMD_OK)
  {;
    LOGMSG(DBG_ERROR, "Error updating sensor");
    pmdClose (_hnd);
    return(false);
  }

	_res = pmdGetSourceDataDescription(_hnd, &_dd);
	if (_res != PMD_OK)
	{
	  LOGMSG(DBG_ERROR, "Error updating data description");
	  pmdClose (_hnd);
	  return(false);
	}

	float* coordsTmp  	= new float [_size * 3];
	float     dist[_size];
	float     amp[_size];

	_res = pmdGetDistances(_hnd, dist, sizeof(dist));
	_res = pmdGetAmplitudes(_hnd, amp, sizeof(amp));
	_res = pmdGet3DCoordinates(_hnd, coordsTmp, _size * sizeof(float) * 3);

	if (_rawSet)
	  this->noFilterPoints(coordsTmp);
	else
	  this->filterPoints(coordsTmp, dist, amp);

	delete [] coordsTmp;

	if (_res != PMD_OK)
	{
	  LOGMSG(DBG_ERROR, "Error filtering data");
	  pmdClose (_hnd);
	  return(false);
	}

	if (_autoIntegrat)
	  this->setAutoIntegration();
	return(true);
}

/*
 * Function to set integration time
 */
void CamNano::setIntegrationTime(unsigned value)
{
  _res = pmdSetIntegrationTime(_hnd, 0, value);
  if (_res != PMD_OK)
    LOGMSG(DBG_ERROR, "Wrong integration time");
}


void CamNano::setIntegrationAuto(bool autom)
{
  _autoIntegrat = autom;
}

/*
 * Function to show set parameters of camera
 */
void CamNano::showParameters(void)
{
   unsigned i;
   char serialNr[128];
   _res = pmdGetIntegrationTime (_hnd, &i, 0);
   _res = pmdSourceCommand(_hnd, serialNr, sizeof(serialNr), "GetSerialNumber");

   std::cout << "Serial number of device: " << serialNr           << std::endl;
   std::cout << "Integration time: "     << i <<  " microseconds" << std::endl;
   std::cout << "Modulation frequency: 30 MHz"                    << std::endl;
}

/*
 * Function to return data array
 */
double* CamNano::getCoords() const
{
	return _coords;
}

/*
 * Function to return frame rate
 */
float CamNano::getFrameRate(void) const
{
  return _frameRate;
}

/*
 * Function to activate debug mode
 */
void CamNano::setDebug(bool debug)
{
  _debug = debug;
}

/*-----------------------------------------------------------------------------
 * Private functions
 */
/*
 * Function to filter invalid points
 */
void CamNano::filterPoints(const float* points, const float* dist, const float* amp)
{
  float ampSum;
  /*
   * Check data for valid points with the help of distances and amplitude
   */
  unsigned int i = 0, j = 0;
  for (unsigned int k = 0 ; k < _size*3 ; )
  {
    // filter points distance distance
     if (dist[j] <= DIST_THRESHOLD_MAX && dist[j] >= DIST_THRESHOLD_MIN)
     {
       // filter points with amplitude
       if (amp[j] >= AMP_THRESHOLD)
       {
         /*
          * saving points to member variable
          *
          * NOTE: image is mirrored so her we mirror it again to get the right
          * perspective by changing the leading sign of x
          */
        _coords[i]    = -points[k];     // x
        _coords[i+1]  = points[k+1];    // y
        _coords[i+2]  = points[k+2];    // z
        ampSum += amp[j];
        i += 3;
       }
     }
     k += 3; j += 1;
  }

  _points = i / 3;

  if (i != 0)
    _meanAmp = ampSum / i;
  else
    _meanAmp = 0;

  if (_debug)
    std::cout << "Mean amplitude: " <<  _meanAmp << std::endl;
}

/*
 * Function to set points unfiltered to member
 */
void CamNano::noFilterPoints(const float* points)
{
  unsigned int i = 0;
  for (unsigned int k = 0 ; k < _size*3 ; k++ )
  {
    _coords[i] = points[k];
    i += 1;;
  }
}

/*
 * Function to estimate frame rate
 */
void CamNano::estimateFrameRate(void)
{
  static long double oldTime;
  long double newTime = _time.getTime();

  _frameRate = 1 / (newTime - oldTime) * 1000;
  oldTime = newTime;
}

void CamNano::setAutoIntegration(void)
{
  float time;
  _ctrl.setSetValue(300);
  time = _ctrl.controll(_meanAmp);
  setIntegrationTime(time);
  if (_debug)
    this->showParameters();
}

