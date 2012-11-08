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
   _res = pmdOpen (&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);

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
   pmdProcessingCommand(hnd, lens, 128, "GetLensParameters");
   pmdProcessingCommand(hnd, 0, 0, "SetAveraging On");
   pmdProcessingCommand(hnd, 0, 0, "SetAveragingFrames 2");
   pmdProcessingCommand(hnd, 0, 0, "SetConsistencyThreshold 0.5");
   pmdProcessingCommand(hnd, 0, 0, "SetBilateralFilterKernelSize 10");
   pmdProcessingCommand(hnd, 0, 0, "SetBilateralFilter on");
   pmdProcessingCommand(hnd, 0, 0, "SetBilateralFitlerSigmaRange 15");
   pmdProcessingCommand(hnd, 0, 0, "SetBilateralFilterenhanceImage off");
   pmdProcessingCommand(hnd, 0, 0, "SetSignalStrengthCheck On");

   std::cout << "Camera parameter: " << *lens << std::endl;
   /*
    * config logging messages
    */
   LOGMSG_CONF("CamNano.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_ERROR);

	_rows = 120;
	_cols = 165;
	_size = _rows * _cols;

	_coords  = new double[_size*3];

	_intrinsic = true;
	_init      = true;
}

/*
 * Standard destructor of class CamNano
 */
CamNano::~CamNano()
{
	delete [] _coords;
	pmdClose (hnd);
}

/*
 * Function to get number of columns
 */
unsigned int CamNano::getCols() const
{
	return _cols;
}

/*
 * Function to get number of rows
 */
unsigned int CamNano::getRows() const
{
	return _rows;
}

/*
 * Function to grab data from camera
 */
bool CamNano::grab()
{
  if (!_init)
  {
    LOGMSG(DBG_ERROR, "Sensor uninitialized");
    pmdClose(hnd);
    return(false);
  }

  _coords  = new double[_rows*_cols*3];
  _res = pmdUpdate(hnd);
  if (_res != PMD_OK)
  {;
    LOGMSG(DBG_ERROR, "Error updating sensor");
    pmdClose (hnd);
  }

	_res = pmdGetSourceDataDescription(hnd, &dd);
	if (_res != PMD_OK)
	{
	  LOGMSG(DBG_ERROR, "Error updating data description");
	  pmdClose (hnd);
	  return false;
	}

	float* _coordsF  	= new float [_size * 3];

	float     dist[_size];
	float     amp[_size];

	_res = pmdGetDistances(hnd, dist, sizeof(dist));
	_res = pmdGetAmplitudes(hnd, amp, sizeof(amp));
	_res = pmdGet3DCoordinates(hnd, _coordsF, _size * sizeof (float) * 3);

	/*
	 * Check data for valid points
	 */
	unsigned int i = 0, j = 0;
	for (unsigned int k = 0 ; k < _size*3 ; )
	{
	   if (dist[j] <= 2.0 && dist[j] >= 0.05)
	   {
       if (amp[j] >= 100)
       {
        _coords[i]    = _coordsF[k];      // x
        _coords[i+1]  = _coordsF[k+1];    // y
        _coords[i+2]  = _coordsF[k+2];    // z
        i += 3;
       }
	   }
	   k += 3;
	   j += 1;
	}

	delete [] _coordsF;

	if (_res != PMD_OK)
	{
	  LOGMSG(DBG_ERROR, "Error filtering data");
	  pmdClose (hnd);
	  return false;
	}
	return(true);
}

/*
 * Function to grab raw data from camera
 */
bool CamNano::grabRaw(void)
{

  return(true);
}

/*
 * Function to set integration time
 */
void CamNano::setIntegrationTime(unsigned value)
{
  _res = pmdSetIntegrationTime(hnd, 0, value);
  if (_res != PMD_OK)
    LOGMSG(DBG_ERROR, "Wrong integration time");
}

/*
 * Function to show set parameters of camera
 */
void CamNano::showParameters(void)
{
   unsigned i;
   char serialNr[128];
   _res = pmdGetIntegrationTime (hnd, &i, 0);
   _res = pmdSourceCommand(hnd, serialNr, sizeof(serialNr), "GetSerialNumber");

   std::cout << "Serial number of device: " << serialNr           << std::endl;
   std::cout << "Integration time: "     << i <<  " microseconds" << std::endl;
   std::cout << "Modulation frequency: 30 MHz"                    << std::endl;
}

/*
 * Function to return data array
 */
double* CamNano::getCoords()
{
	return _coords;
}

/*-----------------------------------------------------------------------------
 * Private functions
 */
/*
 * Function to filter invalid points
 */
void CamNano::filterPoints(double& points)
{

}
