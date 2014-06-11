/**
* @file  CamNano.cpp
* @autor Christian Pfitzner
* @date  08.11.2012
*
*/


#include "CamNano.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <math.h>


using namespace obvious;

/*
 * Standard constructor of class CamNano
 */
CamNano::CamNano()
  : ParentDevice3D(165,120)
{
   _res = pmdOpen(&_hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);
   sleep(2);
   if (_res != PMD_OK)
   {
     std::cout << "Error code: " <<_res << std::endl;
     LOGMSG(DBG_ERROR, "Error openning sensor");
   }
   else
     LOGMSG(DBG_DEBUG, "Opened sensor");


  /*
  * config pmd processing
  */
  char lens[128];
  pmdProcessingCommand(_hnd, lens, 128,  "GetLensParameters");
  pmdProcessingCommand(_hnd, 0, 0,       "SetAveraging On");
  pmdProcessingCommand(_hnd, 0, 0,       "SetAveragingFrames 1");
  pmdProcessingCommand(_hnd, 0, 0,       "SetConsistencyThreshold 0.5");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterKernelSize 5");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilter on");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFitlerSigmaRange 30");
  pmdProcessingCommand(_hnd, 0, 0,			 "SetBilateralFilterSigmaSpatial 5.0");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterenhanceImage on");
  pmdProcessingCommand(_hnd, 0, 0,       "SetSignalStrengthCheck On");
  pmdSourceCommand(_hnd, 0, 0, "SetPriority 3");


  /*
  * config logging messages
  */
  //LOGMSG_CONF("CamNano.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_ERROR);

  _points    = _rows*_cols;
  _coordsV   = new double[_points*3];
  _coordsF   = new float [_points*3];
  _image     = new unsigned char [3*_points];
  _imageF    = new float [_points];
  _dist      = new float [_points];
  _amp       = new float [_points];

  _meanAmp      = 0.0f;
  _intTime      = 0.0f;

  // init flags
  _autoIntegrat 		 = true;
  _rawSet       		 = false;
  _intrinsic    		 = true;
  _init         		 = true;
  _debug        		 = false;
  _useBilinearFilter = false;

  _jumpingEdgeTH = 155;


  // init of pid controller for auto integration time
  _ctrl.setDebug(_debug);
  _ctrl.setP(2.0f);
  _ctrl.setI(0.5f);
  _ctrl.setD(0.0f);
  _ctrl.setAWU(30.f);
  _ctrl.setSetValue(200);
  _ctrl.setMinValue(MIN_INTEGRATIONTIME);
  _ctrl.setMaxValue(MAX_INTEGRATIONTIME);
}

/*
 * Standard destructor of class CamNano
 */
CamNano::~CamNano()
{
  //delete [] _coords;
  delete [] _coordsF;
  delete [] _image;
  delete [] _imageF;
  delete [] _coordsV;
  delete [] _dist;
  delete [] _amp;
  pmdClose(_hnd);
}

double* CamNano::getDistImage()
{
  double* _distD = new double [_cols * _rows];
  for(unsigned int i=0 ; i<_cols*_rows ; i++)
    _distD[i] = _dist[i];
  return _distD;
}

unsigned int CamNano::getValidSize(void) const
{
  return _points;
}

/*
 * Function to grab raw data from camera
 */
void CamNano::setRaw(bool raw)
{
  _rawSet = raw;
}

/*
 * Function to grab data from camera
 */
bool CamNano::grab()
{
  // check if sensor initialized
  if (!_init)
  {
    LOGMSG(DBG_ERROR, "Sensor uninitialized");
    pmdClose(_hnd);
    return(false);
  }

  this->estimateFrameRate();

  // update sensor
  _res = pmdUpdate(_hnd);
  if (_res != PMD_OK)
  {
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

  // get array with distances
  _res = pmdGetDistances(_hnd, _dist, _rows*_cols * sizeof(float));
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting the distance");
    pmdClose(_hnd);
    return(false);
  }

  // get array with amplitudes from sensor
  _res = pmdGetAmplitudes(_hnd, _amp, _rows*_cols * sizeof(float));
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting the amplitudes");
    pmdClose(_hnd);
    return(false);
  }

  // calculate 3d coordinates
  _res = pmdGet3DCoordinates(_hnd, _coordsF, _rows*_cols * sizeof(float) * 3);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting coordinates");
    pmdClose(_hnd);
    return(false);
  }

  _imageF = _amp;

  unsigned int maxval = 0;
  unsigned int minval = 10000;
  for (unsigned int i=0; i<_rows*_cols; i++) {
    if(_amp[i] > maxval)
      maxval = _amp[i];
    if(_amp[i] < minval)
      minval = _amp[i];
  }


  /*
   * Jumping edge filter based on
   * S. May. 3D Time-of-Flight Ranging for Robotic Perception in Dynamic Environments (Dissertation).
   * Fortschr.-Ber. VDI Reihe 10 Nr. 798. Duesseldorf: VDI Verlag, 2009
   */
  unsigned int k=0;
  for(unsigned int i=0 ; i<_rows*_cols*3 ; i+=3, k++)
  {
    _coords[i]    = -(double)_coordsF[i];
    _coords[i+1]  = (double)_coordsF[i+1];
    _coords[i+2]  = (double)_coordsF[i+2];
    _rgb[i]       = (_amp[k]-minval) * 255 / (maxval);
    _rgb[i+1]     = (_amp[k]-minval) * 255 / (maxval);
    _rgb[i+2]     = (_amp[k]-minval) * 255 / (maxval);
    _z[k]         = _coords[i+2] / 1000.0;

    double n[24];

    int idx[8];
    idx[0] = i-_rows-3;       // up left
    idx[1] = i-_rows  ;       // up
    idx[2] = i-_rows+3;       // up right
    idx[3] = i-3;             // left
    idx[4] = i+3;             // right
    idx[5] = i+_rows-3;       // down left
    idx[6] = i+_rows  ;       // down
    idx[7] = i+_rows+3;       // down right

    // caluclate indices of neighbours
    for(unsigned int j=0 ; j<8 ; j++) {
      if(idx[j]>=0 && idx[j] <=(int)(_rows*_cols*3))
        memcpy(&n[3*j],      &(_coords[idx[j]]), 3*sizeof(double));
    }

    double dist      = abs3D(&_coords[i]);
    double alpha_max = 0.0;
    // check for eigth neighbours
    for(unsigned int j=0 ; j<8 ; j++)
    {
      double n2[3];
      n2[0] = n[3*j+0]  - _coords[i+0];
      n2[1] = n[3*j+1]  - _coords[i+1];
      n2[2] = n[3*j+2]  - _coords[i+2];

      double dotProduct = dot3<double>(&_coords[i], n2);
      if(dotProduct<0)
      {
        double alpha = acos(dotProduct / (abs3D(n2) * dist));
        if(alpha_max<alpha) alpha_max = alpha;
      }
    }
    bool angleFilter = false;
    if(alpha_max>deg2rad(_jumpingEdgeTH))
      angleFilter = true;

    bool edge = false;
    if (i%(3*_cols)==0 || i%(3*_cols)==3 || i%(3*_cols)==6 || i%(3*_cols)==9 || i%(3*_cols)==12)
        edge = true;

    _mask[k]      = (!isnan(_z[k]))                &&
                    (_amp[k]>AMP_THRESHOLD)        &&
                    (_z[k] < DIST_THRESHOLD_MAX)   &&
                    !angleFilter && !edge;
                    /*
                    (_z[k] > DIST_THRESHOLD_MIN); */
  }
  // bilinear filter
  if (_useBilinearFilter)
  	this->filterBilinear(_mask, _z);

  // auto intregration time for tof
  if (_autoIntegrat)
    this->setAutoIntegration();
  return(true);
}

/*
 * Function to get valid coords
 */
double* CamNano::getValidCoords(void)
{
  unsigned int v=0;
  for(unsigned int i=0 ; i<_rows*_cols ; i++)
  {
    if(_mask[i] == true)
    {
      _coordsV[v]   = _coords[3*i];
      _coordsV[v+1] = _coords[3*i+1];
      _coordsV[v+2] = _coords[3*i+2];
      v+=3;
    }
  }
  _points = v/3;
  return(_coordsV);
}

/*
 * Function to set integration time
 */
void CamNano::setIntegrationTime(unsigned value)
{
  _res = pmdSetIntegrationTime(_hnd, 0, value);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Wrong integration time");
  }
  else
    _intTime = value;

  _autoIntegrat = false;
}

/*
 * Function to switch between autonomous and static integration time
 */
void CamNano::setIntegrationAuto(bool autom)
{
  _autoIntegrat = autom;
}

void CamNano::setThresholdJumpingEdge(double threshold)
{
	if(threshold <= 180)
		_jumpingEdgeTH = threshold;
	else
		LOGMSG(DBG_ERROR, "Threshold must be lower 180 Degrees");
}


/*
 * Function to get integration Time
 */
float CamNano::getIntegrationTime(void) const
{
  return (_intTime);
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

void CamNano::filterBilinear(bool* mask, double* z_filtered)
{
//  DepthMetaData depthMD;
  int mini, maxi;
  int minj, maxj;
  unsigned int radius = 4; // pixel radius?
  double val;
  double totalw;

  double distance_dim;
  double distance_range;
  double sigmad = 1.0/(3.0*3.0);
  double sigmar = 1.0/(30.0*30.0);

//  _depth.GetMetaData(depthMD);

	#pragma omp parallel
	{
  #pragma omp for
  for (unsigned int i=0; i<_rows; i++)
  {
     mini = i - radius;
     maxi = i + radius;
     if (mini < 0) mini = 0;
     if (maxi >= (int)_rows) maxi = _rows - 1;

     for (unsigned int j=0; j<_cols; j++)
     {
        minj = j - radius;
        maxj = j + radius;
        if (minj < 0) minj = 0;
        if (maxj >= (int)_cols) maxj = _cols - 1;

        if (!mask[i*_cols+j]) continue;

        val = 0; totalw = 0;

        // get depth value of pixel
        double depthCenter = _coords[3*(i*_rows+j)+2];
        for (int k=mini; k<= maxi; k++)
        {
           double distRow2 = (i - k)*(i - k);
           for (int l=minj; l<=maxj; l++)
           {
              if (!mask[k*_cols+l]) continue;
              double depthNeighbor = _coords[3*(i*_rows+j)+2];

              distance_dim   = distRow2 + (j - l)*(j - l) * sigmad;
              distance_range = (depthCenter - depthNeighbor)*(depthCenter - depthNeighbor) * sigmar;

              double w = exp(-0.5 * (distance_dim + distance_range));

              val += w * depthNeighbor;
              totalw += w;
           }
        }

        val /= totalw;
        z_filtered[i*_cols+j] = val;
     }
  }
	}
}

/*
 * Function to return image array
 */
unsigned char* CamNano::getImage(void) const
{
  float minMag = NAN;
  float maxMag = 0;
  for (unsigned int i=0 ; i<_rows*_cols ; i++)
  {
    if (_imageF[i] < minMag)
      minMag = _imageF[i];
    if (_imageF[i] > maxMag)
      maxMag = _imageF[i];
  }

  float range = minMag - maxMag;
  for (unsigned int i=0 ; i < _rows*_cols ; i++)
  {
     _image[3*i]   = (unsigned char)((_imageF[3*i]   - minMag)/range*255);
     _image[3*i+1] = (unsigned char)((_imageF[3*i+1] - minMag)/range*255);
     _image[3*i+2] = (unsigned char)((_imageF[3*i+2] - minMag)/range*255);

  }
  return _image;
}

/*
 * Function to activate debug mode
 */
void CamNano::setDebug(bool debug)
{
  _debug = debug;
}

/*
 * Function to set automatic integration time
 */
void CamNano::setAutoIntegration(void)
{
  _ctrl.setSetValue(300);
  _intTime = _ctrl.control(_meanAmp);
  setIntegrationTime(_intTime);
  if (_debug)
    this->showParameters();
}

void CamNano::activeBilinearFilter(bool activate)
{
	_useBilinearFilter = activate;
}


