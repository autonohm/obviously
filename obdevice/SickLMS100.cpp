/**
* @file   SickLMS100.cpp
* @author Christian Pfitzner
* @date   12.03.2013
*
*
*/

#define DEG2RAD M_PI/180.0

#include "obdevice/SickLMS100.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <iostream>
#include <stdio.h>
#include <csignal>
#include <cstdio>

#define DIST_TH 10
#define INT_TH  10


using namespace obvious;

SickLMS100::SickLMS100(double minAngle, double maxAngle, unsigned int rays)
: LaserDevice(minAngle, maxAngle, rays)
{
  std::string host = "192.168.3.50";
  _laser.connect(host);

  // check if connection is established
  if (_laser.isConnected())
  {
    _laser.login();
    _cfg = _laser.getScanCfg();
//    std::cout << "resolution : "  << (double)_cfg.angleResolution/10000.0 << " deg "  << std::endl;
//    std::cout << "frequency : "   << (double)_cfg.scaningFrequency/100.0  << " Hz "   << std::endl;
    if (_cfg.angleResolution == 2500) {
      _nrOfRays = 1081;
    }
    else if (_cfg.angleResolution == 5000) {
      _nrOfRays = 541;
    }
    else
    {
      LOGMSG(DBG_ERROR, "Unsupported resolution");
      exit(1);
    }

    _ranges       = new double[_nrOfRays];
    _intensities  = new double[_nrOfRays];
    _angles       = new double[_nrOfRays];
    _coords2D     = new double[2*_nrOfRays];
    _coords3D     = new double[3*_nrOfRays];
    _normals      = new double[2*_nrOfRays];
    _mask         = new bool  [_nrOfRays];

    _dataCfg.outputChannel  = 1;
    _dataCfg.remission      = true;
    _dataCfg.resolution     = 1;
    _dataCfg.encoder        = 0;
    _dataCfg.position       = false;
    _dataCfg.deviceName     = false;
    _dataCfg.outputInterval = 1;

    _laser.setScanDataCfg(_dataCfg);
    _laser.startMeas();

    status_t stat;
    do // wait for ready status
    {
      stat = _laser.queryStatus();
      sleep(1.0);
    }
    while (stat != ready_for_measurement);

    _laser.scanContinous(1);
  }
  else
  {
    LOGMSG(DBG_ERROR, "Connection error")
    exit(1);
  }
  this->estimateAngularRes();
  this->estimateAngles();
  LOGMSG(DBG_DEBUG, "Configuration completed");
  std::cout << _angRes << std::endl;
}

SickLMS100::~SickLMS100()
{
  _laser.scanContinous(0);
  _laser.stopMeas();
  _laser.disconnect();
}

bool SickLMS100::grab(void)
{
  if(_laser.isConnected())
  {
    _laser.getData(_data);
    this->estimateRanges();
    this->estimateIntensities();
    this->estimateCoords2D();
    this->estimateCoords3D();
  }
  else
  {
    LOGMSG(DBG_ERROR, "Connection error");
  }

  return(true);
}

void SickLMS100::estimateAngularRes(void)
{
  _angRes = (_maxAngle-_minAngle)/_nrOfRays;
}

void SickLMS100::estimateRanges(void)
{
  for(unsigned int i=0; i<_data.dist_len1; i++)
    _ranges[i] = _data.dist1[i] * 0.001;
}

void SickLMS100::estimateIntensities(void)
{
  for(unsigned int i=0; i<_data.rssi_len1; i++)
    _intensities[i] = _data.rssi1[i];
}

void SickLMS100::estimateAngles(void)
{
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
    _angles[i] = _minAngle + i*_angRes;
}

void SickLMS100::estimateCoords2D(void)
{
  unsigned int k = 0;
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
  {
    // std::cout << _angles[i] << std::endl;
    _coords2D[k+0] = _ranges[i] * cos(_angles[i]*M_PI / 180);
    _coords2D[k+1] = _ranges[i] * sin(_angles[i]*M_PI / 180);
    k+=2;
  }
}

void SickLMS100::estimateCoords3D(void)
{
  unsigned int k = 0;
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
  {
    _coords3D[k+0] = _ranges[i] * cos(_angles[i]*M_PI / 180);
    _coords3D[k+1] = _ranges[i] * sin(_angles[i]*M_PI / 180);
    _coords3D[k+2] = 1.0;
    k+=3;
  }
}

void SickLMS100::estimateNormals(void)
{

}

void SickLMS100::estimateMask(void)
{
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
  {
    if((_ranges[i] > DIST_TH) || (_intensities[i] > INT_TH))
    {
      _mask[i] = false;
    }
    else
    {
      _mask[i] = true;
    }
  }
}
