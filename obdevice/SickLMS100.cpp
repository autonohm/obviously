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

#include <pthread.h>
#include <string.h>

#define DIST_TH 10
#define INT_TH  10

using namespace obvious;

SickLMS100* _this;

pthread_t _thread;
bool _shutdown = false;
bool _run      = true;
pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;

void* taskCode(void*)
{
  while(_run)
  {
    _this->schedule();
    usleep(3000);
    //  pthread_yield();
  }

  _shutdown = true;
  //pthread_exit(NULL);

  return NULL;
}

SickLMS100::SickLMS100()
{
  _this = this;

  std::string host = "192.168.3.50";
  _laser.connect(host);

  // check if connection is established
  if (_laser.isConnected())
  {
    _laser.login();
    _cfg = _laser.getScanCfg();
    std::cout << "resolution : "  << (double)_cfg.angleResolution/10000.0 << " deg "  << std::endl;
    std::cout << "frequency : "   << (double)_cfg.scaningFrequency/100.0  << " Hz "   << std::endl;
    std::cout << "start angle : " << _cfg.startAngle << " stop angle : " << _cfg.stopAngle << std::endl;
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
      cout << "waiting ... stat = " << stat << endl;
    }
    while (stat != ready_for_measurement);

    cout << "go" << endl;
    _laser.scanContinous(1);

    pthread_create(&_thread, NULL, &taskCode, NULL);
  }
  else
  {
    LOGMSG(DBG_ERROR, "Connection error")
              exit(1);
  }

  this->calculateAngles();
  LOGMSG(DBG_DEBUG, "Configuration completed");
}

SickLMS100::~SickLMS100()
{
  _run = false;
  //pthread_join( _thread, NULL);
  while(!_shutdown)
    usleep(100);

  /*cout << "stopping continuous mode" << endl;
   _laser.scanContinous(0);

   cout << "stopping measurement" << endl;
   _laser.stopMeas();

   cout << "disconnect" << endl;
   _laser.disconnect();*/
}

double SickLMS100::getStartAngle()
{
  return deg2rad(((double)_cfg.startAngle) / 10000.0);
}

double SickLMS100::getStopAngle()
{
  return deg2rad(((double)_cfg.stopAngle) / 10000.0);
}

unsigned int SickLMS100::getNumberOfRays()
{
  return _nrOfRays;
}

double SickLMS100::getAngularRes(void)
{
  return deg2rad(((double)_cfg.angleResolution) / 10000.0);
}

double* SickLMS100::getRanges()
{
  return _ranges;
}

double* SickLMS100::getCoords()
{
  return _coords2D;
}

void SickLMS100::schedule()
{
  pthread_mutex_lock(&_mutex);
  _laser.getData(_data);
  pthread_mutex_unlock(&_mutex);
}

bool SickLMS100::grab(void)
{
  if(_laser.isConnected())
  {
    pthread_mutex_lock(&_mutex);
    memcpy(&_dataBuffer, &_data, sizeof(_dataBuffer));
    pthread_mutex_unlock(&_mutex);

    this->calculateRanges();
    this->calculateIntensities();
    this->calculateCoords2D();
  }
  else
  {
    LOGMSG(DBG_ERROR, "Connection error");
  }

  return(true);
}

void SickLMS100::calculateRanges(void)
{
  cout << _data.dist_len1 << endl;
  for(unsigned int i=0; i<_data.dist_len1; i++)
    _ranges[i] = _data.dist1[i] * 0.001;
}

void SickLMS100::calculateIntensities(void)
{
  for(unsigned int i=0; i<_data.rssi_len1; i++)
    _intensities[i] = _data.rssi1[i];
}

void SickLMS100::calculateAngles(void)
{
  double res = getAngularRes();
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
    _angles[i] = getStartAngle() + ((double)i)*res;
}

void SickLMS100::calculateCoords2D(void)
{
  unsigned int k = 0;
  for(unsigned int i=0 ; i<_nrOfRays ; i++)
  {
    // std::cout << _angles[i] << std::endl;
    _coords2D[k+0] = _ranges[i] * cos(_angles[i]);
    _coords2D[k+1] = _ranges[i] * sin(_angles[i]);
    k+=2;
  }
}

