/**
* @file   OpenNIDevice.h
* @author Christian Pfitzner
* @date   28.12.2012
*
*
*/

#ifndef OPENNIDEVICE_H_
#define OPENNIDEVICE_H_

#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

#include "obdevice/ParentDevice3D.h"

using namespace xn;
using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class OpenNIDevice
 * @brief Abstract class for devices of OpenNI
 * @see   Device3D
 */
class OpenNIDevice : public ParentDevice3D
{
public:
  /**
   * Standard constructor
   * @param     path    path to config file of OpenNI
   */
  OpenNIDevice(const char* path);
  /**
   * Default destructor
   */
  virtual ~OpenNIDevice() = 0;
  /**
   * Grab new image
   * @return success
   */
  virtual bool grab() = 0;
protected:
  /**
   * Function to grab points
   * @return  success
   */
  bool grabDistance();
  /**
   * Grab new "night vision" image
   * @return success
   */
  bool grabIR();

  void record();

  Context        _context;
  DepthGenerator _depth;
  IRGenerator    _ir;
  ImageGenerator _image;
  bool          _useIR;

  XnPoint3D* _proj;
  XnPoint3D* _wrld;

  bool      _init;

  const char*    _path;
};

}; // namespace

#endif /* OPENNIDEVICE_H_ */
