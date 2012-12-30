/**
* @file OpenNIDeviceRGB.h
* @autor christian
* @date  30.12.2012
*
*
*/

#ifndef OPENNIDEVICERGB_H_
#define OPENNIDEVICERGB_H_

#include "obdevice/OpenNIDevice.h"
#include "obcore/math/MatRGB.h"

/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class OpenNIDeviceRGB
 * @brief Function derived from OpenNIDevice for added rgb image
 * @see   OpenNIDevice
 */
class OpenNIDeviceRGB : public OpenNIDevice
{
public:
  /**
   * Standard constructor
   */
  OpenNIDeviceRGB(void);
  /**
   * Default destructor
   */
  virtual ~OpenNIDeviceRGB(void);
  /**
   * Accessor to pointer of color data
   * @return pointer to color buffer (layout r1g1b1r2...)
   */
  unsigned char* getRGB() const { return _rgb; }
  /**
   * Get Matrix containing RGB data
   * @return MatRGB
   */
  MatRGB getMatRGB(void) const;
protected:
  /**
   * Grab new color image
   * @return success
   */
  bool grabRGB(void);
  /**
   * Function to record data to specified file
   */
  void record(void);

  ImageGenerator    _image;
  unsigned char*  _rgb;
private:
};
};

#endif /* OPENNIDEVICERGB_H_ */
