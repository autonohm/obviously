#ifndef UVCVIRTUALCAM_H
#define UVCVIRTUALCAM_H

#include "obdevice/UvcCam.h"

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class UvcVirtualCam
 * @brief Class encapsulates virtual UVC camera device with subsampled resolution
 * @author Stefan May, Philipp Koch
 **/
class UvcVirtualCam
{

public:

  /**
   * Standard constructor
   */
  UvcVirtualCam(const char* dev, unsigned int maxWidth, unsigned int maxHeight, EnumCameraColorMode mode);

  /**
   * Standard destructor
   */
  ~UvcVirtualCam();

  /**
   * Get width of virtual camera image
   * @return width
   */
  unsigned int getWidth();

  /**
   * Get height of virtual camera image
   * @return height
   */
  unsigned int getHeight();

  /**
   * Set scale factor (image must be a multiple in each dimension)
   * @param scale downsampling factor in each dimension
   * @return success
   */
  EnumCameraError setScale(unsigned int scale);

  /**
   * Set color mode of returned images
   * @param mode color mode
   */
  void setColorMode(EnumCameraColorMode mode);

  /**
   * Get number of image channels, i.e., RGB=3, Grayscale=1
   * @return image channels
   */
  unsigned int getChannels();

  /**
   * Grab images from device in non-blocking mode.
   * @param img Pointer to image buffer (must be instantiated externally)
   * @return Grabbing state
   */
  EnumCameraError grab(unsigned char* img);

private:

  void average(unsigned char* img, unsigned int cols, unsigned int rows, unsigned int scale);

  UvcCam* _cam;
  unsigned char* _buf;
  unsigned char* _bufR;
  unsigned char* _bufG;
  unsigned char* _bufB;
  unsigned int* _bufI;

  unsigned int _maxWidth;
  unsigned int _maxHeight;
  unsigned int _scale;
};

}

#endif
