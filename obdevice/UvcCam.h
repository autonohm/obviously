#ifndef UVCCAM_H
#define UVCCAM_H

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

#define LIMITVALUE(x)  ((x)>0xffffff?0xff: ((x)<=0xffff?0:((x)>>16)))
#define NB_BUFFERS 4

enum EnumCameraError { CAMSUCCESS, CAMGRABBING, CAMERRORINIT, CAMFAILURE };
enum EnumCameraColorMode { CAMRGB, CAMGRAYSCALE };

/**
 * @class UvcCam
 * @brief Class encapsulates local camera device handling with uvc chipset
 * @author Stefan May
 **/
class UvcCam
{

public:

  /**
   * Standard constructor
   * Call UvcCam::connect() subsequently for connecting and initializing the device.
   * @param dev file path to device
   * @param width width
   * @param height height
   */
  UvcCam(const char* dev, unsigned int width=640, unsigned int height=480);

  /**
   * Standard destructor
   */
  ~UvcCam();

  static void FindDevice(const char* serial, char* &path);

  /**
   * Get image width, i.e., number of columns
   * @return image width
   */
  unsigned int getWidth();

  /**
   * Get image height, i.e., number of rows
   * @return image height
   */
  unsigned int getHeight();

  /**
   * Set color mode, i.e, rgb or grayscale
   * @param mode color mode
   */
  void setColorMode(EnumCameraColorMode mode);

  /**
   * Query current color mode
   * @return color mode
   */
  EnumCameraColorMode getColorMode();

  /**
   * Image formats
   * @return either V4L2_PIX_FMT_MJPEG or V4L2_PIX_FMT_YUYU
   */
  unsigned int getFormat();

  /**
   * Get number of image channels
   * @return channels (rgb=3, grayscale=1)
   */
  unsigned int getChannels();

  /**
   * Grab images from device in non-blocking mode.
   * @param img Pointer to image buffer (must be instantiated externally)
   * @return Grabbing state
   */
  EnumCameraError grab(unsigned char* img, unsigned int* bytes = NULL);

  /**
   * Opens the connection to the UVC camera and initializes the device with
   * the settings specified in the constructor.
   * If the process fails for whatever reason, a stdout report line is printed and
   * the device handle is closed and set back to INVALID_DEVICE again.
   * @return CAMSUCCESS if connection was established successfully, CAMERRORINIT otherwise.
   */
  EnumCameraError connect();

  EnumCameraError disconnect();

  void resetControls();

  EnumCameraError startStreaming();

  EnumCameraError stopStreaming();

  EnumCameraError setFormat(unsigned int width, unsigned int height, unsigned int format = V4L2_PIX_FMT_YUYV);

  /**
   * Print available image formats to stdout
   */
  EnumCameraError printAvailableFormats();

  /**
   * Set frame rate numerator/denominator
   * @param numerator
   * @param denominator
   */
  EnumCameraError setFramerate(unsigned int numerator, unsigned int denominator);

private:

  EnumCameraError mapMemory();

  void unmapMemory();

  /**
   * Convert an image in YUV format to RGB format
   * @param input_ptr Source buffer with yuv image
   * @param output_ptr Destination buffer to store rgb data (must be instantiated externally).
   * @param image_width Width of source image
   * @param image_height Height of source image
   * @return Status of conversion. 0 if image could not be successfully converted.
   */
  unsigned int Pyuv422torgb24(unsigned char* input_ptr, unsigned char* output_ptr, unsigned int image_width, unsigned int image_height);

  /**
   * Convert an image in YUV format to grayscale format
   * @param input_ptr Source buffer with yuv image
   * @param output_ptr Destination buffer to store grayscale data (must be instantiated externally).
   * @param image_width Width of source image
   * @param image_height Height of source image
   * @return Status of conversion. 0 if image could not be successfully converted.
   */
  unsigned int Pyuv422togray(unsigned char* input_ptr, unsigned char* output_ptr, unsigned int image_width, unsigned int image_height);

  int isv4l2Control(int nHandle, int control, struct v4l2_queryctrl *queryctrl);

  int v4l2GetControl(int nHandle, int control);

  int v4l2SetControl(int nHandle, int control, int value);

  int v4l2ResetControl(int nHandle, int control);

  /**
   * Handle to video device
   */
  int _nDeviceHandle;

  /**
   * File path of video device
   */
  char* _dev;

  /**
   * Color mode, i.e., GrayScale, RGB, ...
   */
  EnumCameraColorMode _colorMode;

  /**
   * Image width
   */
  unsigned int _width;

  /**
   * Image height
   */
  unsigned int _height;

  int _nb_buffers;
  struct v4l2_buffer _buf;
  struct buffer* _mem;
  struct v4l2_requestbuffers _rb;
  int* _LutRv;
  int* _LutGu;
  int* _LutGv;
  int* _LutBu;
  unsigned int _format;
};

}

#endif
