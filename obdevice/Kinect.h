#ifndef KINECT_H
#define KINECT_H

#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

#include <iostream>
#include <fstream>

using namespace xn;
using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Kinect
 * @brief Kinect interface
 * @author Stefan May
 **/
class Kinect
{
public:
  /**
   * Standard constructor;
   */
    Kinect(const char* path);

  /**
   * Standard destructor
   */
  ~Kinect();

  /**
   * Grab new image
   * @return success
   */
  bool grab();

  /**
   * Start serializing the data stream to file
   * @param filename name of file
   */
  void startRecording(char* filename);

  /**
   * Stop previously started recording
   */
  void stopRecording();

  /**
   * Get number of rows of images
   * @return rows
   */
  unsigned int getRows();

  /**
   * Get number of columns of images
   * @return columns
   */
  unsigned int getCols();

  /**
   * Accessor to pointer of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords();

  /**
   * Accessor to mask of valid points
   * @return pointer to mask
   */
  bool* getMask();

  /**
   * Get pointer to z-Buffer
   */
  double* getZ();

  /**
   * Accessor to pointer of color data
   * @return pointer to color buffer (layout r1g1b1r2...)
   */
  unsigned char* getRGB();

  /**
   * Experimental: Skeleton tracking - get mask of valid points
   * @return mask
   */
  unsigned short* getUserMask();

  /**
   * Experimental: Activate bilinear Filter
   */
  void useBilinearFilter(bool activate);

private:

  /**
   * Grab new color image
   * @return success
   */
  bool grabRGB();

  /**
   * Grab new "night vision" image
   * @return success
   */
  bool grabIR();

  /**
   * Experimental: Filter depth image with bilinear filter (slow)
   */
  void filterBilinear(bool* mask, double* z_filtered);

  Context _context;

  DepthGenerator _depth;
  ImageGenerator _image;
  IRGenerator _ir;
  bool _useIR;

  XnPoint3D* _proj;
  XnPoint3D* _wrld;

  double* _coords;
  double* _z;
  bool*   _mask;
  unsigned char* _rgb;

  unsigned int _rows;
  unsigned int _cols;

  bool _init;
  bool _record;
  ofstream _recfile;

  bool _useBilinearFilter;
};

} // end namespace

#endif
