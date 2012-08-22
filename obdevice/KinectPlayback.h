#ifndef KINECTPLAYBACK_H
#define KINECTPLAYBACK_H

#include <iostream>
#include <fstream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class KinectPlayback
 * @brief Kinect file interface
 * @author Stefan May
 **/
class KinectPlayback
{
public:
  /**
   * Standard constructor;
   */
  KinectPlayback(const char* filename);

  /**
   * Standard destructor
   */
  ~KinectPlayback();

  /**
   * Grab new image
   * @return success
   */
  bool grab();

  /**
   * Signals end of file. Replay will start again from the beginning when calling reset.
   * @return end of file flag
   */
  bool eof();

  /**
   * Resets playback to the beginning of file stream.
   */
  void reset();

  /**
   * Skip a number of frames
   * @params frames number of frames
   */
  void skip(unsigned int frames);

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
   * Accessor to point of coordinate data
   * @return pointer to coordinate buffer (layout x1y1z1x2...)
   */
  double* getCoords();

  /**
   * Accessor to mask of valid points
   * @return pointer to mask
   */
  unsigned char* getMask();

  /**
   * Accessor to array of distance data
   * @return pointer to z-array
   */
  double* getZ();

  /**
   * Accessor to point of color data
   * @return pointer to color buffer (layout r1g1b1r2...)
   */
  unsigned char* getRGB();

private:
  double* _coords;
  double* _z;
  unsigned char* _rgb;
  unsigned char* _mask;

  unsigned int _rows;
  unsigned int _cols;

  bool _init;

  ifstream _recfile;
  unsigned int _frames;
  unsigned int _frame;

  bool _eof;
};

} // end namespace

#endif
