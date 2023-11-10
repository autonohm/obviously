#include "UvcVirtualCam.h"

namespace obvious
{

UvcVirtualCam::UvcVirtualCam(const char* dev, unsigned int maxWidth, unsigned int maxHeight, EnumCameraColorMode mode)
{
  _cam = new UvcCam(dev, maxWidth, maxHeight);
  _cam->connect();
  _cam->setFormat(maxWidth, maxHeight, V4L2_PIX_FMT_YUYV);
  _cam->setColorMode(mode);
  _cam->startStreaming();

  _buf  = new unsigned char[maxWidth*maxHeight*3];
  _bufR = new unsigned char[maxWidth*maxHeight];
  _bufG = new unsigned char[maxWidth*maxHeight];
  _bufB = new unsigned char[maxWidth*maxHeight];
  _bufI = new unsigned int[maxWidth*maxHeight];

  _maxWidth = maxWidth;
  _maxHeight = maxHeight;
  _scale = 1;
}

UvcVirtualCam::~UvcVirtualCam()
{
  delete [] _bufI;
  delete [] _bufR;
  delete [] _bufG;
  delete [] _bufB;
  delete [] _buf;
  delete _cam;
}

unsigned int UvcVirtualCam::getWidth()
{
  return _maxWidth/_scale;
}

unsigned int UvcVirtualCam::getHeight()
{
  return _maxHeight/_scale;
}

EnumCameraError UvcVirtualCam::setScale(unsigned int scale)
{
  if(scale<1)
  {
    cout << "UvcVirtualCam::grab(): Scale factor must be greater than 0." << endl;
    return CAMFAILURE;
  }

  unsigned int rows     = _cam->getHeight();
  unsigned int cols     = _cam->getWidth();

  unsigned int checkrows = rows/scale;
  unsigned int checkcols = cols/scale;

  if(checkrows*scale != rows || checkcols*scale != cols)
  {
    cout << "UvcVirtualCam::grab(): Width and height must be a multiple of the scale factor." << endl;
    return CAMFAILURE;
  }

  _scale = scale;
  return CAMSUCCESS;
}

void UvcVirtualCam::setColorMode(EnumCameraColorMode mode)
{
  _cam->setColorMode(mode);
}

unsigned int UvcVirtualCam::getChannels()
{
  if(_cam->getColorMode()==CAMRGB)
    return 3;
  else
    return 1;
}

EnumCameraError UvcVirtualCam::grab(unsigned char* image)
{

  unsigned int channels = _cam->getChannels();
  unsigned int cols     = _cam->getWidth();
  unsigned int rows     = _cam->getHeight();
  unsigned int ctr      = 0;

  EnumCameraError retval = _cam->grab(_buf);

  if(retval!=CAMSUCCESS) return retval;

  if(channels==3)
  {
    for(unsigned int i=0;i<cols*rows*channels;i++)
    {
      _bufR[ctr]=_buf[i];
      _bufG[ctr]=_buf[++i];
      _bufB[ctr]=_buf[++i];
      ctr++;
    }

    average(_bufR, cols, rows, _scale);
    average(_bufG, cols, rows, _scale);
    average(_bufB, cols, rows, _scale);

    ctr=0;
    for(unsigned int i=0;i<(rows/_scale)*(cols/_scale)*channels;i++)
    {
      image[i]  =_bufR[ctr];
      image[++i]=_bufG[ctr];
      image[++i]=_bufB[ctr];
      ctr++;
    }
  }
  else
  {
    average(_buf, cols, rows, _scale);
    memcpy(image, _buf, cols*rows/(_scale*_scale)*sizeof(*image));
  }

  return retval;
}

void UvcVirtualCam::average(unsigned char* img, unsigned int cols, unsigned int rows, unsigned int scale)
{
  // sum up column-wise
  unsigned int iold = 0;
  unsigned int inew = 0;
  for(unsigned int r=0; r<rows; r++)
  {
    for(unsigned int c=0; c<cols; c+=scale)
    {
      _bufI[inew] = 0;
      for(unsigned int i=0; i<scale; i++)
      {
        _bufI[inew] += img[iold];
        iold++;
      }
      inew++;
    }
  }

  // sum up row-wise
  unsigned int area = scale*scale;
  cols/=scale;
  inew = 0;
  for(unsigned int c=0; c<cols; c++)
  {
    iold = c;
    inew = c;
    for(unsigned int r=0; r<rows; r+=scale)
    {
      unsigned int sum = 0;
      for(unsigned int i=0; i<scale; i++)
      {
        sum += _bufI[iold];
        iold+=cols;
      }
      img[inew] = sum/area;
      inew+=cols;
    }
  }
}

}
