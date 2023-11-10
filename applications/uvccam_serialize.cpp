#include <iostream>
#include "obdevice/UvcCam.h"
#include "obcore/base/tools.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  unsigned int width       = 640;
  unsigned int height      = 480;
  EnumCameraColorMode mode = CAMRGB;

  char* dev           = (char*)"/dev/video0";

  if(argc>1) dev = argv[1];

  if(argc>2)
  {
    if(argc!=5)
    {
      cout << "usage: " << argv[0] << " [device] [width height colormode]" << endl;
      cout << " colormode: 1=Grayscale, else=RGB" << endl;
      return 0;
    }
    width           = atoi(argv[2]);
    height          = atoi(argv[3]);
    int tmp         = atoi(argv[4]);
    if(tmp==1) mode = CAMGRAYSCALE;
  }

  UvcCam* cam            = new UvcCam(dev, width, height);
  unsigned char* img     = new unsigned char[width*height*3];

  EnumCameraError retval = cam->connect();
  if(retval != CAMSUCCESS) return -1;

  unsigned int format = V4L2_PIX_FMT_YUYV;
  retval = cam->setFormat(width, height, format);
  if(retval!=CAMSUCCESS) return -1;

  retval = cam->startStreaming();

  if(retval==CAMSUCCESS)
  {
    cam->setColorMode(mode);
    unsigned int bytes;
    cam->grab(img, &bytes);

    if(format == V4L2_PIX_FMT_MJPEG)
    {
       char* path = (char*)"/tmp/test.jpg";
       FILE* file = fopen (path, "wb");
       fwrite (img, bytes, 1, file);
       fclose (file);
    }
    else
    {
       if(mode==CAMRGB)
       {
         char* path = (char*)"/tmp/test.ppm";
         cout << "Serializing image to " << path << " (width: " << width << ", " << height << ")" << endl;
         serializePPM(path, img, width, height, 0);
       }
       else
       {
         char* path = (char*)"/tmp/test.pgm";
         cout << "Serializing image to " << path << " (width: " << width << ", " << height << ")" << endl;
         serializePGM(path, img, width, height, 0);
       }
    }
  }
  else
  {
    cout << "Failed to connect to camera " << dev << endl;
  }

  delete [] img;
  delete cam;
  return 0;
}
