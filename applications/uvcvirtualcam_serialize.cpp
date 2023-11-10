#include <iostream>
#include "obdevice/UvcVirtualCam.h"
#include "obcore/base/tools.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  unsigned int width       = 640;
  unsigned int height      = 480;
  unsigned int scale       = 1;
  EnumCameraColorMode mode = CAMRGB;

  char* dev           = (char*)"/dev/video0";

  if(argc>1) dev = argv[1];

  if(argc>2)
  {
    if(argc!=6)
    {
      cout << "usage: " << argv[0] << " [device] [width height scale colormode]" << endl;
      cout << " colormode: 1=Grayscale, else=RGB" << endl;
      return 0;
    }
    width           = atoi(argv[2]);
    height          = atoi(argv[3]);
    scale           = atoi(argv[4]);
    int tmp         = atoi(argv[5]);
    if(tmp==1) mode = CAMGRAYSCALE;
  }

  if(scale<1)
  {
    cout << "Scale factor must be greater than 0." << endl;
    return -1;
  }

  UvcVirtualCam* cam     = new UvcVirtualCam(dev, width, height, mode);
  EnumCameraError retval = cam->setScale(scale);

  if(retval!=CAMSUCCESS) return -1;

  unsigned char* img     = new unsigned char[cam->getWidth() * cam->getHeight() * 3];

  retval = cam->grab(img);

  if(retval==CAMSUCCESS)
  {
    if(mode==CAMRGB)
    {
      char* path = (char*)"/tmp/test.ppm";
      cout << "Serializing image to " << path << " (width: " << cam->getWidth() << ", " << cam->getHeight() << ")" << endl;
      serializePPM(path, img, cam->getWidth(), cam->getHeight(), 0);
    }
    else
    {
      char* path = (char*)"/tmp/test.pgm";
      cout << "Serializing image to " << path << " (width: " << cam->getWidth() << ", " << cam->getHeight() << ")" << endl;
      serializePGM(path, img, cam->getWidth(), cam->getHeight(), 0);
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
