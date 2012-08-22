#include <iostream>
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obdevice/UvcCam.h"
#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  unsigned int width       = 640;
  unsigned int height      = 480;

  char* dev           = (char*)"/dev/video0";
  if(argc>1) dev = argv[1];
  if(argc>2)
  {
    if(argc!=4)
    {
      cout << "usage: " << argv[0] << " [device] [width height]" << endl;
      return 0;
    }
    width           = atoi(argv[2]);
    height          = atoi(argv[3]);
  }

  Obvious2D viewer(width, height, "UVC streaming example");

  UvcCam* cam            = new UvcCam(dev, width, height);
  unsigned char* img     = new unsigned char[width*height*3];

  EnumCameraError retval = cam->connect();
  if(retval!=CAMSUCCESS) return -1;

  retval = cam->setResolution(width,height);
  if(retval!=CAMSUCCESS) return -1;

  retval = cam->setFramerate(1,10);
  if(retval!=CAMSUCCESS) return -1;

  retval = cam->startStreaming();
  if(retval!=CAMSUCCESS) return -1;

  if(retval==CAMSUCCESS)
  {
    Timer t;
    while(1)
    {
      cam->grab(img);
      cout << "Elapsed: " << t.reset() << " ms" << endl;
      viewer.draw(img, width, height, 3);
    }
  }

  delete [] img;
  delete cam;
  return 0;
}
