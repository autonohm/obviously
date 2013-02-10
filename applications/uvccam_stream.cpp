#include <iostream>
#include <signal.h>
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obdevice/UvcCam.h"
#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

UvcCam* _cam;
unsigned char* _img;

unsigned int _widthPrev = 1920;
unsigned int _heightPrev = 1080;

void toggleMJPEG()
{
   unsigned int width = _widthPrev;
   unsigned int height = _heightPrev;
   _widthPrev = _cam->getWidth();
   _heightPrev = _cam->getHeight();
   _cam->disconnect();
   delete [] _img;
   _img = new unsigned char[width*height*3];
   _cam->connect();
   _cam->setFormat(width, height, V4L2_PIX_FMT_MJPEG);
   _cam->startStreaming();
}

void saveHighResImage()
{
   unsigned int width = 2304;
   unsigned int height = 1536;
   _widthPrev = _cam->getWidth();
   _heightPrev = _cam->getHeight();
   _cam->disconnect();
   delete [] _img;
   _img = new unsigned char[width*height*3];
   _cam->connect();
   _cam->setFormat(width, height, V4L2_PIX_FMT_YUYV);
   _cam->startStreaming();
   unsigned int bytes;
   _cam->grab(_img, &bytes);
   char* path = (char*)"/tmp/snapshot.ppm";
   serializePPM(path, _img, _cam->getWidth(), _cam->getHeight(), 0);
   toggleMJPEG();
   cout << "snapshot saved to " << path << endl;
}

int main(int argc, char* argv[])
{
   // Default parameters
   char* dev           = (char*)"/dev/video0";
   unsigned int width       = 640;
   unsigned int height      = 360;

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

   _cam = new UvcCam(dev, width, height);

   EnumCameraError retval = _cam->connect();
   if(retval!=CAMSUCCESS) return -1;

   retval = _cam->setFormat(width, height, V4L2_PIX_FMT_MJPEG);
   if(retval!=CAMSUCCESS) return -1;

   Obvious2D viewer(1280, 720, "UVC streaming example");
   viewer.registerKeyboardCallback(' ', toggleMJPEG);
   viewer.registerKeyboardCallback('s', saveHighResImage);
   _img = new unsigned char[width*height*3];

   retval = _cam->setFramerate(1,15);
   if(retval!=CAMSUCCESS) return -1;

   retval = _cam->startStreaming();
   if(retval!=CAMSUCCESS) return -1;

   Timer t;
   while(retval==CAMSUCCESS && viewer.isAlive())
   {
      unsigned int bytes;
      retval = _cam->grab(_img, &bytes);
      if(retval==CAMSUCCESS)
      {
         viewer.draw(_img, _cam->getWidth(), _cam->getHeight(), 3, (_cam->getFormat()==V4L2_PIX_FMT_MJPEG), bytes);
         cout << "Elapsed: " << t.reset() << " ms" << endl;
      }

   }

   delete [] _img;
   delete _cam;
   return 0;
}
