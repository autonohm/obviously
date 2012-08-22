#include <iostream>
#include "obdevice/UvcCam.h"
#include "obcore/base/tools.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  char* dev = (char*)"/dev/video0";
  if(argc>1) dev = argv[1];

  UvcCam* cam = new UvcCam(dev, 640, 480);

  cam->printAvailableFormats();

  delete cam;
  return 0;
}
