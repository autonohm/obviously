#include <iostream>
#include "obdevice/UvcCam.h"
#include "obcore/base/tools.h"

using namespace std;
using namespace obvious;

/**
 * In order to find out the serial number of your uvc cam use:
 * udevadm info -a -p $(udevadm info -q path -p /class/video4linux/video0)
 */
int main(int argc, char* argv[])
{
  if(argc!=2)
  {
	cout << "usage " << argv[0] << " serial#" << endl;
	return -1;
  }

  char* path = NULL;
  UvcCam::FindDevice(argv[1], path);
  if(path==NULL)
  {
	  cout << "Device could not be found" << endl;
	  return -1;
  }
  cout << path << endl;
  delete path;
  return 0;
}
