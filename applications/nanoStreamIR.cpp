/**
* @file nanoStreamIR.cpp
* @autor christian
* @date  30.11.2012
*
*
*/


#include <iostream>
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obgraphic/Obvious2D.h"
#include "obdevice/CamNano.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  CamNano* nano = new CamNano;
  unsigned int width   = nano->getCols();
  unsigned int height  = nano->getRows();

  Obvious2D viewer(width, height, "UVC streaming example");

  while(1)
  {
    if (nano->grab())
      viewer.draw(nano->getImage(), width, height, 1);
  }
  delete [] nano;
  return 0;
}





