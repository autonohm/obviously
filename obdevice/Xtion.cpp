/**
* @file Xtion.cpp
* @autor christian
* @date  23.12.2012
*
*
*/

#include "Xtion.h"
#include <obcore/math/mathbase.h>
#include "obcore/math/linalg/linalg.h"
#include "math.h"

namespace obvious
{

Xtion::Xtion(const char* path)
  : OpenNIDevice(path)
{

}

Xtion::~Xtion()
{

}

bool Xtion::grab()
{
  if(_useIR)
    return grabIR();
  else
    return grabDistance();
}


} // namespace




