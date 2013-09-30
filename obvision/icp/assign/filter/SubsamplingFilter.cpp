#include "SubsamplingFilter.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>

namespace obvious
{

SubsamplingFilter::SubsamplingFilter(unsigned int step)
{
  _step = step;
};

SubsamplingFilter::~SubsamplingFilter()
{

}

void SubsamplingFilter::filter(double** scene, unsigned int size, bool* mask)
{
  if(!_active) return;

  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i] == 0) continue;

    if(i%_step==0)
    {
        mask[i] = 1;
    }
    else
    {
      mask[i] = 0;
    }
  }
}

}
