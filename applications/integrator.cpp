#include "obcore/math/IntegratorSimpson.h"
#include "obcore/base/Timer.h"
#include <stdio.h>
#include <iostream>
#include <cmath>

using namespace obvious;

int main(void)
{
  IntegratorSimpson _integrator;
  Timer _timer;

  while(1)
  {
    double input = cos(_timer.getTime()/1000);
    std::cout << _integrator.integrate(input) << std::endl;
  }
}
