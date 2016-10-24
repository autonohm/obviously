#include "obvision/ransac/RansacPrimitives.h"

#include <iostream>
#include <math.h>

using namespace obvious;
using namespace std;

int main(int argc, char* argv[])
{
  const unsigned int n=1000;
  Matrix D(n, 2);
  bool mask[n];

  // Generate data
  double x0 = 4.0;
  double y0 = 5.0;
  double r = 6.0;
  double s = 1.0;
  for(unsigned int i=0; i<n; i++)
  {
    // Get random number between -1 and 1
    double rnd = 2.0*(((double)(rand() % 1000)) / 1000.0 - 0.5);
    s = -s;
    double x = rnd*r + x0;
    double y = s*sqrt(r*r - (x-x0)*(x-x0)) + y0;

    // add 5% noise
    double noiseX = r * 0.05 * 2.0*(((double)(rand() % 1000)) / 1000.0 - 0.5);
    double noiseY = r * 0.05 * 2.0*(((double)(rand() % 1000)) / 1000.0 - 0.5);

    D(i, 0) = x + noiseX;
    D(i, 1) = y + noiseY;

    cout << D(i, 0) << " " << D(i, 1) << endl;
  }

  RansacPrimitives ransac(30);
  double params[3];
  ransac.findCircle(&D, params, mask);

  cout << "Found parameters: " << params[0] << " " << params[1] << " " << params[2] << endl;
}
