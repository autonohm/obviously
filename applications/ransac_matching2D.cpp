/**
 * Sample application showing the usage of the OBVIOUS 2D range image implementation.
 * @author Stefan May
 * @date 23.09.2014
 */

#include <string.h>
#include <iostream>
#include <math.h>

#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Timer.h"

#include "obvision/ransacMatching/RansacMatching.h"

using namespace std;
using namespace obvious;

#define DATASETSIZE 1081

int main(int argc, char** argv)
{
  Timer timer;
  timer.start();

  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(DATASETSIZE, 2);
  bool mask[DATASETSIZE];

  for(int i = 0; i < DATASETSIZE; i++)
  {
    double di  = (double)i;
    (*M)(i, 0) = sin(di / 500.0);
    (*M)(i, 1) = sin(di / 100.0);
    mask[i]    = true;
  }

  obvious::Matrix T = MatrixFactory::TransformationMatrix33(deg2rad(35.0), 0.4, 0.35);

  obvious::Matrix S = M->createTransform(T);

  cout << "Applied transformation:" << endl;
  T.print();

  cout << "Search for inverse:" << endl;
  T.invert();
  T.print();

  unsigned int trials         = 50;
  double epsThresh            = 0.15;
  unsigned int sizeControlSet = 180;
  bool clipPeripheralArea     = true;
  RansacMatching matcher(trials, epsThresh, sizeControlSet, clipPeripheralArea);

  Matrix F = matcher.match(M, mask, &S, mask, deg2rad(45.0), deg2rad(0.25));

  F.invert();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  cout << "elapsed: " << timer.elapsed() << " s" << endl;
  return 0;
}
