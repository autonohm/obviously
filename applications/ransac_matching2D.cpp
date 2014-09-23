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

int main(int argc, char** argv)
{
  Timer timer;
  timer.start();

  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(1000, 2);

  for(int i=0; i<1000; i++)
  {
    double di = (double) i;
    (*M)(i,0) = sin(di/500.0);
    (*M)(i,1) = sin(di/100.0);
  }


  obvious::Matrix T = MatrixFactory::TransformationMatrix33(deg2rad(29.0), 0.4, 0.35);
  obvious::Matrix S = M->createTransform(T);

  RansacMatching matcher;

  Matrix F = matcher.match(M, &S);

  cout << "Applied transformation:" << endl;
  T.print();
  F.invert();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  cout << "elapsed: " << timer.elapsed() << " s" << endl;
  return 0;
}
