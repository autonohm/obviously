/**
 * Sample application showing the usage of the OBVIOUS NDT implementation.
 * @author Stefan May
 * @date 23.08.2014
 */

#include <string.h>
#include <iostream>

#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Timer.h"
#include "obvision/registration/ndt/Ndt.h"

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(100, 2);

  for(int i=0; i<100; i++)
  {
    double di = (double) i;
    (*M)(i,0) = sin(di/25.0);
    (*M)(i,1) = sin(di/10.0);
  }

  obvious::Matrix T = MatrixFactory::TransformationMatrix33(deg2rad(9.0), 0.4, 0.35);
  obvious::Matrix S = M->createTransform(T);

  Ndt* ndt = new Ndt(-1, 1, -1, 1);
  ndt->setModel(M);
  ndt->setScene(&S);

  double rms;
  unsigned int it;
  ndt->iterate(&rms, &it);
  obvious::Matrix F = ndt->getFinalTransformation();
  F.invert();
  cout << endl << "Error: " << rms << endl;
  cout << "Iterations: " << it << endl;

  cout << "Applied transformation:" << endl;
  T.print();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  delete ndt;

  return 0;
}
