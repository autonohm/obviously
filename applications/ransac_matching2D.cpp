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

#include "obvision/registration/ransacMatching/RansacMatching.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"

using namespace std;
using namespace obvious;

#define DATASETSIZE 1081

int main(int argc, char** argv)
{
  Timer timer;
  timer.start();

  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(DATASETSIZE, 2);
  bool maskM[DATASETSIZE];
  bool maskS[DATASETSIZE];

  double inc = deg2rad(270.0/(DATASETSIZE-1));
  double angle = deg2rad(-135.0);
  for(int i = 0; i < DATASETSIZE; i++)
  {
    double d = sin(2.0*angle)+2.0;

    (*M)(i, 0) = cos(angle)*d;
    (*M)(i, 1) = sin(angle)*d;
    angle += inc;

    maskM[i] = ((i % 2) != 0);
    maskS[i] = ((i % 2) != 0);
    //maskM[i] = 1;
    //maskS[i] = 1;
  }
  //Model Normals
//  obvious::Matrix* N = new obvious::Matrix(DATASETSIZE, 2);
//  // compute mean of components build by left and right neighbors
//  for(int i=1; i<DATASETSIZE-1; i++)
//  {
//    double xleft  = (*M)(i, 0)   - (*M)(i-1, 0);
//    double xright = (*M)(i+1, 0) - (*M)(i, 0);
//    double yleft  = (*M)(i, 1)   - (*M)(i-1, 1);
//    double yright = (*M)(i+1, 1) - (*M)(i, 1);
//
//
//    // x-component of normal
//    double xNormal = -(yright + yleft) * 0.5;
//    // y-component of normal
//    double yNormal = (xright + xleft) * 0.5;
//    //Normalize
//    double lengthNormal = sqrt(xNormal*xNormal + yNormal*yNormal);
//    (*N)(i-1, 0) = xNormal / lengthNormal;
//    (*N)(i-1, 1) = yNormal / lengthNormal;
//  }
//
//  // left bound
//  (*N)(0, 0) = -((*M)(1, 1) - (*M)(0, 1));
//  (*N)(0, 1) = (*M)(1, 0) - (*M)(0, 0);
//
//  // right bound
//  (*N)(DATASETSIZE-1, 0) = -((*M)(DATASETSIZE-1, 1) - (*M)(DATASETSIZE-2, 1));
//  (*N)(DATASETSIZE-1, 1) = (*M)(DATASETSIZE-1, 0) - (*M)(DATASETSIZE-2, 0);

  obvious::Matrix T = MatrixFactory::TransformationMatrix33(deg2rad(35.0), 0.4, 0.35);

  obvious::Matrix S = M->createTransform(T);

  //for(unsigned int i=0; i<S.getRows(); i++)
  //  cout << S(i, 0) << endl;

  cout << "Applied transformation:" << endl;
  T.print();

  cout << "Search for inverse:" << endl;
  T.invert();
  T.print();

  unsigned int trials = 20;
  double epsThresh = 0.02;
  unsigned int sizeControlSet = 100;

  //RansacMatching matcher(trials, epsThresh, sizeControlSet);
  RandomNormalMatching matcher(trials, epsThresh, sizeControlSet);
  matcher.activateTrace();

  //Matrix F = matcher.match(M, maskM, &S, maskS, deg2rad(45.0), 1.5 , deg2rad(0.25));
  Matrix F = matcher.match(M, maskM, NULL, &S, maskS, deg2rad(45.0), 1.5 , deg2rad(0.25));

  matcher.serializeTrace("/tmp/trace");

  F.invert();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  cout << "elapsed: " << timer.elapsed() << " s" << endl;
  return 0;
}
