/**
 * Sample application showing the usage of the OBVIOUS 2D range image implementation.
 * @author Stefan May
 * @date 23.09.2014
 */

#include <string.h>
#include <iostream>
#include <math.h>

#include "obcore/math/linalg/linalg.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"

using namespace std;
using namespace obvious;

obvious::Matrix match(Matrix* M, Matrix* S)
{
  obvious::Matrix  TBest(3,3);
  TBest.setIdentity();
  unsigned int cntBest = 0;

  Matrix SHom(3, S->getRows());
  for(unsigned int r=0; r<S->getRows(); r++)
  {
    SHom(0, r) = (*S)(r, 0);
    SHom(1, r) = (*S)(r, 1);
    SHom(2, r) = 1.0;
  }

  // distance threshold
  double epsDist = 0.05;

  // rotational threshold
  double phiMax = M_PI / 4.0;

  unsigned int pointsM = M->getRows();
  unsigned int pointsS = S->getRows();

  for(unsigned int trials = 0; trials<20; trials++)
  {
    // First model sample: Index i
    unsigned int i = rand() % pointsM;
    // Second model sample: Random != i
    unsigned int i2 = rand() % pointsM;
    if(i==i2) continue;
    double pointM[2];
    double pointM2[2];
    pointM[0] = (*M)(i, 0);
    pointM[1] = (*M)(i, 1);
    pointM2[0] = (*M)(i2, 0);
    pointM2[1] = (*M)(i2, 1);

    double vM[2];
    vM[0] = pointM2[0] - pointM[0];
    vM[1] = pointM2[1] - pointM[1];

    // Centroid of model
    double cM[2];
    cM[0] = (pointM2[0] + pointM[0])/2.0;
    cM[1] = (pointM2[1] + pointM[1])/2.0;

    double distM = euklideanDistance<double>(pointM, pointM2, 2);

    double pointS[2];
    double pointS2[2];
    for(unsigned int j=0; j<pointsS; j++)
    {
      // Assign scene sample j
      pointS[0] = (*S)(j, 0);
      pointS[1] = (*S)(j, 1);

      // Search left
      unsigned int jStart = 0;
      unsigned int jStop = j;

      // Search right
      if(i<i2)
      {
        jStart = j+1;
        jStop = pointsS;
      }

      // Find scene sample with similar distance
      unsigned int jMin = 0;
      double distSMin = 1e12;
      for(unsigned int j2=jStart; j2<jStop; j2++)
      {
        pointS2[0] = (*S)(j2, 0);
        pointS2[1] = (*S)(j2, 1);
        double distS = euklideanDistance(pointS, pointS2, 2);
        if(fabs(distS - distM)<distSMin)
        {
          distSMin = distS;
          jMin = j2;
        }
      }

      if(distSMin<epsDist)
      {
        pointS2[0] = (*S)(jMin, 0);
        pointS2[1] = (*S)(jMin, 1);
      }
      else
        continue;

      // Align scans
      double vS[2];
      vS[0] = pointS2[0] - pointS[0];
      vS[1] = pointS2[1] - pointS[1];

      double phi = acos( dot2(vS,vM) / (abs2D(vM)*abs2D(vS)));

      if(tan(vS[1]/vS[0])>tan(vM[1]/vM[0]))
        phi = -phi;

      if(phi < phiMax)
      {
        obvious::Matrix  T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

        // Centroid of scene
        double cS[2];
        cS[0] = (pointS2[0] + pointS[0])/2.0;
        cS[1] = (pointS2[1] + pointS[1])/2.0;

        // Calculate translation
        T(0,2) = cM[0] - (T(0,0)*cS[0] + T(0,1)*cS[1]);
        T(1,2) = cM[1] - (T(1,0)*cS[0] + T(1,1)*cS[1]);

        Matrix STemp = T * SHom;

        unsigned int cntMatch = 0;
        unsigned int controlSamples = 25;
        for(unsigned int s=0; s<controlSamples; s++)
        {
          unsigned int k = rand() % pointsM;
          for(unsigned int l=0; l<pointsS; l++)
          {
            double vMatch[2];
            vMatch[0] = (*M)(k, 0) - STemp(0, l);
            vMatch[1] = (*M)(k, 1) - STemp(1, l);
            double distMatch = abs2D(vMatch);
            double epsMatch = 0.025;
            if(distMatch<epsMatch)
            {
              cntMatch++;
              break;
            }
          }
        }
        if(cntMatch>cntBest)
        {
          cntBest = cntMatch;
          TBest = T;
        }
      }
    }
  }

  return TBest;
}

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

  Matrix F = match(M, &S);

  cout << "Applied transformation:" << endl;
  T.print();
  F.invert();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  cout << "elapsed: " << timer.elapsed() << " s" << endl;
  return 0;
}
