/*
 * RansacMatching.cpp
 *
 *  Created on: 23.09.2014
 *      Author: mayst
 */

#include "RansacMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

using namespace std;

namespace obvious
{

RansacMatching::RansacMatching()
{

  _epsDist = 0.05;
  _phiMax  = M_PI / 4.0;
  _trials  = 20;

  _model = NULL;
  _index = NULL;
}

RansacMatching::~RansacMatching()
{
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
}

obvious::Matrix RansacMatching::match(obvious::Matrix* M, obvious::Matrix* S)
{
  double** mData;
  obvious::System<double>::allocate(M->getRows(), M->getCols(), mData);
  for(unsigned int r=0; r<M->getRows(); r++)
  {
    mData[r][0] = (*M)(r,0);
    mData[r][1] = (*M)(r,1);
  }
  _model = new flann::Matrix<double>(&mData[0][0], M->getRows(), 2);
  flann::KDTreeSingleIndexParams p;
  _index = new flann::Index<flann::L2<double> >(*_model, p);
  _index->buildIndex();

  obvious::Matrix  TBest(3,3);
  TBest.setIdentity();
  unsigned int cntBest = 0;

  vector<unsigned int> idxControl;
  for(unsigned int r=0; r<S->getRows(); r++)
  {
    if(rand()%1000<50)
      idxControl.push_back(r);
  }

  obvious::Matrix SControl(3, idxControl.size());
  unsigned int ctr = 0;
  for(vector<unsigned int>::iterator it=idxControl.begin(); it!=idxControl.end(); ++it)
  {
      SControl(0, ctr) = (*S)(*it, 0);
      SControl(1, ctr) = (*S)(*it, 1);
      SControl(2, ctr++) = 1.0;
  }

  unsigned int pointsM = M->getRows();
  unsigned int pointsS = S->getRows();

  for(unsigned int trial = 0; trial<_trials; trial++)
  {
    // First model sample: Index i
    unsigned int i = rand() % pointsM;
    // Second model sample: Random != i
    unsigned int i2 = rand() % pointsM;
    if(i==i2) continue;

    if(i2<i) swap(i, i2);

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

      // Find scene sample with similar distance
      unsigned int jMin = 0;
      double distSMin = 1e12;
      for(unsigned int j2=j+1; j2<pointsS; j2++)
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

      if(distSMin<_epsDist)
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

      if(phi < _phiMax)
      {
        obvious::Matrix  T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

        // Centroid of scene
        double cS[2];
        cS[0] = (pointS2[0] + pointS[0])/2.0;
        cS[1] = (pointS2[1] + pointS[1])/2.0;

        // Calculate translation
        T(0,2) = cM[0] - (T(0,0)*cS[0] + T(0,1)*cS[1]);
        T(1,2) = cM[1] - (T(1,0)*cS[0] + T(1,1)*cS[1]);

        obvious::Matrix STemp = T * SControl;

        double q[2];
        unsigned int cntMatch = 0;
        flann::Matrix<int> indices(new int[1], 1, 1);
        flann::Matrix<double> dists(new double[1], 1, 1);
        for(unsigned int s=0; s<STemp.getCols(); s++)
        {
          q[0] = STemp(0, s);
          q[1] = STemp(1, s);
          flann::Matrix<double> query(q, 1, 2);

          int count = _index->radiusSearch(query, indices, dists, _epsDist, -1);
          cntMatch+=count;
        }
        delete[] indices.ptr();
        delete[] dists.ptr();

        if(cntMatch>cntBest)
        {
          cntBest = cntMatch;
          TBest = T;
        }
      }
    }
  }

  obvious::System<double>::deallocate(mData);

  return TBest;
}

}
