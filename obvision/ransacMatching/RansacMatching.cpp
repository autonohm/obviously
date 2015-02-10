/*
 * RansacMatching.cpp
 *
 *  Created on: 23.09.2014
 *      Author: mayst
 */

#include "RansacMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

using namespace std;

namespace obvious
{

RansacMatching::RansacMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet, bool clipPeripheralArea)
{
  _epsSqr             = epsThresh*epsThresh;
  _trials             = trials;
  _sizeControlSet     = sizeControlSet;
  _clipPeripheralArea = clipPeripheralArea;
  _model              = NULL;
  _index              = NULL;
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

obvious::Matrix RansacMatching::match(obvious::Matrix* M, obvious::Matrix* S, double phiMax)
{
  obvious::Matrix  TBest(3,3);
  TBest.setIdentity();

  if(S->getRows()<2)
  {
    LOGMSG(DBG_ERROR, "Size of scene too small, size: " << S->getRows());
    return TBest;
  }

  unsigned int pointsM = M->getRows();
  unsigned int pointsS = S->getRows();

  // -----------------------------------------------------
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  double** mData;
  obvious::System<double>::allocate(pointsM, cols, mData);
  for(unsigned int r=0; r<pointsM; r++)
  {
    mData[r][0] = (*M)(r,0);
    mData[r][1] = (*M)(r,1);
  }
  _model = new flann::Matrix<double>(&mData[0][0], pointsM, 2);
  flann::KDTreeSingleIndexParams p;
  _index = new flann::Index<flann::L2<double> >(*_model, p);
  _index->buildIndex();
  // -----------------------------------------------------


  // -----------------------------------------------------
  // randomly pick points from scene as control set
  vector<unsigned int> indices;
  unsigned int lowerBound = 0;
  unsigned int upperBound = S->getRows()-1;
  if(_clipPeripheralArea)
  {
    lowerBound = S->getRows()/8;
    upperBound = 7*S->getRows()/8-1;
  }

  for(unsigned int i=lowerBound; i<upperBound; i++)
    indices.push_back(i);

  unsigned int sizeControlSet = _sizeControlSet;
  if((indices.size())<sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size");
    sizeControlSet = indices.size();
  }
  vector<unsigned int> idxControl;
  while(idxControl.size()<sizeControlSet)
  {
    unsigned int r = rand()%indices.size();
    idxControl.push_back(indices[r]);
    indices.erase(indices.begin()+r);
  }
  obvious::Matrix SControl(3, idxControl.size());
  unsigned int ctr = 0;
  for(vector<unsigned int>::iterator it=idxControl.begin(); it!=idxControl.end(); ++it)
  {
      SControl(0, ctr)   = (*S)(*it, 0);
      SControl(1, ctr)   = (*S)(*it, 1);
      SControl(2, ctr++) = 1.0;
  }
  // -----------------------------------------------------

  LOGMSG(DBG_DEBUG, "Scene size: " << S->getRows() << ", Control set: " << idxControl.size());


  // -----------------------------------------------------
  // Lookup table for distances between scene points
  double** SDists;
  obvious::System<double>::allocate(pointsS, pointsS, SDists);
  for(unsigned int j=0; j<pointsS; j++)
  {
    for(unsigned int j2=j+1; j2<pointsS; j2++)
    {
      double dx = (*S)(j2, 0)-(*S)(j, 0);
      double dy = (*S)(j2, 1)-(*S)(j, 1);
      SDists[j][j2]  = dx*dx + dy*dy;
    }
  }
  // -----------------------------------------------------

  // -----------------------------------------------------
  // Perform RANSAC scheme as follows:
  // 1) pick random point from center part of model
  // 2) pick 2nd random point from model (right of first point)
  // 3) assign scene points to 1st point
  // 4) search 2nd point in scene with similar distance (1st and 2nd point in model)
  // 5) calculate transformation
  // 6) rate control set, i.e., determine consensus
  unsigned int cntBest = 0;
  double errBest       = 1e12;
  for(unsigned int trial = 0; trial<_trials; trial++)
  {
    // First model sample: Index i (only from center part)
    unsigned int i = rand() % (3*pointsM/8) + pointsM/4;

    // Second model sample: Random != i
    unsigned int i2 = i + rand()%(pointsM/8 - 1) + 1;

    // Vector between model points (for determining orientation)
    double vM[2];
    vM[0] = (*M)(i2, 0) - (*M)(i, 0);
    vM[1] = (*M)(i2, 1) - (*M)(i, 1);

    // Centroid of model (for determining translation)
    double cM[2];
    cM[0] = ((*M)(i, 0) + (*M)(i2, 0))/2.0;
    cM[1] = ((*M)(i, 1) + (*M)(i2, 1))/2.0;

    double distM = vM[0]*vM[0] + vM[1]*vM[1];

    double pointS[2];
    double pointS2[2];
    for(unsigned int j=0; j<pointsS; j++)
    {
      // Assign scene sample j
      pointS[0] = (*S)(j, 0);
      pointS[1] = (*S)(j, 1);

      // Find scene sample with similar distance
      unsigned int jMin = 0;
      double distSMin   = 1e12;
      for(unsigned int j2=j+1; j2<pointsS; j2++)
      {
        double distEps   = fabs(SDists[j][j2]-distM);
        if(distEps<distSMin)
        {
          distSMin = distEps;
          jMin     = j2;
        }
      }

      if(distSMin<_epsSqr)
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

      // Calculate polar angle
      double phiM = atan2(vM[1],vM[0]);
      if(phiM<0) phiM += 2.0*M_PI;
      double phiS = atan2(vS[1],vS[0]);
      if(phiS<0) phiS += 2.0*M_PI;

      // Solution for rotational part
      double phi = phiM - phiS;

      if(fabs(phi) < phiMax)
      {
        // Centroid of scene
        double cS[2];
        cS[0] = (pointS2[0] + pointS[0])/2.0;
        cS[1] = (pointS2[1] + pointS[1])/2.0;

        obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

        // Calculate translation
        T(0,2) = cM[0] - (T(0,0)*cS[0] + T(0,1)*cS[1]);
        T(1,2) = cM[1] - (T(1,0)*cS[0] + T(1,1)*cS[1]);

        obvious::Matrix STemp = T * SControl;

        // Determine how many nearest neighbors (model <-> scene) are close enough
        double q[2];
        unsigned int cntMatch = 0;
        flann::Matrix<int> indices(new int[1], 1, 1);
        flann::Matrix<double> dists(new double[1], 1, 1);
        double err = 0;
        for(unsigned int s=0; s<STemp.getCols(); s++)
        {
          q[0] = STemp(0, s);
          q[1] = STemp(1, s);
          flann::Matrix<double> query(q, 1, 2);

          flann::SearchParams p(-1, 0.0);
          _index->knnSearch(query, indices, dists, 1, p);
          if(dists[0][0]<_epsSqr)
          {
            err += dists[0][0];
            cntMatch++;
          }
        }
        delete[] indices.ptr();
        delete[] dists.ptr();

        if(cntMatch==0) continue;

        err /= cntMatch;

        if(cntMatch>cntBest)
        {
          errBest = err;
          cntBest = cntMatch;
          TBest   = T;
        }
        else if(cntMatch==cntBest)
        {
          if(err < errBest)
          {
            errBest = err;
            cntBest = cntMatch;
            TBest   = T;
          }
        }
      } // if(fabs(phi) < _phiMax)
    } // for all points in scene
  } // for trials

  LOGMSG(DBG_DEBUG, "Matching result - cnt(best): " << cntBest << ", err(best): " << errBest);

  obvious::System<double>::deallocate(SDists);
  obvious::System<double>::deallocate(mData);

  return TBest;
}

}
