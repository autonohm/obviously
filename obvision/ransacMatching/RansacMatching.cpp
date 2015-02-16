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

RansacMatching::RansacMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet, bool clipControlSet)
{
  _epsSqr = epsThresh * epsThresh;
  _trials = trials;
  _sizeControlSet = sizeControlSet;
  _clipControlSet = clipControlSet;
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

obvious::Matrix* RansacMatching::extractValidSubmatrix(const obvious::Matrix* M, const bool* mask)
{
  unsigned int size       = M->getRows();
  unsigned int sizeValid  = 0;

  for(unsigned int i=0; i<size; i++)
    if(mask[i]) sizeValid++;

  obvious::Matrix* MValid = new Matrix(sizeValid, M->getCols());

  unsigned int iValid = 0;
  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i])
    {
      for(unsigned int j=0; j<M->getCols(); j++)
      {
        (*MValid)(iValid, j) = (*M)(i, j);
      }
      iValid++;
    }
  }
  return MValid;
}

obvious::Matrix RansacMatching::match(const obvious::Matrix* M,  const bool* maskM, const obvious::Matrix* S,  const bool* maskS, double phiMax, double resolution)
{

  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  obvious::Matrix* MValid = extractValidSubmatrix(M, maskM);
  obvious::Matrix* SValid = extractValidSubmatrix(S, maskS);

  unsigned int pointsM = MValid->getRows();
  unsigned int pointsS = SValid->getRows();

  if(pointsS < 2 && pointsM < 2)
  {
    LOGMSG(DBG_ERROR, "Size of model or scene too small, size M: " << pointsM << ", size S: " << pointsS);
    return TBest;
  }

  // ------------------------------------------------------
  // Calculate search "radius", i.e., maximum difference of indices because of rotation
  int span = pointsS;
  if(resolution > 1e-6)
  {
    span = (int)(phiMax / resolution);
  }

  // Clip area for picking points in model since peripheral points are likely to be outliers
  unsigned int modelLowerBound = 0;
  unsigned int modelUpperBound = MValid->getRows()-1;
  for(int i=0; i<span; i++)
    if(maskM[i]) modelLowerBound++;
  for(int i=(int)(M->getRows()-1); i>(int)M->getRows()-span; i--)
    if(maskM[i]) modelUpperBound--;
  unsigned int modelSpan = modelUpperBound - modelLowerBound;
  LOGMSG(DBG_DEBUG, "Model search aream " << modelLowerBound << " ... " << modelUpperBound);

  // Calculate look up tables for phiMax selection
  vector<int> reverseMapM;
  vector<int> lutSearchRangeS;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    if(maskM[i])
    {
      reverseMapM.push_back(i);
    }
  }
  unsigned int cnt = 0;
  for(unsigned int i=0; i<S->getRows(); i++)
  {
    if(maskS[i]) cnt++;
    if(cnt==0)
      lutSearchRangeS.push_back(0);
    else
      lutSearchRangeS.push_back(cnt-1);
  }
  // with this table, one can select an area as follows
  // [lutSearchRangeS(reverseMapM(i)-span); lutSearchRangeS(reverseMapM(i)+span)]
  // ------------------------------------------------------

  // -----------------------------------------------------
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = MValid->getCols();
  double** mData;
  obvious::System<double>::allocate(pointsM, cols, mData);
  for(unsigned int r = 0; r < pointsM; r++)
  {
    mData[r][0] = (*MValid)(r, 0);
    mData[r][1] = (*MValid)(r, 1);
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
  unsigned int upperBound = SValid->getRows() - 1;
  if(_clipControlSet)
  {
    for(int i=0; i<span; i++)
      if(maskS[i]) lowerBound++;
    for(int i=(int)(S->getRows()-1); i>(int)S->getRows()-span; i--)
      if(maskS[i]) upperBound--;

    LOGMSG(DBG_DEBUG, "Clipped area of control set to " << lowerBound << " ... " << upperBound);
  }

  for(unsigned int i = lowerBound; i < upperBound; i++)
    indices.push_back(i);

  unsigned int sizeControlSet = _sizeControlSet;
  if((indices.size()) < sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size");
    sizeControlSet = indices.size();
  }
  vector<unsigned int> idxControl;
  while(idxControl.size() < sizeControlSet)
  {
    unsigned int r = rand() % indices.size();
    idxControl.push_back(indices[r]);
    indices.erase(indices.begin() + r);
  }
  obvious::Matrix SControl(3, idxControl.size());
  unsigned int ctr = 0;
  for(vector<unsigned int>::iterator it = idxControl.begin(); it != idxControl.end(); ++it)
  {
    SControl(0, ctr)   = (*SValid)(*it, 0);
    SControl(1, ctr)   = (*SValid)(*it, 1);
    SControl(2, ctr++) = 1.0;
  }
  // -----------------------------------------------------

  LOGMSG(DBG_DEBUG, "Scene size: " << SValid->getRows() << ", Control set: " << idxControl.size());

  // -----------------------------------------------------
  // Lookup table for distances between scene points
  double** SDists;
  obvious::System<double>::allocate(pointsS, pointsS, SDists);
  for(unsigned int j = 0; j < pointsS; j++)
  {
    for(unsigned int j2 = j + 1; j2 < pointsS; j2++)
    {
      double dx = (*SValid)(j2, 0) - (*SValid)(j, 0);
      double dy = (*SValid)(j2, 1) - (*SValid)(j, 1);
      SDists[j][j2] = dx * dx + dy * dy;
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
  for(unsigned int trial = 0; trial < _trials; trial++)
  {
    // First model sample: Index i (only from center part)
    unsigned int i = modelLowerBound + rand() % (modelSpan/2);

    // Second model sample: Random in 2nd half of center != i
    unsigned int i2 = i + rand() % (modelSpan/2 - 1) + 1;

    // Vector between model points (for determining orientation)
    double vM[2];
    vM[0] = (*MValid)(i2, 0) - (*MValid)(i, 0);
    vM[1] = (*MValid)(i2, 1) - (*MValid)(i, 1);

    // Centroid of model (for determining translation)
    double cM[2];
    cM[0] = ((*MValid)(i, 0) + (*MValid)(i2, 0)) / 2.0;
    cM[1] = ((*MValid)(i, 1) + (*MValid)(i2, 1)) / 2.0;

    double distM = vM[0] * vM[0] + vM[1] * vM[1];

    double sceneSimilar[2];

    //Calculate interesting area of scene points according to phiMax, field of view (FoV) and number of sample points
    unsigned int jMin = lutSearchRangeS[max(reverseMapM[i]-span, 0)];
    unsigned int jMax = lutSearchRangeS[min(reverseMapM[i]+span, (int)(lutSearchRangeS.size()-1))];
    //LOGMSG(DBG_DEBUG, "Search range: " << jMin << " " << jMax);
    for(unsigned int j = jMin; j < jMax; j++)
    {
      // Find scene sample with similar distance
      unsigned int jMinDist = 0;
      double distSMin       = 1e12;
      for(unsigned int j2 = j + 1; j2 < pointsS; j2++)
      {
        double distEps = fabs(SDists[j][j2] - distM);
        if(distEps < distSMin)
        {
          distSMin = distEps;
          jMinDist = j2;
        }
      }

      if(distSMin < _epsSqr)
      {
        sceneSimilar[0] = (*SValid)(jMinDist, 0);
        sceneSimilar[1] = (*SValid)(jMinDist, 1);
      }
      else
        continue;

      // Align scans
      double vS[2];
      vS[0] = sceneSimilar[0] - (*SValid)(j, 0);
      vS[1] = sceneSimilar[1] - (*SValid)(j, 1);

      // Calculate polar angle
      double phiM = atan2(vM[1], vM[0]);
      if(phiM < 0)
        phiM += 2.0 * M_PI;
      double phiS = atan2(vS[1], vS[0]);
      if(phiS < 0)
        phiS += 2.0 * M_PI;

      // Solution for rotational part
      double phi = phiM - phiS;

      if(fabs(phi) < phiMax)
      {
        // Centroid of scene
        double cS[2];
        cS[0] = (sceneSimilar[0] + (*SValid)(j, 0)) / 2.0;
        cS[1] = (sceneSimilar[1] + (*SValid)(j, 1)) / 2.0;

        obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

        // Calculate translation
        T(0, 2) = cM[0] - (T(0, 0) * cS[0] + T(0, 1) * cS[1]);
        T(1, 2) = cM[1] - (T(1, 0) * cS[0] + T(1, 1) * cS[1]);

        obvious::Matrix STemp = T * SControl;

        // Determine how many nearest neighbors (model <-> scene) are close enough
        double q[2];
        unsigned int cntMatch = 0;
        flann::Matrix<int> indices(new int[1], 1, 1);
        flann::Matrix<double> dists(new double[1], 1, 1);
        double err = 0;
        for(unsigned int s = 0; s < STemp.getCols(); s++)
        {
          q[0] = STemp(0, s);
          q[1] = STemp(1, s);
          flann::Matrix<double> query(q, 1, 2);

          flann::SearchParams p(-1, 0.0);
          _index->knnSearch(query, indices, dists, 1, p);
          if(dists[0][0] < _epsSqr)
          {
            err += dists[0][0];
            cntMatch++;
          }
        }
        delete[] indices.ptr();
        delete[] dists.ptr();

        if(cntMatch == 0)
          continue;

        err /= cntMatch;

        if(cntMatch > cntBest)
        {
          errBest = err;
          cntBest = cntMatch;
          TBest = T;
        }
        else if(cntMatch == cntBest)
        {
          if(err < errBest)
          {
            errBest = err;
            cntBest = cntMatch;
            TBest = T;
          }
        }
      }  // if(fabs(phi) < _phiMax)
    }  // for all points in scene
  }  // for trials

  LOGMSG(DBG_DEBUG, "Matching result - cnt(best): " << cntBest << ", err(best): " << errBest);

  obvious::System<double>::deallocate(SDists);
  obvious::System<double>::deallocate(mData);

  delete MValid;
  delete SValid;

  return TBest;
}

}
