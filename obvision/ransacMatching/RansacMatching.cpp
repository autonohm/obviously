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

void RansacMatching::initKDTree(obvious::Matrix* M)
{
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  unsigned int rows = M->getRows();
  double** mData;
  obvious::System<double>::allocate(rows, cols, mData);
  for(unsigned int r = 0; r < rows; r++)
  {
    mData[r][0] = (*M)(r, 0);
    mData[r][1] = (*M)(r, 1);
  }
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
  _model = new flann::Matrix<double>(&mData[0][0], rows, 2);
  flann::KDTreeSingleIndexParams p;
  _index = new flann::Index<flann::L2<double> >(*_model, p);
  _index->buildIndex();
  obvious::System<double>::deallocate(mData);
}

obvious::Matrix* RansacMatching::pickControlSet(const obvious::Matrix* M, const bool* mask, unsigned int span)
{
  vector<unsigned int> indices;
  unsigned int lowerBound = 0;
  unsigned int upperBound = M->getRows() - 1;
  if(_clipControlSet)
  {
    lowerBound  = span;
    upperBound -= span;

    LOGMSG(DBG_DEBUG, "Clipped area of control set to " << lowerBound << " ... " << upperBound);
  }

  for(unsigned int i = lowerBound; i < upperBound; i++)
  {
    if(mask[i]) indices.push_back(i);
  }
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
  obvious::Matrix* C = new obvious::Matrix(3, idxControl.size());
  unsigned int ctr = 0;
  for(vector<unsigned int>::iterator it = idxControl.begin(); it != idxControl.end(); ++it)
  {
    (*C)(0, ctr)   = (*M)(*it, 0);
    (*C)(1, ctr)   = (*M)(*it, 1);
    (*C)(2, ctr++) = 1.0;
  }
  return C;
}

double** RansacMatching::createLutIntraDistance(obvious::Matrix* M, int span)
{
  int points = (int)M->getRows();
  double** dists;
  obvious::System<double>::allocate(points, points, dists);
  for(int i = 0; i < points; i++)
  {
    int jmin = max(i-span, 0);
    int jmax = min(i+span, points);
    for(int j = jmin; j < jmax; j++)
    {
      double dx = (*M)(j, 0) - (*M)(i, 0);
      double dy = (*M)(j, 1) - (*M)(i, 1);
      dists[i][j] = dx * dx + dy * dy;
    }
  }
  return dists;
}

obvious::Matrix RansacMatching::match(const obvious::Matrix* M,  const bool* maskM, const obvious::Matrix* S,  const bool* maskS, double phiMax, double resolution)
{

  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  obvious::Matrix* MValid = extractValidSubmatrix(M, maskM);
  obvious::Matrix* SValid = extractValidSubmatrix(S, maskS);

  if(SValid->getRows() < 2 && MValid->getRows() < 2)
  {
    LOGMSG(DBG_ERROR, "Size of model or scene too small, size M: " << MValid->getRows() << ", size S: " << SValid->getRows());
    return TBest;
  }

  // Calculate search "radius", i.e., maximum difference of indices because of rotation
  int span;
  if(resolution > 1e-6)
  {
    span = (int)(phiMax / resolution);
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: " << resolution);
    return TBest;
  }

  vector<int> modelSelection;
  int valid = 0;
  for(int i=0; i<(int)M->getRows(); i++)
  {
    if(maskM[i])
    {
      modelSelection.push_back(valid++);
    }
  }

  // ------------------------------------------------------
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

  initKDTree(MValid);

  obvious::Matrix* Control = pickControlSet(S, maskS, span);

  LOGMSG(DBG_DEBUG, "Scene size: " << SValid->getRows() << ", Control set: " << Control->getRows());

  double** SDists = createLutIntraDistance(SValid, span);

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
    // pick randomly one point in model set
    unsigned int randIdx      = rand() % (modelSelection.size()-10);
    // ... and leave at least n points
    unsigned int remainingIdx = modelSelection.size()-randIdx-1;
    // Index for first model sample
    unsigned int i = modelSelection[randIdx];
    // Second model sample: Random != i
    unsigned int i2 = modelSelection[randIdx + rand()%remainingIdx];

    //LOGMSG(DBG_DEBUG, "Candidates: " << i << ", " << i2);

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
      for(unsigned int j2 = j + 1; j2 < SValid->getRows(); j2++)
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

        obvious::Matrix STemp = T * (*Control);

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
          err += dists[0][0];
          if(dists[0][0] < _epsSqr)
          {
            //err += dists[0][0];
            cntMatch++;
          }
        }
        delete[] indices.ptr();
        delete[] dists.ptr();

        if(cntMatch == 0)
          continue;

        //err /= cntMatch;

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

  delete Control;
  delete MValid;
  delete SValid;

  return TBest;
}

}
