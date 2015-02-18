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

RansacMatching::RansacMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet)
{
  _epsSqr = epsThresh * epsThresh;
  _trials = trials;
  _sizeControlSet = sizeControlSet;
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

vector<unsigned int> RansacMatching::extractValidIndices(const obvious::Matrix* M, const bool* mask)
{
  vector<unsigned int> validIndices;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

void RansacMatching::initKDTree(const obvious::Matrix* M, vector<unsigned int> valid)
{
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  unsigned int rows = valid.size();
  double** mData;
  obvious::System<double>::allocate(rows, cols, mData);
  for(unsigned int r = 0; r < rows; r++)
  {
    mData[r][0] = (*M)(valid[r], 0);
    mData[r][1] = (*M)(valid[r], 1);
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

obvious::Matrix* RansacMatching::pickControlSet(const obvious::Matrix* M, vector<unsigned int> idxValid)
{
  unsigned int sizeControlSet = _sizeControlSet;
  if((idxValid.size()) < sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size");
    sizeControlSet = idxValid.size();
  }
  obvious::Matrix* C = new obvious::Matrix(3, _sizeControlSet);
  vector<unsigned int> idxControl;
  vector<unsigned int> idxTemp = idxValid;
  unsigned int ctr = 0;
  while(idxControl.size() < sizeControlSet)
  {
    unsigned int r = rand() % idxTemp.size();
    unsigned int idx = idxTemp[r];
    idxControl.push_back(idx);
    idxTemp.erase(idxTemp.begin() + r);

    (*C)(0, ctr)   = (*M)(idx, 0);
    (*C)(1, ctr)   = (*M)(idx, 1);
    (*C)(2, ctr++) = 1.0;
  }
  return C;
}

double** RansacMatching::createLutIntraDistance(const obvious::Matrix* M, const bool* mask, int maxDist)
{
  int points = (int)M->getRows();
  double** dists;
  obvious::System<double>::allocate(points, points, dists);
  for(int i = 0; i < points; i++)
  {
    if(mask[i])
    {
      int jmax = min(i+maxDist, points);
      for(int j = i+1; j < jmax; j++)
      {
        if(mask[j])
        {
          double dx = (*M)(j, 0) - (*M)(i, 0);
          double dy = (*M)(j, 1) - (*M)(i, 1);
          dists[i][j] = dx * dx + dy * dy;
        }
      }
    }
  }
  return dists;
}

#define MIN_VALID_POINTS 10
obvious::Matrix RansacMatching::match(const obvious::Matrix* M,  const bool* maskM, const obvious::Matrix* S,  const bool* maskS, double phiMax, double resolution)
{
  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  unsigned int pointsInS = S->getRows();
  unsigned int pointsInM = M->getRows();

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  vector<unsigned int> idxMValid = extractValidIndices(M, maskM);
  vector<unsigned int> idxSValid = extractValidIndices(S, maskS);

  if(idxMValid.size() < MIN_VALID_POINTS || idxSValid.size() < MIN_VALID_POINTS)
  {
    LOGMSG(DBG_ERROR, "Model or scene contain too less valid points, valid in M: " << idxMValid.size() << ", valid in S: " << idxSValid.size());
    return TBest;
  }

  // Calculate search "radius", i.e., maximum difference in polar indices because of rotation
  phiMax = min(phiMax, M_PI / 2.0);
  int span;
  if(resolution > 1e-6)
  {
    span = (int)(phiMax / resolution);
    if(span > (int)pointsInM) span = (int)pointsInM;
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: resolution = " << resolution);
    return TBest;
  }

  // -----------------------------------------------------
  // Map query point in M -> left bound in S
  vector<unsigned int> mapMtoSBoundL;
  // Map query point in M -> right bound in S
  vector<unsigned int> mapMtoSBoundR;

  // determine valid right bounding in S for leftmost query point in M
  unsigned int idxR;
  for(int i=0; i<span-1; i++)
    if(maskS[i]) idxR = i;

  // valid left bounding in S for leftmost query point in M
  unsigned int idxL = idxSValid.front();

  for(int i=0; i<(int)pointsInM; i++)
  {
    unsigned int iL = max(i-span, 0);
    if(maskS[iL]) idxL = iL;
    mapMtoSBoundL.push_back(idxL);

    unsigned int iR = min(i+span, (int)pointsInS-1);
    if(maskS[iR]) idxR = iR;
    mapMtoSBoundR.push_back(idxR);
  }
  // -----------------------------------------------------

  initKDTree(M, idxMValid);

  obvious::Matrix* Control = pickControlSet(S, idxSValid);

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", Control set: " << Control->getCols());

  unsigned int maxDist2ndSample = (M_PI/6.0) / resolution;
  double** SDists = createLutIntraDistance(S, maskS, maxDist2ndSample);

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
    unsigned int randIdx      = rand() % (idxMValid.size()-10);
    // ... and leave at least n points for 2nd choice
    unsigned int remainingIdx = min((unsigned int)(idxMValid.size()-randIdx-1), maxDist2ndSample);
    // Index for first model sample
    unsigned int idx1         = idxMValid[randIdx];
    // Second model sample: Random on right side != i
    unsigned int idx2         = idxMValid[randIdx + rand()%remainingIdx];

    //LOGMSG(DBG_DEBUG, "Candidates: " << i << ", " << i2);

    // Vector between model points (for determining orientation)
    double vM[2];
    vM[0] = (*M)(idx2, 0) - (*M)(idx1, 0);
    vM[1] = (*M)(idx2, 1) - (*M)(idx1, 1);

    // Centroid of model (for determining translation)
    double cM[2];
    cM[0] = ((*M)(idx1, 0) + (*M)(idx2, 0)) / 2.0;
    cM[1] = ((*M)(idx1, 1) + (*M)(idx2, 1)) / 2.0;

    double distM = vM[0] * vM[0] + vM[1] * vM[1];

    // coordinates of point in S with similar intra-distance
    double sceneSimilar[2];

    // leftmost scene point belonging to query point idx1
    unsigned int iMin = mapMtoSBoundL[idx1];
    // rightmost scene point belonging to query point idx1
    unsigned int iMax = mapMtoSBoundR[idx1];

    //LOGMSG(DBG_DEBUG, "Search range: " << jMin << " " << jMax);
    for(unsigned int i = iMin; i < iMax; i++)
    {
      // Find scene sample with similar distance
      unsigned int iMinDist = 0;
      double distSMin       = 1e12;
      unsigned int i2max    = min((unsigned int)idxSValid.size(), i+maxDist2ndSample);
      for(unsigned int i2 = i + 1; i2 < i2max; i2++)
      {
        double distEps = fabs(SDists[i][i2] - distM);
        if(distEps < distSMin)
        {
          distSMin = distEps;
          iMinDist = i2;
        }
      }

      if(distSMin < _epsSqr)
      {
        sceneSimilar[0] = (*S)(iMinDist, 0);
        sceneSimilar[1] = (*S)(iMinDist, 1);
      }
      else
        continue;

      // Align scans
      double vS[2];
      vS[0] = sceneSimilar[0] - (*S)(i, 0);
      vS[1] = sceneSimilar[1] - (*S)(i, 1);

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
        cS[0] = (sceneSimilar[0] + (*S)(i, 0)) / 2.0;
        cS[1] = (sceneSimilar[1] + (*S)(i, 1)) / 2.0;

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

  delete Control;

  return TBest;
}

}
