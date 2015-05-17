#include "PCAMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <limits>

using namespace std;

namespace obvious
{

PCAMatching::PCAMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet)
{
  _epsSqr         = epsThresh * epsThresh;
  _trials         = trials;
  _sizeControlSet = sizeControlSet;
  _model          = NULL;
  _index          = NULL;
  _trace          = NULL;
  _pcaCnt         = 10;
}

PCAMatching::~PCAMatching()
{
  if(_model)
  {
    delete _model;
    _model = NULL;
    delete _index;
    _index = NULL;
  }
  if(_trace)
    delete _trace;
  _trace = NULL;
}

void PCAMatching::activateTrace()
{
  if(!_trace)
    _trace = new Trace(2);
}

void PCAMatching::deactivateTrace()
{
  if(_trace) delete _trace;
  _trace = NULL;
}

vector<unsigned int> PCAMatching::extractSamples(const obvious::Matrix* M, const bool* mask)
{
  vector<unsigned int> validIndices;
  for(unsigned int i=_pcaCnt/2; i<M->getRows()-_pcaCnt/2; i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

void PCAMatching::initKDTree(const obvious::Matrix* M, vector<unsigned int> idxValid)
{
  // Build FLANN tree for fast access to nearest neighbors
  unsigned int cols = M->getCols();
  unsigned int rows = idxValid.size();
  double** mData;
  obvious::System<double>::allocate(rows, cols, mData);
  for(unsigned int r = 0; r < rows; r++)
  {
    mData[r][0] = (*M)(idxValid[r], 0);
    mData[r][1] = (*M)(idxValid[r], 1);
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

obvious::Matrix* PCAMatching::pickControlSet(const obvious::Matrix* M, vector<unsigned int> idxValid, vector<unsigned int> &idxControl)
{
  unsigned int sizeControlSet = _sizeControlSet;
  if((idxValid.size()) < sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size");
    sizeControlSet = idxValid.size();
  }
  obvious::Matrix* C = new obvious::Matrix(3, sizeControlSet);
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

#define MIN_VALID_POINTS 10
obvious::Matrix PCAMatching::match(const obvious::Matrix* M, const bool* maskM, const obvious::Matrix* S,  const bool* maskS, double phiMax, const double transMax, const double resolution)
{
  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  const int pointsInS = S->getRows();
  const int pointsInM = M->getRows();

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  vector<unsigned int> idxMValid = extractSamples(M, maskM);
  vector<unsigned int> idxSValid = extractSamples(S, maskS);

  initKDTree(M, idxMValid);
  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", Control set: " << Control->getCols());

  // Calculate search "radius", i.e., maximum difference in polar indices because of rotation
  phiMax = min(phiMax, M_PI * 0.5);
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

  // Determine orientation in local model neighborhood
  double* phiM = new double[pointsInM];
  for(int i=_pcaCnt/2; i<pointsInM-_pcaCnt/2; i++)
  {
    unsigned int cnt = 0;
    for(int j=-_pcaCnt/2; j<_pcaCnt/2; j++)
    {
      if(maskM[i+j]) cnt++;
    }
    if(cnt>4)
    {
      Matrix A(cnt, 2);
      cnt = 0;
      for(int j=-_pcaCnt/2; j<_pcaCnt/2; j++)
      {
        if(maskM[i+j])
        {
          A(cnt, 0) = (*M)(i+j, 0);
          A(cnt, 1) = (*M)(i+j, 1);
          cnt++;
        }
      }
      Matrix* Axes = A.pcaAnalysis();
      phiM[i] = atan2((*Axes)(0,3)-(*Axes)(0,2), (*Axes)(0,1)-(*Axes)(0,0));
      delete Axes;
    }
    else
      phiM[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // Determine number of valid samples in local scene neighborhood
  unsigned int* localSceneCnt = new unsigned int[pointsInS];
  for(int i=_pcaCnt/2; i<pointsInS-_pcaCnt/2; i++)
  {
    unsigned int cnt = 0;
    for(int j=-_pcaCnt/2; j<_pcaCnt/2; j++)
    {
      if(maskS[i+j]) cnt++;
    }
    localSceneCnt[i] = cnt;
  }

  unsigned int cntBest     = 0;
  double errBest           = 1e12;
  double cntRateBest       = 0;

#pragma omp parallel
{
  //cout<<"Number of Threads: "<< omp_get_num_threads()<<endl;
  #pragma omp for
  for(unsigned int trial = 0; trial < _trials; trial++)
  {
    const int randIdx     = rand() % (idxSValid.size()-1);
    const int idx         = idxSValid[randIdx];

    // Ensure at least a few samples for PCA analysis
    if(localSceneCnt[idx]>4)
    {
      Matrix A(localSceneCnt[idx], 2);
      unsigned int cnt = 0;
      for(int j=-_pcaCnt/2; j<_pcaCnt/2; j++)
      {
        if(maskS[idx+j])
        {
          A(cnt, 0) = (*S)(idx+j, 0);
          A(cnt, 1) = (*S)(idx+j, 1);
          cnt++;
        }
      }

      Matrix* Axes = A.pcaAnalysis();
      double phiS = atan2((*Axes)(0,3)-(*Axes)(0,2), (*Axes)(0,1)-(*Axes)(0,0));
      delete Axes;

      // leftmost scene point belonging to query point idx1
      const int iMin = max((int) idx-span, _pcaCnt/2);
      // rightmost scene point belonging to query point idx1
      const int iMax = min(idx+span, pointsInM-_pcaCnt/2);

      for(int i=iMin; i<iMax; i++)
      {
        if(maskM[i] && !isnan(phiM[i]))
        {
          double phi = phiM[i] - phiS;

          if(fabs(phi) < phiMax)
          {
            obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

            // Calculate translation
            T(0, 2) = (*M)(i,0) - (T(0, 0) * (*S)(idx,0) + T(0, 1) * (*S)(idx,1));
            T(1, 2) = (*M)(i,1) - (T(1, 0) * (*S)(idx,0) + T(1, 1) * (*S)(idx,1));

            obvious::Matrix STemp = T * (*Control);

            // Determine how many nearest neighbors (model <-> scene) are close enough
            double q[2];
            unsigned int cntMatch = 0;
            flann::Matrix<int> indices(new int[1], 1, 1);
            flann::Matrix<double> dists(new double[1], 1, 1);
            double err = 0;

            //We can cut off the outer parts of the scene/model as
            //the rotations tells how much information is not shared by the scans
            int clippedBeams = (int) (phi / resolution);

            unsigned int clippedPoints = 0;
            for(unsigned int s = 0; s < STemp.getCols(); s++)
            {
              /* Clip control points according to phi:
               *------------------------------------------
               * Cases:
               * for positive clipped points -> Scene is rotated left
               *    Scene uses: [0; size-clippedPoints] -> cut of left side
               *    Model uses: [0+clippedPoints; size] -> cut of right side
               * for negative clipped points. -> Scene is rotated right
               *    Scene uses: [0-clippedPoints; size] -> cut of right
               *    Model uses: [0; scene + clippedPoints] -> cut of left
               */

              if( idxControl[s] < (unsigned int) max(0, -clippedBeams) || idxControl[s] > min(pointsInS, pointsInS-clippedBeams) )
              {
                clippedPoints++;
                continue; // Cut of Scene Points, points that won't have a corresponding point due to rotation are ignored for the metric
              }

              //Find nearest neighbor for control point
              q[0] = STemp(0, s);
              q[1] = STemp(1, s);
              flann::Matrix<double> query(q, 1, 2);
              flann::SearchParams p(-1, 0.0);
              _index->knnSearch(query, indices, dists, 1, p);

              //Check if model point is not clipped
              unsigned int rawIdx = idxMValid[ indices[0][0] ]; //raw index of closest model point
              if(rawIdx < (unsigned int) max(0, clippedBeams) || rawIdx > min(pointsInS, pointsInS+clippedBeams))
              {
                clippedPoints++;
                continue; //Cut off point correspondences to Model points that don't have a reasonable corresponding point due to rotation.
              }

              err += dists[0][0];
              if(dists[0][0] < _epsSqr)
              {
                //err += sqrt(dists[0][0]);
                cntMatch++;
              }
            }
            err = sqrt(err);

            delete[] indices.ptr();
            delete[] dists.ptr();

            if(cntMatch == 0)
              continue;

            // Relative MatchCnt Score
            unsigned int maxMatchCnt = (STemp.getCols() - clippedPoints);
            double cntRate = (double)cntMatch / (double) maxMatchCnt;
            //double cntStepSize = 1.0 / STemp.getCols();
            double equalThres = 1e-5;//cntStepSize;// 1e-5;

#pragma omp critical
{
            bool rateCondition = ((cntRate - cntRateBest) > equalThres) && (cntMatch > cntBest);
            bool errorCondition = fabs( (cntRate-cntRateBest) < equalThres ) && (cntMatch == cntBest) && err < errBest;
            bool goodMatch = rateCondition || errorCondition;

            if(goodMatch)
            {
              errBest = err;
              cntBest = cntMatch;
              cntRateBest = cntRate;
              TBest = T;
            }
}
          }
        } // if(!isnan(phiM[i]))
      } // for(int i=_pcaCnt/2; i<pointsInM-_pcaCnt/2; i++)
    } //if(localSceneCnt[idx]>4)
  } // for _trials

} // OMP

  delete [] localSceneCnt;
  return TBest;
}

void PCAMatching::serializeTrace(const char* folder)
{
  if(_trace)
    _trace->serialize(folder);
  else
    LOGMSG(DBG_ERROR, "Trace not activated");
}

}
