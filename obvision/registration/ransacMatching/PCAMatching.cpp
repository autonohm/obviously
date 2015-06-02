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
  _pcaSearchRange = 10;
  _pcaMinSamples  = 3;
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
  for(unsigned int i=_pcaSearchRange/2; i<M->getRows()-_pcaSearchRange/2; i++)
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

obvious::Matrix PCAMatching::match(const obvious::Matrix* M,
                                   const bool* maskM,
                                   const obvious::Matrix* N,
                                   const obvious::Matrix* S,
                                   const bool* maskS,
                                   double phiMax,
                                   const double transMax,
                                   const double resolution)
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

  // Determine orientation in local model neighborhood
  // Only from these points a valid orientation is computable
  double* phiM = new double[pointsInM];
  bool* maskMpca = new bool[pointsInM];
  memcpy(maskMpca, maskM, pointsInM*sizeof(bool));
  if(N)
  {
    for(int i=0; i<pointsInM; i++)
    {
      phiM[i] = atan2((*N)(i, 1), (*N)(i, 0));
    }
  }
  else
  {
    for(int i=0; i<_pcaSearchRange/2; i++)
      maskMpca[i] = false;
    for(int i=pointsInM-_pcaSearchRange/2; i<pointsInM; i++)
      maskMpca[i] = false;
    for(int i=_pcaSearchRange/2; i<pointsInM-_pcaSearchRange/2; i++)
    {
      if(maskM[i])
      {
        unsigned int cnt = 0;
        for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
        {
          if(maskM[i+j]) cnt++;
        }
        if(cnt>_pcaMinSamples)
        {
          Matrix A(cnt, 2);
          cnt = 0;
          for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
          {
            if(maskM[i+j])
            {
              A(cnt, 0) = (*M)(i+j, 0);
              A(cnt, 1) = (*M)(i+j, 1);
              cnt++;
            }
          }
          Matrix* Axes = A.pcaAnalysis();
          // longer axis
          double xLong  = (*Axes)(0,1)-(*Axes)(0,0);
          double yLong  = (*Axes)(0,3)-(*Axes)(0,2);
          // shorter axis
          double xShort = (*Axes)(1,1)-(*Axes)(1,0);
          double yShort = (*Axes)(1,3)-(*Axes)(1,2);
          // rate axes lengths -> main axis needs to be twice as long as second axis
          double lenLongSqr  = xLong*xLong + yLong*yLong;
          double lenShortSqr = xShort*xShort + yShort*yShort;
          assert(lenLongSqr>lenShortSqr);
          if(lenShortSqr>1e-6 && (lenLongSqr/lenShortSqr)<4.0) maskMpca[i] = false;

          // shorter axis is normal
          phiM[i] = atan2(yShort, xShort);

          // correct orientation with cross product
          if(((*M)(i,0)*xShort+(*M)(i,1)*yShort)>0.0)
            phiM[i] = -phiM[i];

          delete Axes;
        }
        else
          maskMpca[i] = false;
      }
    }
  }

  /*for(int i=0; i<pointsInM; i++)
    cout << phiM[i] << ", " << phiM2[i] << ", Mask: " << maskM[i] << endl;*/

  vector<unsigned int> idxMValid = extractSamples(M, maskMpca);
  vector<unsigned int> idxSValid = extractSamples(S, maskS);

  initKDTree(M, idxMValid);
  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);
  unsigned int sizeControl = Control->getCols();
  bool* maskControl        = new bool[sizeControl];

  // Determine frustum, i.e., direction of leftmost and rightmost model point
  double phiBoundMin = atan2((*M)(idxMValid.front(),1), (*M)(idxMValid.front(),0));
  double phiBoundMax = atan2((*M)(idxMValid.back(),1),  (*M)(idxMValid.back(),0));

  cout << "Valid points in scene: " << idxSValid.size() << ", points in M: " << pointsInM << ", valid points in model: " << idxMValid.size() << ", Control set: " << sizeControl << endl;
  cout << "Model frustum: [" << rad2deg(phiBoundMin) << "; " << rad2deg(phiBoundMax) << "]" << endl;

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", valid points in model: " << idxMValid.size() << ", Control set: " << Control->getCols());

  LOGMSG(DBG_DEBUG, "Model phi min:: " << rad2deg(phiBoundMin) << ", Model phi max: " << rad2deg(phiBoundMax));

  if(_trace)
  {
    double** rawModel;
    double** rawScene;
    System<double>::allocate(idxMValid.size(), 2, rawModel);
    System<double>::allocate(idxSValid.size(), 2, rawScene);
    for(unsigned int i=0; i<idxMValid.size(); i++)
    {
      rawModel[i][0] = (*M)(idxMValid[i], 0);
      rawModel[i][1] = (*M)(idxMValid[i], 1);
    }
    for(unsigned int i=0; i<idxSValid.size(); i++)
    {
      rawScene[i][0] = (*S)(idxSValid[i], 0);
      rawScene[i][1] = (*S)(idxSValid[i], 1);
    }
    _trace->reset();
    _trace->setModel(rawModel, idxMValid.size());
    _trace->setScene(rawScene, idxSValid.size());

    System<double>::deallocate(rawModel);
    System<double>::deallocate(rawScene);
  }

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

  // Determine number of valid samples in local scene neighborhood
  // only from these points a valid orientation is computable
  unsigned int* localSceneCnt = new unsigned int[pointsInS];
  for(int i=_pcaSearchRange/2; i<pointsInS-_pcaSearchRange/2; i++)
  {
    unsigned int cnt = 0;
    for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
    {
      if(maskS[i+j]) cnt++;
    }
    localSceneCnt[i] = cnt;
  }

  srand (time(NULL));

#ifndef DEBUG
if (_trace)
{
  omp_set_num_threads(1);
}
#endif

int          bestM;
int          bestS;
double       bestRatio = 0.0;
double       bestScore = M_PI / 2.0;
unsigned int bestCnt = 0;
double       bestErr = 1e12;

#pragma omp parallel
{
  #pragma omp for
  for(unsigned int trial = 0; trial < _trials; trial++)
  {
    int idx;
#pragma omp critical
{
    const int randIdx = rand() % (idxSValid.size()-1);
    idx               = idxSValid[randIdx];

    // remove chosen element to avoid picking same index a second time
    idxSValid.erase(idxSValid.begin() + randIdx);
}
    // Ensure that scene sample has at least a few valid neighbors for PCA analysis
    if(localSceneCnt[idx]>_pcaMinSamples)
    {

      // ----------- Determine orientation of scene sample ------------
      Matrix A(localSceneCnt[idx], 2);
      unsigned int cnt = 0;
      for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
      {
        if(maskS[idx+j])
        {
          A(cnt, 0) = (*S)(idx+j, 0);
          A(cnt, 1) = (*S)(idx+j, 1);
          cnt++;
        }
      }
      Matrix* Axes = A.pcaAnalysis();
      // longer axis
      double xLong  = (*Axes)(0,1)-(*Axes)(0,0);
      double yLong  = (*Axes)(0,3)-(*Axes)(0,2);
      // shorter axis
      double xShort = (*Axes)(1,1)-(*Axes)(1,0);
      double yShort = (*Axes)(1,3)-(*Axes)(1,2);
      // rate axes lengths -> main axis needs to be twice as long as second axis
      double lenLongSqr  = xLong*xLong + yLong*yLong;
      double lenShortSqr = xShort*xShort + yShort*yShort;
      assert(lenLongSqr>lenShortSqr);
      if(lenShortSqr>1e-6 && (lenLongSqr/lenShortSqr)<4.0) continue;

      // shorter axis is normal
      double phiS = atan2(yShort, xShort);

      // correct orientation with cross product
      if(((*S)(idx,0)*xShort+(*S)(idx,1)*yShort)>0.0)
        phiS = -phiS;

      delete Axes;
      // --------------------------------------------------------------

      // leftmost scene point belonging to query point idx1
      const int iMin = max((int) idx-span, _pcaSearchRange/2);
      // rightmost scene point belonging to query point idx1
      const int iMax = min(idx+span, pointsInM-_pcaSearchRange/2);

      for(int i=iMin; i<iMax; i++)
      {
        if(maskMpca[i])
        {
          double phi              = phiM[i] - phiS;
          if(phi>M_PI)       phi -= 2.0*M_PI;
          else if(phi<-M_PI) phi += 2.0*M_PI;

          if(fabs(phi) < phiMax)
          {
            obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

            // Calculate translation
            const double sx = (*S)(idx,0);
            const double sy = (*S)(idx,1);
            T(0, 2) = (*M)(i,0) - (T(0, 0) * sx + T(0, 1) * sy);
            T(1, 2) = (*M)(i,1) - (T(1, 0) * sx + T(1, 1) * sy);

            //T.print();
            // Transform control set
            obvious::Matrix STemp = T * (*Control);

            // Determine number of control points in field of view
            unsigned int maxCntMatch = 0;
            for(unsigned int j=0; j<STemp.getCols(); j++)
            {
              double phiControl = atan2(STemp(1, j), STemp(0, j));
              if(phiControl>phiBoundMax || phiControl<phiBoundMin)
              {
                maskControl[j] = false;
              }
              else
              {
                maskControl[j] = true;
                maxCntMatch++;
              }
            }

            if(maxCntMatch<sizeControl/3)
              continue;

            // Determine how many nearest neighbors (model <-> scene) are close enough
            double q[2];
            unsigned int cntMatch = 0;
            flann::Matrix<int> indices(new int[1], 1, 1);
            flann::Matrix<double> dists(new double[1], 1, 1);
            double err = 0;

            for(unsigned int s = 0; s < STemp.getCols(); s++)
            {
              if(maskControl[s])
              {
                //Find nearest neighbor for control point
                q[0] = STemp(0, s);
                q[1] = STemp(1, s);
                flann::Matrix<double> query(q, 1, 2);
                flann::SearchParams p(-1, 0.0);
                _index->knnSearch(query, indices, dists, 1, p);

                err += dists[0][0];
                if(dists[0][0] < _epsSqr)
                {
                  cntMatch++;
                }
              }
            }

            //cout << "idx: " << idx << ", i:" << i << ", phiM:" << rad2deg(phiM[i]) << ", phiS: " << rad2deg(phiS) << ", phi: " << rad2deg(fabs(phi)) << ", cntMatch: " << cntMatch << ", maxCntMatch: " << maxCntMatch << endl;
            delete[] indices.ptr();
            delete[] dists.ptr();

            if(cntMatch <= sizeControl/3)
              continue;

            // Experimental rating
            double ratio = (double)cntMatch / (double) maxCntMatch;
            double errMean = err / (double)cntMatch;
            double score = atan2(errMean,(double)cntMatch);
            //bool goodMatch = score<bestScore && cntMatch>bestCnt;

            // Rating of Markus Kuehn
            double equalThres = 1e-5;//cntStepSize;// 1e-5;
            bool rateCondition = ((ratio-bestRatio) > equalThres) && (cntMatch > bestCnt);
            bool errorCondition = fabs( (ratio-bestRatio) < equalThres ) && (cntMatch == bestCnt) && err < bestErr;
            bool goodMatch = rateCondition || errorCondition;


            //cout << errMean << " " << cntMatch << " " << rad2deg(score) << endl;
#pragma omp critical
{
            if(goodMatch)
            {
              bestScore = score;
              bestRatio = ratio;
              bestCnt = cntMatch;
              bestErr = err;
              bestM = i;
              bestS = idx;
              TBest = T;
            }
}

            if(_trace)
            {
              double** rawScene;
              System<double>::allocate(STemp.getCols(), 2, rawScene);
              for(unsigned int j=0; j<STemp.getCols(); j++)
              {
                rawScene[j][0] = STemp(0, j);
                rawScene[j][1] = STemp(1, j);
              }
              vector<StrTraceCartesianPair> tracePair;
              StrTraceCartesianPair p;
              p.first[0] = (*M)(i, 0);
              p.first[1] = (*M)(i, 1);
              p.second[0] = (*S)(idx, 0);
              p.second[1] = (*S)(idx, 1);
              tracePair.push_back(p);
              vector<unsigned int> id;
              id.push_back(trial);
              id.push_back(i);
              id.push_back(idx);
              _trace->addAssignment(rawScene, STemp.getCols(), tracePair, err, id);
              System<double>::deallocate(rawScene);
            }
          }
        } // if(!isnan(phiM[i]))
      } // for(int i=_pcaCnt/2; i<pointsInM-_pcaCnt/2; i++)
    } //if(localSceneCnt[idx]>4)
  } // for _trials

} // OMP

  cout << "Best match: " << bestM << " " << bestS << " bestRatio: " << bestRatio << " bestCnt: " << bestCnt << " bestScore: " << bestScore << endl;
  delete [] maskControl;
  delete [] phiM;
  delete [] maskMpca;
  delete [] localSceneCnt;
  delete Control;

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
