#include "RandomNormalMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <limits>

#include "obcore/base/Timer.h"

using namespace std;

namespace obvious
{

RandomNormalMatching::RandomNormalMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet)
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

RandomNormalMatching::~RandomNormalMatching()
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

void RandomNormalMatching::activateTrace()
{
  if(!_trace)
    _trace = new Trace(2);
}

void RandomNormalMatching::deactivateTrace()
{
  if(_trace) delete _trace;
  _trace = NULL;
}

vector<unsigned int> RandomNormalMatching::extractSamples(const obvious::Matrix* M, const bool* mask)
{
  vector<unsigned int> validIndices;
  for(unsigned int i=_pcaSearchRange/2; i<M->getRows()-_pcaSearchRange/2; i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

void RandomNormalMatching::initKDTree(const obvious::Matrix* M, vector<unsigned int> idxValid)
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

obvious::Matrix* RandomNormalMatching::pickControlSet(const obvious::Matrix* M, vector<unsigned int> idxValid, vector<unsigned int> &idxControl)
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

void RandomNormalMatching::calcNormals(const Matrix* M, Matrix* N, const bool* maskIn, bool* maskOut)
{
  int points = M->getRows();

  // mask borders at which we cannot calculate normal vectors
  for(int i=0; i<_pcaSearchRange/2; i++)
    maskOut[i] = false;
  for(int i=points-_pcaSearchRange/2; i<points; i++)
    maskOut[i] = false;

  for(int i=_pcaSearchRange/2; i<points-_pcaSearchRange/2; i++)
  {
    if(maskIn[i])
    {
      unsigned int cnt = 0;

      for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
        if(maskIn[i+j]) cnt++;

      if(cnt>_pcaMinSamples)
      {
        Matrix A(cnt, 2);
        cnt = 0;
        for(int j=-_pcaSearchRange/2; j<_pcaSearchRange/2; j++)
        {
          if(maskIn[i+j])
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

        if(lenShortSqr>1e-6 && (lenLongSqr/lenShortSqr)<4.0)
        {
          maskOut[i] = false;
          continue;
        }

        // shorter axis is normal
        double len = sqrt(lenShortSqr);
        if(((*M)(i,0)*xShort+(*M)(i,1)*yShort)<0.0)
        {
          (*N)(i, 0) = xShort / len;
          (*N)(i, 1) = yShort / len;
        }
        else
        {
          (*N)(i, 0) = -xShort / len;
          (*N)(i, 1) = -yShort / len;
        }

        delete Axes;
      }
      else
        maskOut[i] = false;
    }
  }
}

void RandomNormalMatching::calcPhi(const Matrix* N,  const bool* mask, double* phi)
{
  for(unsigned int i=0; i<N->getRows(); i++)
  {
    if(mask[i])
    {
      phi[i] = atan2((*N)(i,1), (*N)(i, 0));
    }
  }
}

void RandomNormalMatching::subsampleMask(bool* mask, unsigned int size, double probability)
{
  unsigned int sizeOut = 0;
  if(probability>1.0) probability = 1.0;
  if(probability<0.0) probability = 0.0;
  int probability_thresh = (int)(1000.0 - probability * 1000.0 + 0.5);
  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i])
    {
      if((rand()%1000)<probability_thresh)
      {
        mask[i] = 0;
        sizeOut++;
      }
    }
  }
}

obvious::Matrix RandomNormalMatching::match(const obvious::Matrix* M,
    const bool* maskM,
    const obvious::Matrix* NM,
    const obvious::Matrix* S,
    const bool* maskS,
    double phiMax,
    const double transMax,
    const double resolution)
{
  obvious::Matrix TBest(3, 3);
  TBest.setIdentity();

  const int pointsInM = M->getRows();
  const int pointsInS = S->getRows();

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

  // Determine orientation in local neighborhood
  // Only from these points a valid orientation is computable -> use a mask
  obvious::Matrix* NMpca = new Matrix(pointsInM, 2); // Normals for model
  obvious::Matrix* NSpca = new Matrix(pointsInS, 2); // Normals for scene
  double* phiM = new double[pointsInM]; // Orientation of model points
  double* phiS = new double[pointsInS]; // Orientation of scene points
  bool* maskMpca = new bool[pointsInM]; // Validity mask of model points
  bool* maskSpca = new bool[pointsInS]; // Validity mask of scene points
  // TODO: Array of structs (cache friendliness)

  memcpy(maskMpca, maskM, pointsInM*sizeof(bool));

  if(NM)
  {
    calcPhi(NM, maskM, phiM);
  }
  else // if normals are not supplied
  {
    calcNormals(M, NMpca, maskM, maskMpca);
    calcPhi(NMpca, maskM, phiM);
  }

  // Determine number of valid samples in local scene neighborhood
  // only from these points a valid orientation is computable
  memcpy(maskSpca, maskS, pointsInS*sizeof(bool));

  // Calculate probability of point masking
  unsigned int validPoints = 0;
  for(int i=0; i<pointsInS; i++)
    if(maskSpca[i]) validPoints++;

  double probability = 180.0/(double)pointsInS;
  if(probability<0.99)
    subsampleMask(maskSpca, pointsInS, probability);

  calcNormals(S, NSpca, maskS, maskSpca);
  calcPhi(NSpca, maskSpca, phiS);

  vector<unsigned int> idxMValid = extractSamples(M, maskMpca);
  vector<unsigned int> idxSValid = extractSamples(S, maskSpca);

  initKDTree(M, idxMValid);
  vector<unsigned int> idxControl;  //represents the indices of points used for Control in S.
  obvious::Matrix* Control = pickControlSet(S, idxSValid, idxControl);
  unsigned int sizeControl = Control->getCols();

  // Determine frustum, i.e., direction of leftmost and rightmost model point
  double phiBoundMin = atan2((*M)(idxMValid.front(),1), (*M)(idxMValid.front(),0));
  double phiBoundMax = atan2((*M)(idxMValid.back(),1),  (*M)(idxMValid.back(),0));

  LOGMSG(DBG_DEBUG, "Valid points in scene: " << idxSValid.size() << ", valid points in model: " << idxMValid.size() << ", Control set: " << Control->getCols());
  LOGMSG(DBG_DEBUG, "Model phi min:: " << rad2deg(phiBoundMin) << ", Model phi max: " << rad2deg(phiBoundMax));

  if(_trace)
  {
    _trace->reset();
    _trace->setModel(M, idxMValid);
    _trace->setScene(S, idxSValid);
  }

  // Calculate search "radius", i.e., maximum difference in polar indices because of rotation
  phiMax = min(phiMax, M_PI * 0.5);
  int span;
  if(resolution > 1e-6)
  {
    span = floor(phiMax / resolution);
    if(span > (int)pointsInM) span = (int)pointsInM;
  }
  else
  {
    LOGMSG(DBG_ERROR, "Resolution not properly set: resolution = " << resolution);
    return TBest;
  }

  srand (time(NULL));

  double       bestRatio = 0.0;
  unsigned int bestCnt = 0;
  double       bestErr = 1e12;
  //double       bestScore = 0.0;

#ifndef DEBUG
  // trace is only possible for single threaded execution
  if(_trace)
  {
    omp_set_num_threads(1);
    LOGMSG(DBG_WARN, "Configured single-threaded execution due to application of trace module");
  }
#endif

  //Timer t;
  //t.start();
#pragma omp parallel
  {
    bool* maskControl        = new bool[sizeControl];
#pragma omp for
    for(unsigned int trial = 0; trial < _trials; trial++)
    {
      int idx;
#pragma omp critical
      {
        const int randIdx = rand() % (idxMValid.size()-1); //TODO: rand_r
        idx               = idxMValid[randIdx];

        // remove chosen element to avoid picking same index a second time
        idxMValid.erase(idxMValid.begin() + randIdx);
      }

      // leftmost scene point belonging to query point idx1
      const int iMin = max(idx-span, _pcaSearchRange/2);
      // rightmost scene point belonging to query point idx1
      const int iMax = min(idx+span, pointsInS-_pcaSearchRange/2);


      for(int i=iMin; i<iMax; i++)
      {
        if(maskSpca[i])
        {
          double phi              = phiM[idx] - phiS[i];
          if(phi>M_PI)       phi -= 2.0*M_PI;
          else if(phi<-M_PI) phi += 2.0*M_PI;

          if(fabs(phi) < phiMax)
          {
            obvious::Matrix T = obvious::MatrixFactory::TransformationMatrix33(phi, 0, 0);

            // Calculate translation
            const double sx = (*S)(i,0);
            const double sy = (*S)(i,1);
            T(0, 2) = (*M)(idx,0) - (T(0, 0) * sx + T(0, 1) * sy);
            T(1, 2) = (*M)(idx,1) - (T(1, 0) * sx + T(1, 1) * sy);

            // Transform control set
            obvious::Matrix STemp = T * (*Control);
            unsigned int pointsInControl = STemp.getCols();
            assert(pointsInControl<sizeControl);

            // Determine number of control points in field of view
            unsigned int maxCntMatch = 0;
            for(unsigned int j=0; j<pointsInControl; j++)
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

            if(maxCntMatch<sizeControl/3) // TODO: Determine meaningful parameter
              continue;

            // Determine how many nearest neighbors (model <-> scene) are close enough
            double q[2];
            unsigned int cntMatch = 0;
            flann::Matrix<int> indices(new int[1], 1, 1);
            flann::Matrix<double> dists(new double[1], 1, 1);
            double errSum = 0;
            //double scoreSum = 0.0;

            for(unsigned int s = 0; s < pointsInControl; s++)
            {
              // clip points outside of model frustum
              if(maskControl[s])
              {
                // find nearest neighbor of control point
                q[0] = STemp(0, s);
                q[1] = STemp(1, s);
                flann::Matrix<double> query(q, 1, 2);
                flann::SearchParams p(-1, 0.0);
                _index->knnSearch(query, indices, dists, 1, p);

                int idxQuery = idxMValid[indices[0][0]];
                // should never happen
                // assert(maskM[idxQuery]);

                // some experimental ideas
                // 1) weight matching results with normal consensus
                double weight = fabs(phiM[idxQuery] - phiS[i]);
                // 2) calculate score, and give penalty to matches with wrong orientation
                //double score = cos(phiM[idxQuery] - phiS[i]);
                //errSum += weight*dists[0][0];

                errSum += dists[0][0] * weight;

                if(dists[0][0] < _epsSqr)
                {
                  cntMatch++;
                  //scoreSum += score;
                }
              }
            }

            delete[] indices.ptr();
            delete[] dists.ptr();

            if(cntMatch <= sizeControl/3)
              continue;

            // Experimental rating
            double ratio = (double)cntMatch / (double) maxCntMatch;

#pragma omp critical
{
              // Rating from Markus Kuehn
              double equalThres = 1e-5;//cntStepSize;// 1e-5;
              bool rateCondition = ((ratio-bestRatio) > equalThres) && (cntMatch > bestCnt);
              bool errorCondition = fabs( (ratio-bestRatio) < equalThres ) && (cntMatch == bestCnt) && errSum < bestErr;
              bool goodMatch = rateCondition || errorCondition;

              //bool goodMatch = (ratio>0.66) && (scoreSum > bestScore);

              if(goodMatch)
              {
                bestRatio = ratio;
                bestCnt = cntMatch;
                bestErr = errSum;
                TBest = T;
                //bestScore = scoreSum;
              }

}

            if(_trace)
            {
              //trace is only possible for single threaded execution
              vector<unsigned int> idxM;
              idxM.push_back(idx);
              vector<unsigned int> idxS;
              idxS.push_back(i);
              _trace->addAssignment(M, idxM, S, idxS, &STemp, errSum, trial);
            }

          }
        } // if(!isnan(phiM[i]))
      } // for(int i=_pcaCnt/2; i<pointsInM-_pcaCnt/2; i++)
    } // for _trials

    delete [] maskControl;

  } // OMP

  //cout << "elapsed: " << t.elapsed() << endl;
  //t.reset();

  delete NMpca;
  delete NSpca;
  delete [] phiM;
  delete [] phiS;
  delete [] maskMpca;
  delete [] maskSpca;
  delete Control;

  return TBest;
}

void RandomNormalMatching::serializeTrace(const char* folder)
{
  if(_trace)
    _trace->serialize(folder);
  else
    LOGMSG(DBG_ERROR, "Trace not activated");
}

}
