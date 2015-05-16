#include "PCAMatching.h"

#include <math.h>
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

using namespace std;

namespace obvious
{

PCAMatching::PCAMatching(unsigned int trials, double epsThresh, unsigned int sizeControlSet)
{
  _epsSqr = epsThresh * epsThresh;
  _trials = trials;
  _sizeControlSet = sizeControlSet;
  _model = NULL;
  _index = NULL;
  _trace = NULL;
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

vector<unsigned int> PCAMatching::extractValidIndices(const obvious::Matrix* M, const bool* mask)
{
  vector<unsigned int> validIndices;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

void PCAMatching::initKDTree(const obvious::Matrix* M, vector<unsigned int> valid)
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

  const unsigned int pointsInS = S->getRows();
  const unsigned int pointsInM = M->getRows();

  const double transMaxSqr = transMax*transMax;

  if(pointsInM != pointsInS)
  {
    LOGMSG(DBG_ERROR, "Model and scene need to be of same size, size of M: " << pointsInM << ", size of S: " << pointsInS);
    return TBest;
  }

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
