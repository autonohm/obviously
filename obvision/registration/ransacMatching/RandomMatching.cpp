#include "RandomMatching.h"

#include "obcore/base/Logger.h"

namespace obvious
{

RandomMatching::RandomMatching(unsigned int sizeControlSet)
{
  _sizeControlSet   = sizeControlSet;
}

RandomMatching::~RandomMatching()
{
  if(_trace)
    delete _trace;
  _trace = NULL;
}

void RandomMatching::activateTrace()
{
  if(!_trace)
    _trace = new Trace(2);
}

void RandomMatching::deactivateTrace()
{
  if(_trace) delete _trace;
  _trace = NULL;
}

void RandomMatching::serializeTrace(const char* folder)
{
  if(_trace)
    _trace->serialize(folder);
  else
    LOGMSG(DBG_ERROR, "Trace not activated");
}

std::vector<unsigned int> RandomMatching::extractSamples(const obvious::Matrix* M, const bool* mask, unsigned int searchRange)
{
  std::vector<unsigned int> validIndices;
  for(unsigned int i=searchRange; i<M->getRows()-searchRange; i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

obvious::Matrix* RandomMatching::pickControlSet(const obvious::Matrix* M, std::vector<unsigned int> idxValid, std::vector<unsigned int> &idxControl)
{
  unsigned int sizeControlSet = _sizeControlSet;
  if((idxValid.size()) < sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size to " << idxValid.size());
    sizeControlSet = idxValid.size();
  }
  obvious::Matrix* C = new obvious::Matrix(3, sizeControlSet);
  std::vector<unsigned int> idxTemp = idxValid;
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


} /* namespace obvious */
