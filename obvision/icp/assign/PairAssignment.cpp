#include "PairAssignment.h"
#include <math.h>
#include "obcore/base/Timer.h"
namespace obvious
{

PairAssignment::PairAssignment()
{
	_nDimension   = DEFAULTDIMENSION;
	_pairs        = &_initPairs;
	_distancesSqr = &_initDistancesSqr;
}

PairAssignment::PairAssignment(int nDimension)
{
	_nDimension   = nDimension;
	_pairs        = &_initPairs;
	_distancesSqr = &_initDistancesSqr;
}

PairAssignment::~PairAssignment()
{
	_initPairs.clear();
	_nonPairs.clear();
	_initDistancesSqr.clear();
}

void PairAssignment::addPreFilter(IPreAssignmentFilter* filter)
{
  _vPrefilter.push_back(filter);
}

void PairAssignment::addPostFilter(IPostAssignmentFilter* filter)
{
  _vPostfilter.push_back(filter);
}

void PairAssignment::determinePairs(double** ppdScene, int nSize)
{
  unsigned int i;
  bool* mask = new bool[nSize];
  memset(mask, 1, nSize * sizeof(*mask));
  for(i=0; i<_vPrefilter.size(); i++)
  {
    IPreAssignmentFilter* filter = _vPrefilter[i];
    filter->filter(ppdScene, nSize, mask);
  }
  clearPairs();
  determinePairs(ppdScene, mask, nSize);

  _pairs        = &_initPairs;
  _distancesSqr = &_initDistancesSqr;

  bool hasPrecedingFilter = false;
  for(i=0; i<_vPostfilter.size(); i++)
  {
    IPostAssignmentFilter* filter = _vPostfilter[i];

    if(!filter->_active) continue;

    if(hasPrecedingFilter)
    {
      _initPairs.clear();
      _initDistancesSqr.clear();
      _initPairs         = _filteredPairs;
      _initDistancesSqr  = _filteredDistancesSqr;
    }

    filter->filter(_ppdModel,
                   ppdScene,
                   &_initPairs,
                   &_initDistancesSqr,
                   &_filteredPairs,
                   &_filteredDistancesSqr,
                   &_nonPairs);
    _pairs = &_filteredPairs;
    _distancesSqr = &_filteredDistancesSqr;

    hasPrecedingFilter = true;
  }

  delete[] mask;

}

vector<StrCartesianIndexPair>* PairAssignment::getPairs()
{
	return _pairs;
}

vector<double>* PairAssignment::getDistancesSqr()
{
	return _distancesSqr;
}

vector<unsigned int>* PairAssignment::getNonPairs()
{
	return &_nonPairs;	
}

void PairAssignment::addPair(unsigned int unIndexModel, unsigned int unIndexScene, double dDistanceSqr)
{
	StrCartesianIndexPair pair;
	pair.indexFirst = unIndexModel;
	pair.indexSecond = unIndexScene;
	_initPairs.push_back(pair);
	_initDistancesSqr.push_back(dDistanceSqr);
}

void PairAssignment::addNonPair(unsigned int unIndexScene)
{
	_nonPairs.push_back(unIndexScene);
}

int PairAssignment::getDimension()
{
	return _nDimension;	
}

void PairAssignment::clearPairs()
{
	_initPairs.clear();
	_filteredPairs.clear();
	_nonPairs.clear();
	_initDistancesSqr.clear();
	_filteredDistancesSqr.clear();
}

void PairAssignment::reset()
{
  clearPairs();

  for(unsigned int i=0; i<_vPostfilter.size(); i++)
  {
    IPostAssignmentFilter* filter = _vPostfilter[i];
    filter->reset();
  }
}

}	
