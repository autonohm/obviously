#include "NaboPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

void NaboPairAssignment::init()
{
	_nns = NULL;
}

NaboPairAssignment::~NaboPairAssignment()
{
	if(_nns)
	{
		delete _nns;
		_nns = NULL;
	}
}

void NaboPairAssignment::setModel(double** model, int size)
{
	if(_nns)
	{
		delete _nns;
		_nns = NULL;
	}
	_ppdModel = model;
	_M.resize(_nDimension, size);
	for(int i=0; i<size; i++)
	  for(int j=0; j<_nDimension; j++)
	  {
	    _M(j, i) = model[i][j];
	  }
	_nns = NNSearchF::createKDTreeLinearHeap(_M);
}

void NaboPairAssignment::determinePairs(double** scene, bool* mask, int size)
{
  const int K = 1;
  VectorXi indices(K);
  VectorXf dists2(K);

  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      VectorXf q(_nDimension);
      for(int j=0; j<_nDimension; j++)
        q(j) = scene[i][j];
      _nns->knn(q, indices, dists2, K, 0, NNSearchF::ALLOW_SELF_MATCH);
      addPair((unsigned int)indices(0), i, (double)dists2(0));
    }
    else
    {
      addNonPair(i);
    }
  }
}

}
