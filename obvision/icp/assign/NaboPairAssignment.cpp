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
	_model = model;
	_M.resize(_dimension, size);
	for(int i=0; i<size; i++)
	  for(int j=0; j<_dimension; j++)
	  {
	    _M(j, i) = (float)model[i][j];
	  }
	_nns = NNSearchF::createKDTreeLinearHeap(_M);
}

void NaboPairAssignment::determinePairs(double** scene, bool* mask, int size)
{
#pragma omp parallel
{
  VectorXi indices(1);
  VectorXf dists2(1);
#pragma omp for schedule(dynamic)
  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      VectorXf q(_dimension);
      for(int j=0; j<_dimension; j++)
        q(j) = scene[i][j];
      _nns->knn(q, indices, dists2, 1, 0, NNSearchF::ALLOW_SELF_MATCH);
#pragma omp critical
{
      addPair((unsigned int)indices(0), i, (double)dists2(0));
}
    }
    else
    {
#pragma omp critical
{
      addNonPair(i);
}
    }
  }
}
}

}
