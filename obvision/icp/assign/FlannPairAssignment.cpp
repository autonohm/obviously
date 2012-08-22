#include "FlannPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

void FlannPairAssignment::init(double eps)
{
  _dataset = NULL;
  _eps     = eps;
}

FlannPairAssignment::~FlannPairAssignment()
{
  if(_dataset)
  {
    delete _dataset;
    _dataset = NULL;
    delete _index;
    _index = NULL;
  }
}

void FlannPairAssignment::setModel(double** model, int size)
{
  if(_dataset)
  {
    delete _dataset;
    _dataset = NULL;
    delete _index;
    _index = NULL;
  }
  _ppdModel = model;

  _dataset = new flann::Matrix<double>(&model[0][0], size, _nDimension);
  flann::KDTreeSingleIndexParams p;
  //p.algorithm = FLANN_INDEX_KDTREE_SINGLE;
  //p.target_precision = 0.9;
  _index = new flann::Index<flann::L2<double> >(*_dataset, p);
  _index->buildIndex();
}

void FlannPairAssignment::determinePairs(double** scene, bool* mask, int size)
{
  flann::Matrix<int> indices(new int[1], 1, 1);
  flann::Matrix<double> dists(new double[1], 1, 1);
  flann::SearchParams p(-1, _eps);

  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      flann::Matrix<double> query(&scene[i][0], 1, _nDimension);
      _index->knnSearch(query, indices, dists, 1, p);
      addPair(indices[0][0], i, dists[0][0]);
    }
    else
    {
      addNonPair(i);
    }
  }

  delete[] indices.ptr();
  delete[] dists.ptr();
}

}
