#include "FlannPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

FlannPairAssignment::FlannPairAssignment(int dimension, double eps, bool parallelSearch) : PairAssignment(dimension)
{
  _useParallelVersion = parallelSearch;
  init(eps);
};

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

void FlannPairAssignment::init(double eps)
{
  _dataset = NULL;
  _eps     = eps;
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
  if(_useParallelVersion)
    determinePairsParallel(scene, mask, size);
  else
    determinePairsSequential(scene, mask, size);
}

void FlannPairAssignment::determinePairsSequential(double** scene, bool* mask, int size)
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

void FlannPairAssignment::determinePairsParallel(double** scene, bool* mask, int size)
{
#pragma omp parallel
{
  flann::Matrix<int> indices(new int[1], 1, 1);
  flann::Matrix<double> dists(new double[1], 1, 1);
  flann::SearchParams p(-1, _eps);
  unsigned int* buf_pair       = new unsigned int[size];
  unsigned int* buf_nonpair    = new unsigned int[size];
  unsigned int* buf_idx        = new unsigned int[size];
  double* buf_dist             = new double[size];
  unsigned int size_pairs      = 0;
  unsigned int size_nonpairs   = 0;
#pragma omp for schedule(dynamic)
  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      flann::Matrix<double> query(&scene[i][0], 1, _nDimension);
      _index->knnSearch(query, indices, dists, 1, p);
      buf_pair[size_pairs]   = indices[0][0];
      buf_idx[size_pairs]    = i;
      buf_dist[size_pairs++] = dists[0][0];
    }
    else
    {
      buf_nonpair[size_nonpairs++] = i;
    }
  }

#pragma omp critical
{
  for(unsigned int j = 0; j < size_pairs; j++)
  {
    addPair(buf_pair[j], buf_idx[j], buf_dist[j]);
  }
}

#pragma omp critical
{
  for(unsigned int k = 0; k < size_nonpairs; k++)
  {
    addNonPair(buf_nonpair[k]);
  }
}
  delete[] buf_pair;
  delete[] buf_nonpair;
  delete[] buf_idx;
  delete[] buf_dist;
  delete[] indices.ptr();
  delete[] dists.ptr();
}
}

}
