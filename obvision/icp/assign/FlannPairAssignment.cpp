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
  //flann::KDTreeIndexParams p(16);
  //flann::AutotunedIndexParams p;
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

// SM: Tested for speedup -> no effect over single index?? (13.10.2013)
/*void FlannPairAssignment::determinePairsSequential(double** scene, bool* mask, int size)
{
  double* buf = new double[size*_nDimension];
  unsigned int* idx = new unsigned int[size];
  unsigned int validPoints = 0;

  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      buf[_nDimension*validPoints]   = scene[i][0];
      buf[_nDimension*validPoints+1] = scene[i][1];
      if(_nDimension>=3)
        buf[_nDimension*validPoints+2] = scene[i][2];
      idx[validPoints] = i;
      validPoints++;
    }
    else
    {
      addNonPair(i);
    }
  }

  flann::Matrix<int> indices(new int[validPoints], validPoints, 1);
  flann::Matrix<double> dists(new double[validPoints], validPoints, 1);
  //flann::SearchParams p(-1, _eps);
  flann::SearchParams p(4, 0.01);

  flann::Matrix<double> query(buf, validPoints, _nDimension);
  _index->knnSearch(query, indices, dists, 1, p);

  for(unsigned int i = 0; i < validPoints; i++)
  {
    addPair(indices[i][0], idx[i], dists[i][0]);
  }

  delete[] indices.ptr();
  delete[] dists.ptr();

  delete[] buf;
  delete[] idx;
}*/

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
#pragma omp for schedule(dynamic)
  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      flann::Matrix<double> query(&scene[i][0], 1, _nDimension);
      int count = _index->knnSearch(query, indices, dists, 1, p);
      if(count > 0)
      {
#pragma omp critical
{
        addPair(indices[0][0], i, dists[0][0]);
}
      }
      else
      {
        cout << "Error: no data found" << endl;
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
  delete[] indices.ptr();
  delete[] dists.ptr();
}
}

}
