#include "PointToPointEstimator3D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/linalg.h"

using namespace obvious;

namespace obvious
{

PointToPointEstimator3D::PointToPointEstimator3D()
{
  _model  = NULL;
  _scene  = NULL;
  _rms    = 0.0;
  _cm[0]  = 0.0;
  _cm[1]  = 0.0;
  _cm[2]  = 0.0;
  _cs[0]  = 0.0;
  _cs[1]  = 0.0;
  _cs[2]  = 0.0;
  _pairs = NULL;
  _iterations = 0;
}

PointToPointEstimator3D::~PointToPointEstimator3D()
{
	
}

void PointToPointEstimator3D::setModel(double**model, unsigned int size, double** normals)  // normals are ignored in this class
{
  _model = model;
}

void PointToPointEstimator3D::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
  _scene = scene;
}

void PointToPointEstimator3D::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;

  _rms = 0.0;

  // Compute centroids
  _cm[0] = 0.0;         _cs[0] = 0.0;
  _cm[1] = 0.0;         _cs[1] = 0.0;
  _cm[2] = 0.0;         _cs[2] = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel = _model[pair.indexFirst];
    double* pointScene = _scene[pair.indexSecond];

    _cm[0] += pointModel[0];        _cs[0] += pointScene[0];
    _cm[1] += pointModel[1];        _cs[1] += pointScene[1];
    _cm[2] += pointModel[2];        _cs[2] += pointScene[2];

    _rms += distSqr3D(pointModel,pointScene);
  }
  double dSize = (double)size;
  _rms /= dSize;
  _cm[0] /= dSize;        _cs[0] /= dSize;
  _cm[1] /= dSize;        _cs[1] /= dSize;
  _cm[2] /= dSize;        _cs[2] /= dSize;

  _rms = sqrt(_rms);
}

double PointToPointEstimator3D::getRMS()
{
  return _rms;
}

unsigned int PointToPointEstimator3D::getIterations(void)
{
  return _iterations;
}

void PointToPointEstimator3D::estimateTransformation(Matrix* T)
{
  unsigned int i;
  int          r, c;

  double** pm;
  double** ps;

  int size = _pairs->size();
  _iterations++;

  System<double>::allocate(size,3, pm);
  System<double>::allocate(size,3, ps);

  //Calculate centered point pairs
  for(unsigned i = 0; i<(unsigned int)size; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];
    double* pointModel         = _model[pair.indexFirst];
    double* pointScene         = _scene[pair.indexSecond];
    pm[i][0]                   = pointModel[0] - _cm[0];
    pm[i][1]                   = pointModel[1] - _cm[1];
    pm[i][2]                   = pointModel[2] - _cm[2];
    ps[i][0]                   = pointScene[0] - _cs[0];
    ps[i][1]                   = pointScene[1] - _cs[1];
    ps[i][2]                   = pointScene[2] - _cs[2];
  }

  Matrix H(3, 3);

  for(r = 0; r < 3; r++) {
    for(c = 0; c < 3; c++) {
      double val = 0.0;
      for(i=0; i<(unsigned int)size; i++) {
        val += ps[i][r] * pm[i][c];
      }
      H[r][c] = val;
    }
  }

  double s[3];
  Matrix U(3, 3);
  Matrix V(3, 3);
  H.svd(&U, s, &V);

  // R = V * U'
  Matrix R(3, 3);
  U.transpose();
  R = V * U;

  Matrix tmp(3, 1, _cs);
  tmp = R * tmp;
  Matrix tr(3, 1, _cm);
  tr -= tmp;

  (*T)[0][0] = R[0][0];  (*T)[0][1] = R[0][1];  (*T)[0][2] = R[0][2];  (*T)[0][3] = tr[0][0];
  (*T)[1][0] = R[1][0];  (*T)[1][1] = R[1][1];  (*T)[1][2] = R[1][2];  (*T)[1][3] = tr[1][0];
  (*T)[2][0] = R[2][0];  (*T)[2][1] = R[2][1];  (*T)[2][2] = R[2][2];  (*T)[2][3] = tr[2][0];

  System<double>::deallocate(pm);
  System<double>::deallocate(ps);
}

}
