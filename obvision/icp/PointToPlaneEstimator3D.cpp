#include "PointToPlaneEstimator3D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/linalg.h"

using namespace std;
using namespace obvious;

namespace obvious
{

PointToPlaneEstimator3D::PointToPlaneEstimator3D()
{
  _model   = NULL;
  _scene   = NULL;
  _normals = NULL;
  _pairs   = NULL;
  _iterations = 0;
}

PointToPlaneEstimator3D::~PointToPlaneEstimator3D()
{

}

void PointToPlaneEstimator3D::setModel(double** model, unsigned int size, double** normals)
{
  _model   = model;
  _normals = normals;
}

void PointToPlaneEstimator3D::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
  _scene = scene;
}

void PointToPlaneEstimator3D::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;

  _rms = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel = _model[pair.indexFirst];
    double* pointScene = _scene[pair.indexSecond];
    _rms += distSqr3D(pointModel,pointScene);
  }
  _rms /= (double)size;

  _rms = sqrt(_rms);
}

double PointToPlaneEstimator3D::getRMS()
{
	return _rms;
}

/**
 * Estimation based on paper: Sickel, Konrad; Bubnik, Vojtech, Iterative Closest Point Algorithm for Rigid Registration of Ear Impressions,
 * In: Bauman Moscow State Technical University (Eds.) Proceedings of the 6-th Russian-Bavarian Conference on Bio-Medical Engineering
 * (6th Russian Bavarian Conference on Bio-Medical Engineering Moscow, Russia 8-12.11.2010) 2010, pp. 142-145
 */
void PointToPlaneEstimator3D::estimateTransformation(Matrix* T)
{
  if(_normals==NULL)
  {
    cout << "WARNING (PointToPlaneEstimator3D::estimateTransformation): Normals not set." << endl;
    return;
  }
  unsigned int i;

  _iterations++;
  int size = _pairs->size();

  double A_buf[36];
  double b[6];

  for(unsigned int i=0; i<36; i++)
    A_buf[i] = 0.0;

  for(unsigned int i=0; i<6; i++)
    b[i] = 0.0;

  for(i = 0; i < (unsigned int)size; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];

    double* q         = _model[pair.indexFirst];
    double* p         = _scene[pair.indexSecond];
    double* n         = _normals[pair.indexFirst];

    double pxn[3];
    pxn[0] = p[1]*n[2] - p[2]*n[1];
    pxn[1] = p[2]*n[0] - p[0]*n[2];
    pxn[2] = p[0]*n[1] - p[1]*n[0];

    // fill upper right diagonal
    A_buf[0] +=pxn[0]*pxn[0];  A_buf[1] +=pxn[0]*pxn[1];  A_buf[2] +=pxn[0]*pxn[2];  A_buf[3] +=pxn[0]*n[0];  A_buf[4] +=pxn[0]*n[1];  A_buf[5]+=pxn[0]*n[2];
                               A_buf[7] +=pxn[1]*pxn[1];  A_buf[8] +=pxn[1]*pxn[2];  A_buf[9] +=pxn[1]*n[0];  A_buf[10]+=pxn[1]*n[1];  A_buf[11]+=pxn[1]*n[2];
                                                          A_buf[14]+=pxn[2]*pxn[2];  A_buf[15]+=pxn[2]*n[0];  A_buf[16]+=pxn[2]*n[1];  A_buf[17]+=pxn[2]*n[2];
                                                                                     A_buf[21]+=n[0]*n[0];    A_buf[22]+=n[0]*n[1];    A_buf[23]+=n[0]*n[2];
                                                                                                              A_buf[28]+=n[1]*n[1];    A_buf[29]+=n[1]*n[2];
                                                                                                                                       A_buf[35]+=n[2]*n[2];

    double tmp = ( (p[0]-q[0])*n[0] + (p[1]-q[1])*n[1] + (p[2]-q[2])*n[2]);

    b[0]-= pxn[0]*tmp;
    b[1]-= pxn[1]*tmp;
    b[2]-= pxn[2]*tmp;
    b[3]-= n[0]*tmp;
    b[4]-= n[1]*tmp;
    b[5]-= n[2]*tmp;
  }

  // copy lower left diagonal
  A_buf[6]  = A_buf[1];
  A_buf[12] = A_buf[2];        A_buf[13] = A_buf[8];
  A_buf[18] = A_buf[3];        A_buf[19] = A_buf[9];      A_buf[20] = A_buf[15];
  A_buf[24] = A_buf[4];        A_buf[25] = A_buf[10];     A_buf[26] = A_buf[16];    A_buf[27] = A_buf[22];
  A_buf[30] = A_buf[5];        A_buf[31] = A_buf[11];     A_buf[32] = A_buf[17];    A_buf[33] = A_buf[23];   A_buf[34] = A_buf[29];

  Matrix A(6, 6, A_buf);
  double x[6];
  A.solve(b, x);
  (*T)(0,3) = x[3];
  (*T)(1,3) = x[4];
  (*T)(2,3) = x[5];

  double cph = cos(x[0]);
  double cth = cos(x[1]);
  double cps = cos(x[2]);
  double sph = sin(x[0]);
  double sth = sin(x[1]);
  double sps = sin(x[2]);

  (*T)(0,0) = cth*cps;
  (*T)(0,1) = -cph*sps+sph*sth*cps;
  (*T)(0,2) = sph*sth+cph*sth*cps;

  (*T)(1,0) = cth*sps;
  (*T)(1,1) = cph*cps+sph*sth*sps;
  (*T)(1,2) = -sph*cps+cph*sth*sps;

  (*T)(2,0) = -sth;
  (*T)(2,1) = sph*cth;
  (*T)(2,2) = cph*cth;
}

unsigned int PointToPlaneEstimator3D::getIterations(void)
{
  return(_iterations);
}

}
