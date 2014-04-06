#include "TsdGridLocalization.h"
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

TsdGridLocalization::TsdGridLocalization(TsdGrid* grid)
{
  _grid = grid;
}

TsdGridLocalization::~TsdGridLocalization()
{

}

Matrix TsdGridLocalization::localize(SensorPolar2D* sensor)
{
  Matrix Tfinal(3, 3);
  Tfinal.setIdentity();

  int sizeS     = sensor->getRealMeasurementSize();
  double* distS = sensor->getRealMeasurementData();
  double* scene = new double[sizeS*2];
  bool* maskS   = sensor->getRealMeasurementMask();

  unsigned int sizeM = sizeS;
  double* model      = new double[sizeM*2];
  double* normalsM   = new double[sizeM*2];
  bool* maskM        = new bool[sizeM];

  for(unsigned int i=0; i<15; i++)
  {
    _raycaster.calcCoordsFromCurrentViewMask(_grid, sensor, model, normalsM, maskM);

    // Compute centroid of scene
    Matrix* Map = sensor->getNormalizedRayMap(1.0);
    double cs[2];
    cs[0] = 0.0;
    cs[1] = 0.0;
    unsigned int validPoints = 0;
    for (int i = 0; i < sizeS; i++)
    {
      scene[2*i]   = distS[i]*(*Map)(0,i);
      scene[2*i+1] = distS[i]*(*Map)(1,i);
      if(maskM[i] && maskS[i])
      {
        cs[0] += scene[2*i];
        cs[1] += scene[2*i+1];
        validPoints++;
      }
    }
    double v = (double) validPoints;
    cs[0] /= v;
    cs[1] /= v;
    cout << "Centroid scene: " << cs[0] << " " << cs[1] << " valid points: " << validPoints << endl;

    // Compute centroid of model
    double cm[2];
    cm[0] = 0.0;
    cm[1] = 0.0;
    validPoints = 0;
    for (unsigned int i = 0; i < sizeM; i++)
    {
      if(maskM[i] && maskS[i])
      {
        cm[0] += model[2*i];
        cm[1] += model[2*i+1];
        validPoints++;
      }
    }
    v = (double) validPoints;
    cm[0] /= v;
    cm[1] /= v;

    cout << "Centroid model: " << cm[0] << " " << cm[1] << " valid points: " << validPoints << endl;

    double nominator = 0.0, denominator = 0.0;
    for (unsigned int i = 0; i < sizeM; i++)
    {
      if(maskM[i] && maskS[i])
      {
        double xm = model[2*i] - cm[0];
        double ym = model[2*i+1] - cm[1];
        double xs = scene[2*i] - cs[0];
        double ys = scene[2*i+1] - cs[1];
        nominator   += ym * xs - xm * ys;
        denominator += xm * xs + ym * ys;
      }
    }

    // compute rotation
    double deltaTheta = atan2(nominator, denominator);

    double cosDeltaTheta = cos(deltaTheta);
    double sinDeltaTheta = sin(deltaTheta);

    // compute translation
    double deltaX = (cm[0] - (cosDeltaTheta * cs[0] - sinDeltaTheta * cs[1]));
    double deltaY = (cm[1] - (sinDeltaTheta * cs[0] + cosDeltaTheta * cs[1]));

    // fill result matrix
    Matrix T(3, 3);
    T.setIdentity();
    T(0,0) = cosDeltaTheta;
    T(0,1) = -sinDeltaTheta;
    T(0,2) = deltaX;

    T(1,0) = sinDeltaTheta;
    T(1,1) = cosDeltaTheta;
    T(1,2) = deltaY;

    Tfinal = Tfinal * T;
    T.print();

    sensor->transform(&T);
  }

  delete [] scene;
  delete [] model;
  delete [] normalsM;
  delete [] maskM;


  return Tfinal;
}

}
