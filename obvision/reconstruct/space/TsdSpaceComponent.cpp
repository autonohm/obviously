#include "TsdSpaceComponent.h"
#include "obcore/math/mathbase.h"
#include <cmath>

namespace obvious
{

TsdSpaceComponent::TsdSpaceComponent(bool isLeaf)
{
  _isLeaf = isLeaf;
}

TsdSpaceComponent::~TsdSpaceComponent()
{

}

double TsdSpaceComponent::getComponentSize()
{
  return _componentSize;
}

double* TsdSpaceComponent::getCentroid()
{
  return _centroid;
}

double TsdSpaceComponent::getCircumradius()
{
  return _circumradius;
}

Matrix* TsdSpaceComponent::getEdgeCoordsHom()
{
  return _edgeCoordsHom;
}

bool TsdSpaceComponent::isLeaf()
{
  return _isLeaf;
}

bool TsdSpaceComponent::isInRange(double pos[3], Sensor* sensor, double maxTruncation)
{
  double range = sensor->getMaximumRange();
  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();

  // Centroid-to-sensor distance
  double distance = euklideanDistance<double>(pos, _centroid, 3);

  double minDist = distance - _circumradius - maxTruncation;
  if(!isnan(range))
  {
    if(minDist > range) return false;
  }
  double maxDist = distance + _circumradius + maxTruncation;


  /*Matrix C(1, 4);
  C(0,0) = _centroid[0];
  C(0,1) = _centroid[1];
  C(0,2) = _centroid[2];
  C(0,3) = 1.0;

  int idxCentroid;
  sensor->backProject(&C, &idxCentroid);

  if(idxCentroid!=-1)
  {
    if(mask[idxCentroid])
    {
      // not visible
      double sd = data[idxCentroid] - minDist;
      if(sd < -maxTruncation) return false;

      // empty space
      sd = data[idxCentroid] - maxDist;
      if(sd > maxTruncation)
      {
        increaseEmptiness();
        return false;
      }
    }
    else
      return false;
  }
  else
    return false;*/

  int width = sensor->getWidth();
  int height = sensor->getHeight();

  int idxEdge[8];
  sensor->backProject(_edgeCoordsHom, idxEdge);

  //_edgeCoordsHom->print();

  int x_min = width;
  int x_max = -1;
  int y_min = height;
  int y_max = -1;
  int validIndices = 0;

  for(int i=0; i<8; i++)
  {
    if(idxEdge[i]>-1)
    {
      int x_tmp = idxEdge[i] % width;
      int y_tmp = idxEdge[i] / width;
      if(x_min>x_tmp) x_min = x_tmp;
      if(x_max<x_tmp) x_max = x_tmp;
      if(y_min>y_tmp) y_min = y_tmp;
      if(y_max<y_tmp) y_max = y_tmp;
      validIndices++;
    }
    //cout << "idxEdge: " << idxEdge[i] << endl;
  }
//cout << validIndices << endl;
  if(validIndices==0) return false;
  //if(validIndices<8) return true;
  //cout << x_min << " - " << x_max << ", " << y_min << " - " << y_max << endl;

  bool isVisible = false;
  bool isEmpty = true;
  for(int y=y_min; y<=y_max; y++)
  {
    for(int x=x_min; x<=x_max; x++)
    {
      int idx = y*width+x;
      //cout << "idx: " << idx << " x: " << x << " y: " << y << endl;
      if(mask[idx])
      {
        //double sd = data[idx] - minDist;
        //isVisible = isVisible || (sd >= -maxTruncation);
        isVisible = isVisible || (data[idx] > minDist);
        if(isVisible)
        {
          //sd = data[idx] - maxDist;
          //isEmpty = isEmpty && (sd > maxTruncation);
          isEmpty = isEmpty && (data[idx] > maxDist);
        }
        //cout << "# " << sdf << " " << maxTruncation << " " << data[idx] << " " << minDist << " " << maxDist << " " << isEmpty << " " << maxTruncation << endl;
      }
      else
         cout << "filtered" << endl;
    }
  }
  //if(!noMeasurements && y_min != y_max) abort();
  /*if(noMeasurements)
  {
    for(int i=0; i<8; i++)
      cout << idxEdge[i] << " ";
    cout << endl;
    cout << y_min << " " << y_max << " " << x_min << " " << x_max << endl;
    isVisible = true;
  }*/
  //if(!isEmpty) abort();
  //if(x_max > x_min && y_max > y_min) cout << x_min << " " << x_max << " / " << y_min << " " << y_max << " visible: " << isVisible << " empty: " << isEmpty << " radius: " << _circumradius << " mindist: " << minDist << endl;

  /*
  bool isVisible = false;
  bool isEmpty = true;
  for(int i=0; i<8; i++)
  {
    int idx = idxEdge[i];
    if(idx!=-1)
    {
      if(mask[idx])
      {
        double sdf = data[idx] - minDist;
        isVisible = isVisible || (sdf >= -maxTruncation);
        sdf = data[idx] - maxDist;
        isEmpty = isEmpty && (sdf > maxTruncation);
      }
    }
  }*/
  if(!isVisible) return false;

  if(isEmpty)
  {
    //cout << "empty " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
    increaseEmptiness();
    return false;
  }

  return true;
}

}
