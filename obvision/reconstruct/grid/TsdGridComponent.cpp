#include "TsdGridComponent.h"
#include "obcore/math/mathbase.h"
#include <cmath>

namespace obvious
{

TsdGridComponent::TsdGridComponent(bool isLeaf)
{
  _isLeaf = isLeaf;
}

TsdGridComponent::~TsdGridComponent()
{

}

double TsdGridComponent::getComponentSize()
{
  return _componentSize;
}

double* TsdGridComponent::getCentroid()
{
  return _centroid;
}

double TsdGridComponent::getCircumradius()
{
  return _circumradius;
}

Matrix* TsdGridComponent::getEdgeCoordsHom()
{
  return _edgeCoordsHom;
}

bool TsdGridComponent::isLeaf()
{
  return _isLeaf;
}

bool TsdGridComponent::isInRange(double pos[2], Sensor* sensor, double maxTruncation)
{
  double range = sensor->getMaximumRange();
  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();

  // Centroid-to-sensor distance
  double distance = euklideanDistance<double>(pos, _centroid, 2);
  if(!isnan(range))
  {
    if((distance-_circumradius) > range) return false;
  }

  int idxEdge[4];
  sensor->backProject(_edgeCoordsHom, idxEdge);

  int minIdx;
  int maxIdx;
  minmaxArray<int>(idxEdge, 4, &minIdx, &maxIdx);

  // Check whether non of the cells are in the field of view
  if(maxIdx<0) return false;

  if(minIdx<0) minIdx = 0;

  double minDist = distance - _circumradius;
  double maxDist = distance + _circumradius;

  // Check if any cell comes closer than the truncation radius
  bool isVisible = false;
  for(int j=minIdx; j<maxIdx; j++)
  {
    if(mask[j])
    {
      double sdf = data[j] - minDist;
      isVisible = isVisible || (sdf >= -maxTruncation);
    }
  }
  if(!isVisible) return false;

  bool isEmpty = true;
  for(int j=minIdx; j<maxIdx; j++)
  {
    double sdf = data[j] - maxDist;
    isEmpty = isEmpty && (sdf > maxTruncation) && mask[j];
  }

  if(isEmpty)
  {
    increaseEmptiness();
    return false;
  }

  return true;
}

}
