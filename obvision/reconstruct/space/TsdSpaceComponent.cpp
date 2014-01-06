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

  if(!isnan(range))
  {
    if((distance-_circumradius) > range) return false;
  }

  int idxEdge[8];
  sensor->backProject(_edgeCoordsHom, idxEdge);

  double minDist = distance - _circumradius;
  double maxDist = distance + _circumradius;

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
  }
  if(!isVisible) return false;

  if(isEmpty)
  {
    increaseEmptiness();
    return false;
  }

  return true;
}

}
