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
  // Centroid-to-sensor distance
  double distance = euklideanDistance<double>(pos, _centroid, 3);


  double maxRange = sensor->getMaximumRange();

  // closest possible distance of any voxel in partition
  double minDist = distance - _circumradius - maxTruncation;

  // check if partition is out of range
  if(minDist > maxRange) return false;


  double minRange = sensor->getMinimumRange();

  // farthest possible distance of any voxel in partition
  double maxDist = distance + _circumradius + maxTruncation;

  // check if partition is too close
  if(maxDist < minRange) return false;


  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();

  int width = sensor->getWidth();
  int height = sensor->getHeight();

  // Project back edges of partition
  int idxEdge[8];
  sensor->backProject(_edgeCoordsHom, idxEdge);


  // Determine outmost projection range
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
  }

  // Verify that at least one edge is in the field of view
  if(validIndices==0) return false;

  // We might oversee some voxels, if validIndices < 8, but this should be negligible
  // Verify whether any measurement within the projection range is close enough for pushing data
  bool isVisible = false;
  bool isEmpty = true;
  if(x_min<0) x_min = 0;
  if(y_min<0) y_min = 0;
  for(int y=y_min; y<=y_max; y++)
  {
    for(int x=x_min; x<=x_max; x++)
    {
      int idx = y*width+x;

      if(mask[idx])
      {
        isVisible = isVisible || (data[idx] > minDist);
        if(isVisible)
        {
          isEmpty = isEmpty && (data[idx] > maxDist);
        }
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
