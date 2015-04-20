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

bool TsdSpaceComponent::isInRange(obfloat pos[3], Sensor* sensor, obfloat maxTruncation)
{
  // Centroid-to-sensor distance
  obfloat distance = euklideanDistance<obfloat>(pos, _centroid, 3);

  // closest possible distance of any voxel in partition
  obfloat minDist = distance - _circumradius - maxTruncation;

  // check if partition is out of range
  if(minDist > sensor->getMaximumRange()) return false;

  // farthest possible distance of any voxel in partition
  obfloat maxDist = distance + _circumradius + maxTruncation;

  // check if partition is too close
  if(maxDist < sensor->getMinimumRange()) return false;

  if(_isLeaf)
  {
    double* data = sensor->getRealMeasurementData();
    bool* mask = sensor->getRealMeasurementMask();

    int width = sensor->getWidth();
    int height = sensor->getHeight();

    // Project back edges of partition
    int idxEdge[8];
    sensor->backProject(_edgeCoordsHom, idxEdge);

    // Determine outmost projection range
    int x_min = width-1;
    int x_max = -1;
    int y_min = height-1;
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
        }
      }
    }

    if(!isVisible) return false;

    // TODO: verify the following if clause
    //if(validIndices==8)
    {
      bool isEmpty = true;
      for(int y=y_min; y<=y_max; y++)
      {
        for(int x=x_min; x<=x_max; x++)
        {
          int idx = y*width+x;
          isEmpty = isEmpty && (data[idx] > maxDist) && mask[idx];
        }
      }

      if(isEmpty)
      {
        increaseEmptiness();
        return false;
      }
    }
  }
  return true;
}

}
