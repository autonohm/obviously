#include "RayCastAxisAligned3D.h"

namespace obvious {

RayCastAxisAligned3D::RayCastAxisAligned3D() {

}

RayCastAxisAligned3D::~RayCastAxisAligned3D() {

}

void RayCastAxisAligned3D::calcCoords(TsdSpace* space, double* coords, double* normals, unsigned int* cnt)
{
  unsigned int partitionsInX = space->getXDimension() / space->getPartitionSize();
  unsigned int partitionsInY = space->getYDimension() / space->getPartitionSize();
  unsigned int partitionsInZ = space->getZDimension() / space->getPartitionSize();
  double cellSize = space->getVoxelSize();

  *cnt = 0;

  TsdSpacePartition**** partitions = space->getPartitions();

  for(unsigned int z=0; z<partitionsInZ; z++)
  {
    for(unsigned int y=0; y<partitionsInY; y++)
    {
      for(unsigned int x=0; x<partitionsInX; x++)
      {
        TsdSpacePartition* p = partitions[z][y][x];
        if(p->isInitialized())
        {
          if(!(p->isEmpty()))
          {
            for(unsigned int pz=0; pz<p->getDepth(); pz++)
            {
              for(unsigned int py=0; py<p->getHeight(); py++)
              {
                double tsd_prev = (*p)(pz, py, 0);
                double interp = 0.0;
                for(unsigned int px=1; px<p->getWidth(); px++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize + cellSize * (interp-1.0) ;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize;
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize;
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    (*cnt)+=3;
                  }
                  tsd_prev = tsd;
                }
              }
            }

            for(unsigned int pz=0; pz<p->getDepth(); pz++)
            {
              for(unsigned int px=0; px<p->getWidth(); px++)
              {
                double tsd_prev = (*p)(pz, 0, px);
                double interp = 0.0;
                for(unsigned int py=1; py<p->getHeight(); py++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize + cellSize * (interp-1.0);
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize;
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    (*cnt)+=3;
                  }
                  tsd_prev = tsd;
                }
              }
            }

            for(unsigned int px=0; px<p->getWidth(); px++)
            {
              for(unsigned int py=0; py<p->getHeight(); py++)
              {
                double tsd_prev = (*p)(0, py, px);
                double interp = 0.0;
                for(unsigned int pz=1; pz<p->getDepth(); pz++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize;
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize + cellSize * (interp-1.0);
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    (*cnt)+=3;
                  }
                  tsd_prev = tsd;
                }
              }
            }

          }
        }
      }
    }
  }
}

}
