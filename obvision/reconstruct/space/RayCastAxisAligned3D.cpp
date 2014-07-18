#include "RayCastAxisAligned3D.h"
#include "obcore/base/Logger.h"

namespace obvious {

RayCastAxisAligned3D::RayCastAxisAligned3D() {

}

RayCastAxisAligned3D::~RayCastAxisAligned3D() {

}

void RayCastAxisAligned3D::calcCoords(TsdSpace* space, double* coords, double* normals, unsigned char* rgb, unsigned int* cnt)
{
  Timer t;
  t.start();

  unsigned int partitionsInX = space->getXDimension() / space->getPartitionSize();
  unsigned int partitionsInY = space->getYDimension() / space->getPartitionSize();
  unsigned int partitionsInZ = space->getZDimension() / space->getPartitionSize();
  double cellSize = space->getVoxelSize();

  *cnt = 0;

  TsdSpacePartition**** partitions = space->getPartitions();

#pragma omp parallel for
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
                for(unsigned int px=1; px<p->getWidth()+1; px++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
#pragma omp critical
{
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize + cellSize * (interp-1.0) ;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize;
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize;
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    if(rgb)
                      space->interpolateTrilinearRGB(&coords[*cnt], &(rgb[*cnt]));
                    (*cnt)+=3;
}
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
                for(unsigned int py=1; py<p->getHeight()+1; py++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
#pragma omp critical
{
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize + cellSize * (interp-1.0);
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize;
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    if(rgb)
                      space->interpolateTrilinearRGB(&coords[*cnt], &(rgb[*cnt]));
                    (*cnt)+=3;
}
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
                for(unsigned int pz=1; pz<p->getDepth()+1; pz++)
                {
                  double tsd = (*p)(pz, py, px);
                  // Check sign change
                  if(tsd_prev * tsd < 0)
                  {
                    interp = tsd_prev / (tsd_prev - tsd);
#pragma omp critical
{
                    coords[*cnt]     = px*cellSize + (x * p->getWidth()) * cellSize;
                    coords[(*cnt)+1] = py*cellSize + (y * p->getHeight()) * cellSize;
                    coords[(*cnt)+2] = pz*cellSize + (z * p->getDepth()) * cellSize + cellSize * (interp-1.0);
                    if(normals)
                      space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                    if(rgb)
                      space->interpolateTrilinearRGB(&coords[*cnt], &(rgb[*cnt]));
                    (*cnt)+=3;
}
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
  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.elapsed() << "s");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *cnt << " coordinates");
}

void RayCastAxisAligned3D::calcCoordsRoughly(TsdSpace* space, double* coords, double* normals, unsigned int* cnt)
{
  Timer t;
  t.start();

  unsigned int partitionsInX = space->getXDimension() / space->getPartitionSize();
  unsigned int partitionsInY = space->getYDimension() / space->getPartitionSize();
  unsigned int partitionsInZ = space->getZDimension() / space->getPartitionSize();
  double cellSize = space->getVoxelSize();

  *cnt = 0;

  TsdSpacePartition**** partitions = space->getPartitions();
  Matrix* C = TsdSpacePartition::getCellCoordsHom();

  double thresh = cellSize / space->getMaxTruncation();

#pragma omp parallel for
  for(unsigned int z=1; z<partitionsInZ-1; z++)
  {
    for(unsigned int y=1; y<partitionsInY-1; y++)
    {
      for(unsigned int x=1; x<partitionsInX-1; x++)
      {
        TsdSpacePartition* p = partitions[z][y][x];
        if(p->isInitialized())
        {
          double offset[3];
          p->getCellCoordsOffset(offset);

          unsigned int i = 0;
          for(unsigned int pz=0; pz<p->getDepth(); pz++)
          {
            for(unsigned int py=0; py<p->getHeight(); py++)
            {
              for(unsigned int px=0; px<p->getWidth(); px++, i++)
              {
                double tsd = (*p)(pz, py, px);
                // Check sign change
                if(tsd < thresh && tsd>0)
                {
#pragma omp critical
{
                  coords[*cnt]     = (*C)(i, 0) + offset[0];
                  coords[(*cnt)+1] = (*C)(i, 1) + offset[1];
                  coords[(*cnt)+2] = (*C)(i, 2) + offset[2];
                  if(normals)
                    space->interpolateNormal(&coords[*cnt], &(normals[*cnt]));
                  (*cnt)+=3;
}
                }
              }
            }
          }
        }
      }
    }
  }
  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.elapsed() << "s");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *cnt << " coordinates");
}

}
