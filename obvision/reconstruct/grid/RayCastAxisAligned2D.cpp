#include "RayCastAxisAligned2D.h"

namespace obvious {

RayCastAxisAligned2D::RayCastAxisAligned2D() {

}

RayCastAxisAligned2D::~RayCastAxisAligned2D() {

}

void RayCastAxisAligned2D::calcCoords(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt)
{
  unsigned int partitionsInX = grid->getCellsX() / grid->getPartitionSize();
  unsigned int partitionsInY = partitionsInX;
  double cellSize = grid->getCellSize();
  TsdGridPartition*** partitions = grid->getPartitions();

  for(unsigned int y=0; y<partitionsInY; y++)
  {
    for(unsigned int x=0; x<partitionsInX; x++)
    {
      TsdGridPartition* p = partitions[y][x];
      if(p->isInitialized())
      {
        if(!(p->isEmpty()))
        {
          for(unsigned int py=0; py<p->getHeight(); py++)
          {
            double tsd_prev = (*p)(py, 0);
            double interp = 0.0;
            bool found = false;
            for(unsigned int px=1; px<p->getWidth(); px++)
            {
              double tsd = (*p)(py, px);
              // Check sign change
              if(tsd_prev > 0 && tsd < 0)
              {
                interp = tsd_prev / (tsd_prev - tsd);
                coords[2*(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                coords[2*(*cnt)+1] = py*cellSize + (y * p->getHeight())* cellSize;
                if(normals)
                  grid->interpolateNormal(coords, &(normals[*cnt]));
                (*cnt) += 2;
              }
              tsd_prev = tsd;
            }
          }
          for(unsigned int px = 1; px  < p->getWidth(); px++)
          {
            double tsd_prev = (*p)(0, px);
            double interp = 0.0;
            for(unsigned int py = 0; py < p->getHeight(); py++)
            {
              double tsd = (*p)(py, px);
              if(tsd_prev > 0 && tsd < 0)
              {
                interp = tsd_prev / (tsd_prev - tsd);
                coords[2*(*cnt)]   = px*cellSize + (x * p->getWidth()) * cellSize;
                coords[2*(*cnt)+1] = py*cellSize + cellSize * (interp-1.0) + (y * p->getHeight())* cellSize;
                if(normals)
                  grid->interpolateNormal(coords, &(normals[*cnt]));
                (*cnt) += 2;
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
