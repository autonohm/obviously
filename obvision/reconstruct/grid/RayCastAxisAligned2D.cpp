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
                coords[(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                coords[(*cnt)+1] = py*cellSize + (y * p->getHeight())* cellSize;
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
                coords[(*cnt)]   = px*cellSize + (x * p->getWidth()) * cellSize;
                coords[(*cnt)+1] = py*cellSize + cellSize * (interp-1.0) + (y * p->getHeight())* cellSize;
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

void RayCastAxisAligned2D::calcCoords(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt, char* occupiedGrid)
{
  unsigned int partitionsInX = grid->getCellsX() / grid->getPartitionSize();
  unsigned int partitionsInY = partitionsInX;
  unsigned int cellsX = grid->getCellsX();
  unsigned int cellsY = grid->getCellsY();
  double cellSize = grid->getCellSize();
  TsdGridPartition*** partitions = grid->getPartitions();
  unsigned int cellsPPart = partitions[0][0]->getHeight() * partitions[0][0]->getWidth();
  unsigned int cellsPPX = partitions[0][0]->getWidth();

  for(unsigned int y=0; y<partitionsInY; y++)
  {
    for(unsigned int x=0; x<partitionsInX; x++)
    {
      TsdGridPartition* p = partitions[y][x];
      if(p->isInitialized())
      {
        if(!(p->isEmpty()))
        {
          unsigned int gridOffset = y * cellsPPart * partitionsInX + x * cellsPPX;
          for(unsigned int py=0; py<p->getHeight(); py++)
          {
            double tsd_prev = (*p)(py, 0);
            double interp = 0.0;
            occupiedGrid[gridOffset + py * grid->getCellsX()] = ((tsd_prev > 0.0) ? 0 : -1);
            for(unsigned int px=1; px<p->getWidth(); px++)
            {
              double tsd = (*p)(py, px);
              occupiedGrid[gridOffset + py * grid->getCellsX() + px ] = ((tsd > 0.0) ? 0 : -1);
              // Check sign change
              if(tsd_prev * tsd < 0)
              {
                interp = tsd_prev / (tsd_prev - tsd);
                coords[(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                coords[(*cnt)+1] = py*cellSize + (y * p->getHeight())* cellSize;
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
                coords[(*cnt)]   = px*cellSize + (x * p->getWidth()) * cellSize;
                coords[(*cnt)+1] = py*cellSize + cellSize * (interp-1.0) + (y * p->getHeight())* cellSize;
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

