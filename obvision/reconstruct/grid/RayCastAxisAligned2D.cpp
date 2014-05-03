#include "RayCastAxisAligned2D.h"

namespace obvious {

RayCastAxisAligned2D::RayCastAxisAligned2D() {

}

RayCastAxisAligned2D::~RayCastAxisAligned2D() {

}

void RayCastAxisAligned2D::calcCoords(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt, char* occupiedGrid)
{
  unsigned int partitionsInX = grid->getCellsX() / grid->getPartitionSize();
  unsigned int partitionsInY = partitionsInX;
  double cellSize = grid->getCellSize();
  TsdGridPartition*** partitions = grid->getPartitions();
  unsigned int cellsPPart = partitions[0][0]->getHeight() * partitions[0][0]->getWidth();
  unsigned int cellsPPX = partitions[0][0]->getWidth();
  unsigned int gridOffset = 0;

  *cnt = 0;

  for(unsigned int y=1; y<partitionsInY-1; y++)
  {
    for(unsigned int x=1; x<partitionsInX-1; x++)
    {
      TsdGridPartition* p = partitions[y][x];
      if(p->isInitialized())
      {
        if(!(p->isEmpty()))
        {
          if(occupiedGrid)
            gridOffset = y * cellsPPart * partitionsInX + x * cellsPPX;
          for(unsigned int py=0; py<p->getHeight(); py++)
          {
            double tsd_prev = (*p)(py, 0);
            double interp = 0.0;
            if(occupiedGrid)
              occupiedGrid[gridOffset + py * grid->getCellsX()] = ((tsd_prev > 0.0) ? 0 : -1);
            for(unsigned int px=1; px<p->getWidth(); px++)
            {
              double tsd = (*p)(py, px);
              if(occupiedGrid)
                occupiedGrid[gridOffset + py * grid->getCellsX() + px ] = ((tsd > 0.0) ? 0 : -1);
              // Check sign change
              if((tsd_prev > 0 && tsd < 0) || (tsd_prev < 0 && tsd > 0))
              {
                interp = tsd_prev / (tsd_prev - tsd);
                //                coords[(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                //                coords[(*cnt)+1] = py*cellSize + (y * p->getHeight())* cellSize;
                double cordsTmpX = coords[(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                double cordsTmpY = py*cellSize + (y * p->getHeight())* cellSize;
                double smCellSize = 0.3 * cellSize;
                coords[(*cnt)++]   = cordsTmpX;
                coords[(*cnt)++] = cordsTmpY;
                coords[(*cnt)++] = cordsTmpX + smCellSize;
                coords[(*cnt)++] = cordsTmpY + smCellSize;
                coords[(*cnt)++] = cordsTmpX - smCellSize;
                coords[(*cnt)++] = cordsTmpY - smCellSize;
                coords[(*cnt)++] = cordsTmpX + smCellSize;
                coords[(*cnt)++] = cordsTmpY - smCellSize;
                coords[(*cnt)++] = cordsTmpX - smCellSize;
                coords[(*cnt)++] = cordsTmpY + smCellSize;
                if(normals)
                  grid->interpolateNormal(coords, &(normals[*cnt]));
                //(*cnt) += 2;
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
              if((tsd_prev > 0 && tsd < 0) || (tsd_prev < 0 && tsd > 0))
              {
                interp = tsd_prev / (tsd_prev - tsd);
                //                coords[(*cnt)]   = px*cellSize + (x * p->getWidth()) * cellSize;
                //                coords[(*cnt)+1] = py*cellSize + cellSize * (interp-1.0) + (y * p->getHeight())* cellSize;
                double cordsTmpX = coords[(*cnt)]   = px*cellSize + cellSize * (interp-1.0) + (x * p->getWidth()) * cellSize;
                double cordsTmpY = py*cellSize + (y * p->getHeight())* cellSize;
                double smCellSize = 0.3 * cellSize;
                coords[(*cnt)++]   = cordsTmpX;
                coords[(*cnt)++] = cordsTmpY;
                coords[(*cnt)++] = cordsTmpX + smCellSize;
                coords[(*cnt)++] = cordsTmpY + smCellSize;
                coords[(*cnt)++] = cordsTmpX - smCellSize;
                coords[(*cnt)++] = cordsTmpY - smCellSize;
                coords[(*cnt)++] = cordsTmpX + smCellSize;
                coords[(*cnt)++] = cordsTmpY - smCellSize;
                coords[(*cnt)++] = cordsTmpX - smCellSize;
                coords[(*cnt)++] = cordsTmpY + smCellSize;
                if(normals)
                  grid->interpolateNormal(coords, &(normals[*cnt]));
                //(*cnt) += 2;
              }
              tsd_prev = tsd;
            }
          }
        }
      }
      else   //partition is not initialized
      {
        if(p->isEmpty())  //partition is empty
        {
//          std::cout << __PRETTY_FUNCTION__ << " isempty!\n";
          if(occupiedGrid)
            gridOffset = y * cellsPPart * partitionsInX + x * cellsPPX;
          for(unsigned int py=0; py<p->getHeight(); py++)
          {
            if(occupiedGrid)
              occupiedGrid[gridOffset + py * grid->getCellsX()] = 0;
            for(unsigned int px=1; px<p->getWidth(); px++)
              if(occupiedGrid)
                occupiedGrid[gridOffset + py * grid->getCellsX() + px ] = 0;
          }
        }
      }
    }
  }
}

}

