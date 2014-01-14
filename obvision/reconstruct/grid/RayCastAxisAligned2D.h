#ifndef RAYCASTAXISALIGNED2D_H_
#define RAYCASTAXISALIGNED2D_H_

#include "obvision/reconstruct/grid/TsdGrid.h"

namespace obvious {

class RayCastAxisAligned2D {
public:
  RayCastAxisAligned2D();
  virtual ~RayCastAxisAligned2D();
  void calcCoords(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt);
  void calcCoords(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt, char* occupiedGrid);
};

}

#endif /* RAYCASTAXISALIGNED2D_H_ */
