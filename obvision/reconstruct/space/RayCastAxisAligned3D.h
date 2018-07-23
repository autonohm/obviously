#ifndef RAYCASTAXISALIGNED3D_H_
#define RAYCASTAXISALIGNED3D_H_

#include "obvision/reconstruct/space/TsdSpace.h"

namespace obvious {

class RayCastAxisAligned3D {
public:
  RayCastAxisAligned3D();

  virtual ~RayCastAxisAligned3D();

  void calcCoords(TsdSpace* space, obfloat* coords, obfloat* normals, unsigned char* rgb, unsigned int* cnt);

  void calcCoordsRoughly(TsdSpace* space, obfloat* coords, obfloat* normals, unsigned int* cnt);
};

}

#endif
