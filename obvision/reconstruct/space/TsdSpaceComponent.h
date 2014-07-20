#ifndef TSDSPACECOMPONENT_H
#define TSDSPACECOMPONENT_H

#include "obvision/reconstruct/reconstruct_defs.h"
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class TsdSpaceComponent
 * @brief Abstract component for octree implementation
 * @author Stefan May
 */
class TsdSpaceComponent
{
public:
  TsdSpaceComponent(bool isLeaf);

  virtual ~TsdSpaceComponent();

  obfloat getComponentSize();

  obfloat* getCentroid();

  obfloat getCircumradius();

  Matrix* getEdgeCoordsHom();

  bool isLeaf();

  bool isInRange(obfloat pos[3], Sensor* sensor, obfloat maxTruncation);

  virtual void increaseEmptiness() = 0;

protected:

  obfloat _componentSize;

  bool _isLeaf;

  obfloat _centroid[3];

  Matrix* _edgeCoordsHom;

  obfloat _circumradius;

};

}

#endif
