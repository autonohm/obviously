#ifndef TSDGRIDCOMPONENT_H
#define TSDGRIDCOMPONENT_H

#include "obcore/base/types.h"
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/reconstruct_defs.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class TsdGridComponent
 * @brief Abstract component for quadtree implementation
 * @author Stefan May
 */
class TsdGridComponent
{
public:
  TsdGridComponent(bool isLeaf);

  virtual ~TsdGridComponent();

  obfloat getComponentSize();

  obfloat* getCentroid();

  obfloat getCircumradius();

  Matrix* getEdgeCoordsHom();

  bool isLeaf();

  bool isInRange(obfloat pos[2], Sensor* sensor, obfloat maxTruncation);

  virtual void increaseEmptiness() = 0;

protected:

  obfloat _componentSize;

  bool _isLeaf;

  obfloat _centroid[2];

  Matrix* _edgeCoordsHom;

  obfloat _circumradius;

};

}

#endif
