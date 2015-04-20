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

  obfloat getComponentSize() const { return _componentSize; }

  obfloat* getCentroid() { return _centroid; }

  obfloat getCircumradius() const { return _circumradius; };

  Matrix* getEdgeCoordsHom() const { return _edgeCoordsHom; }

  bool isLeaf() const { return _isLeaf; }

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
