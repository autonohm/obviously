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

  double getComponentSize();

  double* getCentroid();

  double getCircumradius();

  Matrix* getEdgeCoordsHom();

  bool isLeaf();

  bool isInRange(double pos[3], Sensor* sensor, double maxTruncation);

  virtual void increaseEmptiness() = 0;

protected:

  double _componentSize;

  bool _isLeaf;

  double _centroid[3];

  Matrix* _edgeCoordsHom;

  double _circumradius;

};

}

#endif
