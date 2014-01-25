#ifndef TSDGRIDCOMPONENT_H
#define TSDGRIDCOMPONENT_H

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

  double getComponentSize();

  double* getCentroid();

  double getCircumradius();

  Matrix* getEdgeCoordsHom();

  bool isLeaf();

  bool isInRange(double pos[2], Sensor* sensor, double maxTruncation);

  virtual void increaseEmptiness() = 0;

protected:

  double _componentSize;

  bool _isLeaf;

  double _centroid[2];

  Matrix* _edgeCoordsHom;

  double _circumradius;

};

}

#endif
