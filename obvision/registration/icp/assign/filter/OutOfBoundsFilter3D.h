#ifndef OUTOFBOUNDSFILTER3D_H
#define OUTOFBOUNDSFILTER3D_H

#include "obvision/registration/icp/assign/filter/IPreAssignmentFilter.h"
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class OutOfBoundsFilter3D
 * @brief Bounding box filtering
 * @author Stefan May
 */
class OutOfBoundsFilter3D : public IPreAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  OutOfBoundsFilter3D(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

  /**
   * Destructor
   */
  ~OutOfBoundsFilter3D();

  void setPose(Matrix* T);

  virtual void filter(double** scene, unsigned int size, bool* mask);

private:
  double _xMin;
  double _xMax;
  double _yMin;
  double _yMax;
  double _zMin;
  double _zMax;
  Matrix* _T;
};

}

#endif /*OUTOFBOUNDSFILTER3D_H*/
