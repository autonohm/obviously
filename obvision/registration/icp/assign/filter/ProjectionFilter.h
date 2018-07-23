#ifndef PROJECTIONFILTER_H
#define PROJECTIONFILTER_H

#include "obcore/base/CartesianCloud.h"
#include "obvision/registration/icp/assign/filter/IPreAssignmentFilter.h"
#include <vector>

using namespace obvious;

namespace obvious
{

/**
 * @class ProjectionFilter
 * @brief
 * @author Stefan May
 */
class ProjectionFilter : public IPreAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  ProjectionFilter(double* P, unsigned int width, unsigned int height);

  /**
   * Destructor
   */
  ~ProjectionFilter();

  void setModel(CartesianCloud3D* cloud);
  void update(CartesianCloud3D* cloud, double* P);
  void update(double* P, unsigned char* mask, double* zbuffer);
  void enableFaultRemovement();
  void disableFaultRemovement();

  virtual void filter(double** scene, unsigned int size, bool* mask);

private:
  double _P[12];
  unsigned char** _mask;
  double** _zbuffer;
  unsigned int _w;
  unsigned int _h;
  double _zFar;
  bool _faultRemovement;
};

}

#endif /*PROJECTIONFILTER_H*/
