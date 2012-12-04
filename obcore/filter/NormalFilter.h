/**
* @file   NormalFilter.h
* @author Christian Pfitzner
* @date   04.12.2012
*
*
*/

#ifndef NORMALFILTER_H_
#define NORMALFILTER_H_

#include "obcore/Point3D.h"
#include "obcore/math/mathbase.h"
#include "obcore/filter/FilterBase.h"

/**
 * @namespace of obviously library
 */
namespace obvious
{

class Filter;

/**
 * @typedef Point3D to CustomAxis
 * Axis similar to Point
 */
typedef Point3D CustomAxis;

/**
 * @class NormalFilter  Class to filter points by its normals out of a
 * given dataset of 3d points in a double array to a specified angle and axis.
 */
class NormalFilter : public FilterBase
{
public:
  /**
   * Default constructor with initialization
   */
  NormalFilter(void)
    : FilterBase(),
      _outputNormals(NULL),
      _inputNormals(NULL),
      _axis(x),
      _customAxis(0, 0, 0) {  }
  /**
   * Default destructor
   */
  virtual ~NormalFilter(void)
  {
    delete [] _inputNormals;
    delete [] _outputNormals;
  }
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
  FILRETVAL setInput(double *addr, double* addrNormals, const unsigned int inputSize)
  {
    _input        = addr;
    _inputNormals = addrNormals;
    _size         = inputSize;
    return(FILTER_OK);
  }
  /**
   * Function to start filtering
   *
   * @note This function must be called after setting input and output of
   * base class.
   */
  FILRETVAL applyFilter(void);
  /**
   * @enum  axis  Enum for switching to xyz axis
   */
  enum Axis{
    x,    /// z axis
    y,    /// y axis
    z     /// z axis
  };
  /**
   * Function to set axis for cartesian filtering (cartesian axis)
   * @param   ax    axis to filter @see axis
   */
  void setAxis(const Axis ax)         { _axis          = ax; }
  /**
   * Function to set axis for filtering
   * @param ax
   */
  void setAxis(const CustomAxis ax)   { _customAxis    = ax; }
  /**
   * Function to set input of normals
   * @param   addr    Address to data of normals
   */
  void setIntputNormals(double* addr) { _inputNormals  = addr; }
  /**
   * Function to set output address of normals
   * @param   addr    Address to output of normals
   */
  void setOutputNormals(double *addr) { _outputNormals = addr; }
private:
  double*       _outputNormals;           ///< output of normals
  double*       _inputNormals;            ///< intput of normals
  Axis          _axis;                    ///< axis to filter
  CustomAxis    _customAxis;              ///< user specified axis

};

};  //namespace



#endif /* NORMALFILTER_H_ */
