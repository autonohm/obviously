#ifndef ASSIGNBASE_H
#define ASSIGNBASE_H

#include <stdlib.h>
#include <string.h>
#include <iostream>

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @struct TdCartesianPoint
 * @brief Represents a single point in Cartesian space
 * @author Stefan May and Dirk Holz
 **/
typedef double* TdCartesianPoint;

/**
 * @struct StrCartesianPair
 * @brief Represents a point pair in Cartesian space
 * @author Stefan May and Dirk Holz
 **/
struct StrCartesianPair
{
  /** first point's coordinates */
  TdCartesianPoint first;
  /** second point's coordinates */
  TdCartesianPoint second;
};

/**
 * @struct StrCartesianIndexPair
 * @brief Representation of one pair of point indices
 * @author Stefan May
 */
struct StrCartesianIndexPair
{
  /** index of first point */
  unsigned int indexFirst;
  /** index of second point */
  unsigned int indexSecond;
};
}

#endif /* ASSIGNBASE_H */
