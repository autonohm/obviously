/**
* @file NormalFilter.cpp
* @autor christian
* @date  04.12.2012
*
*
*/

#include "obcore/filter/NormalFilter.h"

using namespace obvious;

/*
 * Function to start filtering
 */
FILRETVAL NormalFilter::applyFilter(void)
{
  if((!_input)||(!_output)||(!_inputNormals)||(!_outputNormals))
  {
    LOGMSG(DBG_ERROR, "Pointer to input, output or normals invalid");
    return(FILTER_ERROR);
  }
  // check if custom axis is set
  if (_customAxis == CustomAxis(0,0,0))
  {
    if (_axis == x)
      _customAxis = CustomAxis(1,0,0);
    else if (_axis == y)
      _customAxis = CustomAxis(0,1,0);
    else if (_axis == z)
      _customAxis = CustomAxis(0,0,1);
  }

  _validSize      = 0;
  double *dPtr    = _input;
  double *dPtrN   = _inputNormals;

  // init output arrays
  for(unsigned int i=0 ; i<_size ; i++)
  {
    _output[i]        = 0.0;
    _outputNormals[i] = 0.0;
  }

  if (_direction == FILTER_BIGGER)
  {
    for(unsigned int i=0 ; i<_size ; i+=3) {
      double normal[3] = {_inputNormals[i], _inputNormals[i+1], _inputNormals[i+2]};
      double axis[3]   = {_customAxis[0],   _customAxis[1],     _customAxis[2]};
      double angle     = getAngleBetweenVec<double>(normal, axis);

      if(angle > _threshold)
      {
        dPtr       += 3;
        dPtrN      += 3;
      }
      else
      {
        for(unsigned int j=0 ; j<3 ; j++)
        {
          *_output++        = *dPtr++;
          *_outputNormals++ = *dPtr++;
        }
        _validSize += 3;
      }
    }
  }
  else // FILTER_SMALLER
  {
    for(unsigned int i=0 ; i<_size ; i+=3) {
      double normal[3] = {_inputNormals[i], _inputNormals[i+1], _inputNormals[i+2]};
      double axis[3]   = {_customAxis[0],   _customAxis[1],     _customAxis[2]};
      double angle     = getAngleBetweenVec<double>(normal, axis);

      if(angle <= _threshold)
      {
        dPtr       += 3;
        dPtrN      += 3;
      }
      else
      {
        for(unsigned int j=0 ; j<3 ; j++)
        {
          *_output++        = *dPtr++;
          *_outputNormals++ = *dPtr++;
        }
        _validSize += 3;
      }
    }
  }
  return(FILTER_OK);
}






