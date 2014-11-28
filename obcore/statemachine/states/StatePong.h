/*
 * StatePong.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEPONG_H_
#define STATEPONG_H_

#include "obcore/statemachine/IState.h"

namespace obvious
{

class StatePong: public IState
{
public:
  StatePong(void);

  virtual ~StatePong(void);

  void process(void);

private:

};

} /* namespace obvious */

#endif /* STATEPONG_H_ */
