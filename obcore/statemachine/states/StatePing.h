/*
 * StatePing.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEPING_H_
#define STATEPING_H_

#include "obcore/statemachine/IState.h"

namespace obvious {


class StatePing: public IState
{
public:

   StatePing(void);

   virtual ~StatePing(void);

   void process(void);

private:

};

} /* namespace obvious */

#endif /* STATEPING_H_ */
