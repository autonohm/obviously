/*
 * StatePing.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */
#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/StateMachine.h"
#include "StatePing.h"
#include "StatePong.h"

namespace obvious
{

StatePing::StatePing(void)
{

}

StatePing::~StatePing(void)
{

}

void StatePing::process(void)
{
  std::cout << "Ping" << std::endl;
  if(rand()%100<30)
  {
    obvious::StateMachine::getInstance()->setState(new StatePong());
    delete this;
  }
}

} /* namespace obvious */
