/*
 * StatePong.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Context.h"
#include "StatePong.h"
#include "StatePing.h"

namespace obvious
{

StatePong::StatePong(void)
{

}

StatePong::~StatePong(void)
{

}

void StatePong::process(void)
{
  std::cout << "Pong" << std::endl;
  if(rand()%100<30)
  {
    obvious::Context::getInstance()->setState(new StatePing());
    delete this;
  }
}

} /* namespace obvious */
