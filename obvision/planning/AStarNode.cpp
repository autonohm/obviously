#include "AStarNode.h"

#include "obvision/planning/AStarNode.h"
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

namespace obvious
{


AStarNode::AStarNode(int xp, int yp, int d, int p)
{
  _dir = 8;
  _xPos=xp;
  _yPos=yp;
  _level=d;
  _priority=p;
}

AStarNode::~AStarNode()
{

}

int AStarNode::getxPos() const
{
  return _xPos;
}

int AStarNode::getyPos() const
{
  return _yPos;
}

int AStarNode::getLevel() const
{
  return _level;
}

int AStarNode::getPriority() const
{
  return _priority;
}

void AStarNode::updatePriority(const int & xDest, const int & yDest)
{
  _priority=_level+estimate(xDest, yDest)*10; //A*
}

void AStarNode::nextLevel(const int & i)
{
  // give better priority to going strait instead of diagonally
  _level+=(_dir==8?(i%2==0?10:14):10);
}

const int AStarNode::estimate(const int & xDest, const int & yDest) const
{
  int xd = xDest-_xPos;
  int yd = yDest-_yPos;

  // Euclidian Distance
  return(sqrt(xd*xd+yd*yd));
}

} /* namespace obvious */
