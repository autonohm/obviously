#include "AStarNode.h"

#include "obvision/planning/AStarNode.h"
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

namespace obvious
{


AStarNode::AStarNode(int xp, int yp, int d, int p, int currentDir)
{
  _xPos       = xp;
  _yPos       = yp;
  _level      = d;
  _priority   = p;
  _currentDir = currentDir;
}

AStarNode::~AStarNode()
{

}

void AStarNode::setCurrentDir(int dir)
{
  _currentDir = dir;
}

int AStarNode::getCurrentDir() const
{
  return _currentDir;
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
  // factor 10 is used to represent distance with two digits precision
  _priority=_level+estimate(xDest, yDest)*10;
}

void AStarNode::nextLevelPenalty(const int & i)
{
  // give better priority to going strait instead of diagonally
  //_level+=(i%2==0?10:14);

  // factor 10 is used to represent distance with two digits precision
  // 14 represents diagonal
  // a penalty is added when the direction needs to be changed
  _level+=((i%2==0?10:14) + (( ((i+4)%8)==_currentDir) ? 0 : 1));
}

void AStarNode::nextLevel(const int & i)
{
  // give better priority to going strait instead of diagonally
  //_level+=(i%2==0?10:14);

  // factor 10 is used to represent distance with two digits precision
  // 14 represents diagonal
  // a penalty is added when the direction needs to be changed
  _level+=(i%2==0?10:14);
}

const int AStarNode::estimate(const int & xDest, const int & yDest) const
{
  int xd = xDest-_xPos;
  int yd = yDest-_yPos;

  // Euclidian Distance
  return(sqrt(xd*xd+yd*yd));
}

} /* namespace obvious */
