/*
 * AStarNode.h
 *
 *  Created on: 03.11.2014
 *      Author: mayst
 */

#ifndef ASTARNODE_H_
#define ASTARNODE_H_

namespace obvious
{

class AStarNode
{
public:

  /**
   * Constructor
   * @param xp
   * @param yp
   * @param d
   * @param p
   */
  AStarNode(int xp, int yp, int d, int p);

  /**
   * Destructor
   */
  virtual ~AStarNode();

  /**
   *
   * @return
   */
  int getxPos(void) const;

  /**
   *
   * @return
   */
  int getyPos(void) const;

  /**
   *
   * @return
   */
  int getLevel(void) const;

  /**
   *
   * @return
   */
  int getPriority(void) const;

  /**
   *
   * @param xDest
   * @param yDest
   */
  void updatePriority(const int & xDest, const int & yDest);

  /**
   * give better priority to going strait instead of diagonally
   * @param i direction
   */
  void nextLevel(const int & i);

  /**
   * Estimation function for the remaining distance to the goal.
   * @param xDest
   * @param yDest
   */
  const int estimate(const int & xDest, const int & yDest) const;

  int _dir;

private:

  /**
   * current position
   */
  int _xPos;

  /**
   *
   */
  int _yPos;

  /**
   * total distance already travelled to reach the node
   */
  int _level;

  /**
   * priority=level+remaining distance estimate
   * smaller: higher priority
   */
  int _priority;

};

} /* namespace obvious */
#endif /* ASTARNODE_H_ */
