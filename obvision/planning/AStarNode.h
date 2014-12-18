/*
 * AStarNode.h
 *
 *  Created on: 03.11.2014
 *      Author: Jon Martin and Stefan May
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
  AStarNode(int xp, int yp, int d, int p, int currentDir);

  /**
   * Destructor
   */
  virtual ~AStarNode();

  /**
   * Set current direction
   * @param dir direction
   */
  void setCurrentDir(int dir);

  /**
   * Get current direction
   * @return current direction
   */
  int getCurrentDir() const;

  /**
   *
   * @return
   */
  int getxPos() const;

  /**
   *
   * @return
   */
  int getyPos() const;

  /**
   *
   * @return
   */
  int getLevel() const;

  /**
   *
   * @return
   */
  int getPriority() const;

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
   * Calculate next level with directional penalty
   * @param i direction
   */
  void nextLevelPenalty(const int & i);

  /**
   * Estimation function for the remaining distance to the goal.
   * @param xDest
   * @param yDest
   */
  const int estimate(const int & xDest, const int & yDest) const;

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
   * priority=level+remaining distance estimate+heuristic
   * smaller: higher priority
   */
  int _priority;

  /**
   * current direction
   */
  int _currentDir;
};

} /* namespace obvious */
#endif /* ASTARNODE_H_ */
