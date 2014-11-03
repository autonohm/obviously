/*
 * AStar.h
 *
 *  Created on: 03.11.2014
 *      Author: mayst
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <string>
#include <list>

#include "obvision/planning/Obstacle.h"


namespace obvious
{

class AStar
{

public:

  AStar(int width, int height, int resolution, bool obstacleInflation, int inflationFactor, double robotRadius);

  ~AStar();

  void reset();

  void load(std::string path);

  void create(char* data, int width, int height, int offsetX, int offsetY, int resolution);

  void serialize(char* path);

  void addObstacle(Obstacle obstacle);

  void removeObstacle(Obstacle obstacle);

  std::list<int> pathFind(const int & xStart, const int & yStart, const int & xFinish, const int & yFinish);

private:

  int _width;  // horizontal size of the map
  int _height; // vertical size size of the map
  int _resolution;
  int _dir;    // number of possible directions to go at any position
  int _dx[8];
  int _dy[8];

  int** _map;
  int** _map_tmp;     //toDo: find better name (phil)
  int** _closed_nodes_map;
  int** _open_nodes_map;
  int** _dir_map;
  int** _originalMap;        ///< will be initialized one time and stays static  //toDo: unsigned int

  bool _initial;
  double _gridOffsetX;
  double _gridOffsetY;
  bool _obstacleInflation;
  int _inflationFactor;
  double _robotRadius;
};

} /* namespace obvious */
#endif /* ASTAR_H_ */
