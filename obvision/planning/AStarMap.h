#ifndef ASTARMAP_H_
#define ASTARMAP_H_

#include <string>
#include <list>

#include "obvision/planning/Obstacle.h"

namespace obvious
{

struct AStarCoord
{
  double x;
  double y;
};

class AStarMap
{

  friend class AStar;

public:

  /**
   * Constructor
   * @param cellSize edge length of cell (unit [m])
   * @param cellsX number of cells in horizontal direction
   * @param cellsY number of cells in vertical direction
   */
  AStarMap(double cellSize, unsigned int cellsX, unsigned int cellsY);

  /**
   * Copy constructor
   * @param map map to be copied
   */
  AStarMap(AStarMap &map);

  /**
   * Destructor
   */
  ~AStarMap();

  /**
   * Get width of map
   * @return width
   */
  unsigned int getWidth();

  /**
   * Get height of map
   * @return height
   */
  unsigned int getHeight();

  /**
   * Get size of cell
   * @return cell size in meters
   */
  double getCellSize();

  /**
   * Add obstacle to map
   * @param obstacle obstacle container with coordinates
   */
  void addObstacle(Obstacle obstacle);

  /**
   * Remove obstacle from map
   * @param obstacle obstacle container with coordinates
   */
  void removeObstacle(Obstacle obstacle);

  /**
   * Inflate map
   * @param robotRadius radius of robot, i.e., inflation radius of obstacles (unit [m])
   */
  void inflate(double robotRadius);

  void deinflate();

  /**
   * Convert map to raw image
   * @param buffer raw image as RGB triples
   */
  void convertToImage(unsigned char* buffer);

  void translateIndexToCoord(unsigned int xIdx, unsigned int yIdx, double* x, double* y);

  void translateCoordToIndex(double x, double y, unsigned int* xIdx, unsigned int* yIdx);

  /**
   * Translate a given path to map indices
   * @param path path in AStar format
   * @param xStart x-coordinate of starting position in meters
   * @param yStart y-coordinate of starting position in meters
   * @return path in map indices
   */
  std::vector<unsigned int> translatePathToMapIndices(std::vector<unsigned int> path, double xStart, double yStart);

  /**
   * Translate a given path to metric coordinates (unit [m])
   * @param path path in AStar format
   * @param xStart x-coordinate of starting position in meters
   * @param yStart y-coordinate of starting position in meters
   * @return path in metric coordinates
   */
  std::vector<AStarCoord> translatePathToCoords(std::vector<unsigned int> path, double xStart, double yStart);

  /**
   * Load map from file
   * @param path file path
   * @return map
   */
  static AStarMap* load(std::string path);

  /**
   * Serialize map to file
   * @param path file path
   */
  void serialize(std::string path);

  /**
   * Map factory method
   * @param data pointer to data buffer
   * @param cellSize size of cell (unit [m])
   * @param width width of map
   * @param height height of map
   * @return map
   */
  static AStarMap create(char* data, double cellSize, unsigned int width, unsigned int height);

private:

  char** _map;

  char** _mapWork;

  int** _closedNodesMap;

  int** _openNodesMap;

  int** _dirMap;

  double _cellSize;

  unsigned int _cellsX;

  unsigned int _cellsY;

};

} /* namespace obvious */
#endif /* ASTARMAP_H_ */
