#ifndef ASTARMAP_H_
#define ASTARMAP_H_

#include <string>
#include <list>

#include "obvision/planning/Obstacle.h"
#include "obcore/base/Point.h"

namespace obvious
{

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
  AStarMap(obfloat cellSize, unsigned int cellsX, unsigned int cellsY);

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
  obfloat getCellSize();

  /**
   * Add obstacle to map
   * @param obstacle obstacle container with coordinates
   */
  void addObstacle(Obstacle &obstacle);

  /**
   * Remove obstacle from map
   * @param obstacle obstacle container with coordinates
   */
  void removeObstacle(Obstacle* obstacle);

  /**
   * Remove all previously added obstacles
   */
  void removeAllObstacles();

  /**
   * Check whether similar obstacle is already added to the map
   * @param obstacle query obstacle
   * @return similarity obstacle
   */
  Obstacle* checkObstacleIntersection(Obstacle obstacle);

  /**
   * Inflate map
   * @param robotRadius radius of robot, i.e., inflation radius of obstacles (unit [m])
   */
  void inflate(obfloat robotRadius);

  /**
   * Get map with obstacles as occupied cells
   * @return map
   */
  char** getMapWithObstacles();

  /**
   * Convert map to raw image
   * @param buffer raw image as RGB triples
   */
  void convertToImage(unsigned char* buffer);

  /**
   * translate indices to coordinates
   * @param pixel image index (col, row)
   * @param coords metric coordinates
   */
  void translatePixelToCoord(Pixel pixel, Point2D* coords);

  /**
   * translate coordinates to indices
   * @param coords metric coordinates
   * @param pixel image index (col, row)
   */
  void translateCoordToPixel(Point2D coords, Pixel* pixel);

  /**
   * Translate a given path to map indices
   * @param path path in AStar format
   * @param coordStart coordinates of starting position in meters
   * @return path in map indices
   */
  std::vector<unsigned int> translatePathToMapIndices(std::vector<unsigned int> path, Point2D coordStart);

  /**
   * Translate a given path to metric coordinates (unit [m])
   * @param path path in AStar format
   * @param coordStart coordinates of starting position in meters
   * @return path in metric coordinates
   */
  std::vector<Point2D> translatePathToCoords(std::vector<unsigned int> path, Point2D coordStart);

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
  static AStarMap* create(const char* data, obfloat cellSize, unsigned int width, unsigned int height);

private:

  char** _map;

  char** _mapObstacle;

  char** _mapWork;

  bool _mapIsDirty;

  int** _closedNodesMap;

  int** _openNodesMap;

  int** _dirMap;

  obfloat _cellSize;

  unsigned int _cellsX;

  unsigned int _cellsY;

  std::list<Obstacle> _obstacles;
};

} /* namespace obvious */
#endif /* ASTARMAP_H_ */
