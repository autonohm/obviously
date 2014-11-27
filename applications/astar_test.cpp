#include "obvision/planning/AStar.h"

#include <iostream>
#include <list>

#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"

using namespace obvious;
using namespace std;

int main(int argc, char* argv[])
{
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <path_to_file>" << endl;
    return 1;
  }

  Timer timer;

  double robotRadius    = 0.1;

  AStarMap* map         = AStarMap::load(argv[1]);
  unsigned int width    = map->getWidth();
  unsigned int height   = map->getHeight();

  unsigned char* buffer = new unsigned char[width * height * 3];
  map->convertToImage(buffer);
  obvious::serializePPM("/tmp/map.ppm", buffer, width, height, false);

  ObstacleBounds bounds;
  bounds.xmin = 1.15;
  bounds.xmax = 1.75;
  bounds.ymin = -2.1;
  bounds.ymax = -1.5;
  Obstacle obstacle(bounds);
  obstacle.inflate(robotRadius);
  map->addObstacle(obstacle);
  timer.reset();
  map->inflate(robotRadius);
  cout << "elapsed for inflation: " << timer.reset() << " ms" << endl;
  map->convertToImage(buffer);
  obvious::serializePPM("/tmp/map_inflated.ppm", buffer, width, height, false);

  map->serialize("/tmp/map.txt");

  unsigned int idxStart[2];
  unsigned int idxTarget[2];
  idxStart[0]  = 486;
  idxStart[1]  = 334;
  idxTarget[0] = 646;
  idxTarget[1] = 594;
  AStarCoord coordStart;
  AStarCoord coordTarget;
  map->translateIndexToCoord(idxStart, &coordStart);
  map->translateIndexToCoord(idxTarget, &coordTarget);
  timer.reset();
  vector<unsigned int> path = AStar::pathFind(map, coordStart, coordTarget);
  cout << "elapsed for planning: " << timer.reset() << " ms" << endl;

  int cont_route = 0;

  cout << "Path in AStar representation:" << endl;
  for(vector<unsigned int>::iterator it=path.begin(); it!=path.end(); ++it)
  {
    cout << *it;
    cont_route++;
  }

  cout << endl << endl;

  vector<AStarCoord> coords = map->translatePathToCoords(path, coordStart);
  cout << "Path in coordinates:" << endl;
  for(vector<AStarCoord>::iterator it=coords.begin(); it!=coords.end(); ++it)
    cout << (*it).x << " " << (*it).y << endl;
  cout << endl;

  vector<unsigned int> mapIdx = map->translatePathToMapIndices(path, coordStart);

  for(vector<unsigned int>::iterator it=mapIdx.begin(); it!=mapIdx.end(); ++it)
  {
    buffer[3*(*it)] = 0;
    buffer[3*(*it)+1] = 0;
    buffer[3*(*it)+2] = 0;
  }

  obvious::serializePPM("/tmp/path.ppm", buffer, width, height, false);

  if (cont_route < 3)
    cout << "imposible to find a route" << endl;
  delete map;
  delete [] buffer;
}
