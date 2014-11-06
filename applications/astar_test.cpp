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

  timer.reset();
  map->inflate(robotRadius);
  cout << "elapsed for inflation: " << timer.reset() << " ms" << endl;
  map->convertToImage(buffer);
  obvious::serializePPM("/tmp/map_inflated.ppm", buffer, width, height, false);

  map->serialize("/tmp/map.txt");

  int xStart = 495;
  int yStart = 633;
  int xTarget = 626;
  int yTarget = 502;
  timer.reset();
  vector<unsigned int> path = AStar::pathFind(map, xStart, yStart, xTarget, yTarget);
  cout << "elapsed for planning: " << timer.reset() << " ms" << endl;

  cout << "Path in AStar representation:" << endl;
  for(vector<unsigned int>::iterator it=path.begin(); it!=path.end(); ++it)
    cout << *it;
  cout << endl << endl;

  vector<AStarCoord> coords = map->translatePathToCoords(path, xStart, yStart);
  cout << "Path in coordinates:" << endl;
  for(vector<AStarCoord>::iterator it=coords.begin(); it!=coords.end(); ++it)
    cout << (*it).x << " " << (*it).y << endl;
  cout << endl;

  vector<unsigned int> mapIdx = map->translatePathToMapIndices(path, xStart, yStart);

  for(vector<unsigned int>::iterator it=mapIdx.begin(); it!=mapIdx.end(); ++it)
  {
    buffer[3*(*it)] = 0;
    buffer[3*(*it)+1] = 0;
    buffer[3*(*it)+2] = 0;
  }

  obvious::serializePPM("/tmp/path.ppm", buffer, width, height, false);

  delete map;
  delete [] buffer;
}
