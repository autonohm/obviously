#include "obvision/planning/AStar.h"

#include <iostream>
#include <list>

using namespace obvious;
using namespace std;

int main(int argc, char* argv[])
{
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <path_to_file>" << endl;
    return 1;
  }

  int width = 1024;
  int height = 1024;
  int resolution = 40;
  bool obstacleInflation = true;
  int inflationFactor = 8;
  double robotRadius = 0.25;

  AStar astar(width, height, resolution, obstacleInflation, inflationFactor, robotRadius);
  astar.load(argv[1]);
  astar.serialize("/tmp/map.ppm");

  list<int> path = astar.pathFind(495, 633, 522, 340);

  for(list<int>::iterator it=path.begin(); it!=path.end(); ++it)
    cout << *it;
  cout << endl;
}
