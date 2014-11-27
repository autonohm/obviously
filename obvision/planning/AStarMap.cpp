#include "AStarMap.h"

#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "obcore/base/System.h"

using namespace std;

namespace obvious
{

AStarMap::AStarMap(double cellSize, unsigned int cellsX, unsigned int cellsY)
{
  _cellsX    = cellsX;
  _cellsY    = cellsY;
  _cellSize  = cellSize;

  obvious::System<char>::allocate(_cellsY, _cellsX, _map);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapObstacle);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapWork);
  obvious::System<int>::allocate(_cellsY, _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _dirMap);

  for(unsigned int y=0;y<_cellsY;y++)
  {
    for(unsigned int x=0;x<_cellsX;x++)
    {
      _closedNodesMap[y][x] = 0;
      _openNodesMap[y][x]   = 0;
      _map[y][x]        = 1;
    }
  }

  _mapIsDirty = true;
}

AStarMap::AStarMap(AStarMap &map)
{
  _cellsX   = map._cellsX;
  _cellsY   = map._cellsY;
  _cellSize = map._cellSize;

  obvious::System<char>::allocate(_cellsY, _cellsX, _map);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapObstacle);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapWork);
  obvious::System<int>::allocate(_cellsY, _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _dirMap);

  memcpy(*_closedNodesMap, *(map._closedNodesMap), _cellsX*_cellsY*sizeof(**_closedNodesMap));
  memcpy(*_openNodesMap, *(map._openNodesMap), _cellsX*_cellsY*sizeof(**_openNodesMap));
  memcpy(*_map, *(map._map), _cellsX*_cellsY*sizeof(**_map));
  memcpy(*_mapObstacle, *(map._mapObstacle), _cellsX*_cellsY*sizeof(**_mapObstacle));

  _mapIsDirty = map._mapIsDirty;
}

AStarMap::~AStarMap()
{
  obvious::System<char>::deallocate(_map);
  obvious::System<char>::deallocate(_mapObstacle);
  obvious::System<char>::deallocate(_mapWork);
  obvious::System<int>::deallocate(_closedNodesMap);
  obvious::System<int>::deallocate(_openNodesMap);
  obvious::System<int>::deallocate(_dirMap);
}

unsigned int AStarMap::getWidth()
{
  return _cellsX;
}

unsigned int AStarMap::getHeight()
{
  return _cellsY;
}

double AStarMap::getCellSize()
{
  return _cellSize;
}

void AStarMap::addObstacle(Obstacle obstacle)
{
  _obstacles.push_back(obstacle);
  _mapIsDirty = true;
}

void AStarMap::removeObstacle(Obstacle obstacle)
{
  for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
  {
    if((*it).getID()==obstacle.getID())
      _obstacles.erase(it);
  }
  _mapIsDirty = true;
}

Obstacle* AStarMap::checkObstacleIntersection(Obstacle obstacle)
{
  for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
  {
    Obstacle* o = &(*it);
    if(o->intersects(&obstacle)) return o;
  }
  return NULL;
}

void AStarMap::inflate(double robotRadius)
{
  memcpy(*_mapWork, *_map, _cellsY*_cellsX*sizeof(**_map));

  int radius = static_cast<unsigned int>(robotRadius / _cellSize + 0.5);

  for(unsigned int y = 0; y < _cellsY; y++)
  {
    for(unsigned int x = 0; x < _cellsX; x++)
    {
      if(_mapWork[y][x]!=0)  //obstacle
      {
        for(unsigned int v = y - radius; v < y + radius; v++)
        {
          if(v<_cellsY)
          {
            for(unsigned int u = x - radius; u < x + radius; u++)
            {
              if(u < _cellsX)
                _map[v][u]=1;
            }
          }
        }
      }
    }
  }
  _mapIsDirty = true;
}

char** AStarMap::getMapWithObstacles()
{
  if(_mapIsDirty)
  {
    memcpy(*_mapObstacle, *_map, _cellsX*_cellsY*sizeof(**_map));
    for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
    {
      ObstacleBounds* bounds = it->getBounds();
      int xmin = int(bounds->xmin/_cellSize+0.5)+_cellsX/2;
      int xmax = int(bounds->xmax/_cellSize+0.5)+_cellsX/2;
      int ymin = int(bounds->ymin/_cellSize+0.5)+_cellsY/2;
      int ymax = int(bounds->ymax/_cellSize+0.5)+_cellsY/2;

      for(int y=ymin; y<ymax; y++)
      {
        for(int x=xmin; x<xmax; x++)
        {
          _mapObstacle[y][x] = 1;
        }
      }
    }
    _mapIsDirty = false;
  }
  return _mapObstacle;
}

void AStarMap::convertToImage(unsigned char* buffer)
{
  for(unsigned int y=0;y<_cellsY;y++)
  {
    for(unsigned int x=0;x<_cellsX;x++)
    {
      unsigned int idx = 3 * (y * _cellsX + x);
      if(_map[y][x] == 0)
      {
        buffer[idx + 0] = 0;
        buffer[idx + 1] = 255;
        buffer[idx + 2] = 0;
      }
      else if(_map[y][x] == 50)
      {
        buffer[idx + 0] = 0;
        buffer[idx + 1] = 0;
        buffer[idx + 2] = 255;
      }
      else
      {
        buffer[idx + 0] = 255;
        buffer[idx + 1] = 0;
        buffer[idx + 2] = 0;
      }
    }
  }

  for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
  {
    Obstacle* o = &(*it);
    ObstacleBounds* bounds = o->getBounds();
    int xmin = int(bounds->xmin/_cellSize+0.5)+_cellsX/2;
    int xmax = int(bounds->xmax/_cellSize+0.5)+_cellsX/2;
    int ymin = int(bounds->ymin/_cellSize+0.5)+_cellsY/2;
    int ymax = int(bounds->ymax/_cellSize+0.5)+_cellsY/2;

    for(int y=ymin; y<ymax; y++)
    {
      for(int x=xmin; x<xmax; x++)
      {
        unsigned int idx = 3 * (y * _cellsX + x);
        buffer[idx + 0] = 0;
        buffer[idx + 1] = 0;
        buffer[idx + 2] = 255;
      }
    }
  }
}

void AStarMap::translateIndexToCoord(unsigned int idx[2], AStarCoord* coord)
{
  coord->x = ((double)(((int)idx[0]) - ((int)_cellsX/2)))*_cellSize;
  coord->y = ((double)(((int)idx[1]) - ((int)_cellsY/2)))*_cellSize;
}

void AStarMap::translateCoordToIndex(AStarCoord coord, unsigned int idx[2])
{
  idx[0] = (coord.x/_cellSize) + _cellsX/2;
  idx[1] = (coord.y/_cellSize) + _cellsY/2;
}

std::vector<unsigned int> AStarMap::translatePathToMapIndices(std::vector<unsigned int> path, AStarCoord coordStart)
{
  unsigned int idx[2];

  translateCoordToIndex(coordStart, idx);

  std::vector<unsigned int> pixel;
  pixel.push_back(idx[0]);

  unsigned int currentPos = idx[1]*_cellsX + idx[0];
  for(vector<unsigned int>::iterator it=path.begin(); it!=path.end(); ++it)
  {
    switch(*it)
    {
    case 0:
      currentPos++;
      break;
    case 1:
      currentPos+=(_cellsX+1);
      break;
    case 2:
      currentPos+=_cellsX;
      break;
    case 3:
      currentPos+=(_cellsX-1);
      break;
    case 4:
      currentPos--;
      break;
    case 5:
      currentPos-=(_cellsX+1);
      break;
    case 6:
      currentPos-=_cellsX;
      break;
    case 7:
      currentPos-=(_cellsX-1);
      break;
    }
    pixel.push_back(currentPos);
  }
  return pixel;
}

std::vector<AStarCoord> AStarMap::translatePathToCoords(std::vector<unsigned int> path, AStarCoord coordStart)
{
  std::vector<AStarCoord> coords;
  AStarCoord pos;
  pos.x = coordStart.x;
  pos.y = coordStart.y;
  coords.push_back(pos);

  for(vector<unsigned int>::iterator it=path.begin(); it!=path.end(); ++it)
  {
    switch(*it)
    {
    case 0:
      pos.x += _cellSize;
      break;
    case 1:
      pos.x += _cellSize;
      pos.y += _cellSize;
      break;
    case 2:
      pos.y += _cellSize;
      break;
    case 3:
      pos.x -= _cellSize;
      pos.y += _cellSize;
      break;
    case 4:
      pos.x -= _cellSize;
      break;
    case 5:
      pos.x -= _cellSize;
      pos.y -= _cellSize;
      break;
    case 6:
      pos.y -= _cellSize;
      break;
    case 7:
      pos.x += _cellSize;
      pos.y -= _cellSize;
      break;
    }
    coords.push_back(pos);
  }
  return coords;
}

AStarMap* AStarMap::load(std::string path)
{
  //Create a map from a .txt file
  string line;
  ifstream file;

  file.open(path.c_str(), std::ifstream::in);

  if (file.is_open())
  {
    unsigned int width, height;
    double resolution;
    file >> resolution >> width >> height;

    AStarMap* map = new AStarMap(resolution, width, height);
    char** buffer = map->_map;

    int i = 0;
    while(getline (file, line))
    {
      buffer[0][i++] = atoi(line.c_str());
    }
    file.close();
    return map;
  }
  else
    cout << "Opening of input file " << path << " failed!" << endl;

  return NULL;
}

AStarMap AStarMap::create(char* data, double cellSize, unsigned int width, unsigned int height)
{
  AStarMap map(cellSize, width, height);
  memcpy(*(map._map), data, width*height*sizeof(**(map._map)));

  return map;
}

void AStarMap::serialize(std::string path)
{
  string line;
  ofstream file;

  file.open(path.c_str(), std::ofstream::out);

  if (file.is_open())
  {
    file << _cellSize << " " << _cellsX << " " << _cellsY << endl;

    for(unsigned int i=0; i<_cellsX*_cellsY; i++)
    {
      file << _map[0][i] << endl;
    }
    file.close();
  }
  else
    cout << "Opening of output file " << path << " failed!" << endl;
}
} /* namespace obvious */
