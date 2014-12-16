#include "AStarMap.h"

#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "obcore/base/System.h"

using namespace std;

namespace obvious
{

AStarMap::AStarMap(obfloat cellSize, unsigned int cellsX, unsigned int cellsY)
{
  _cellsX    = cellsX;
  _cellsY    = cellsY;
  _cellSize  = cellSize;

  obvious::System<char>::allocate(_cellsY, _cellsX, _map);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapBuf);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapInflated);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapObstacle);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _dirMap);

  for(unsigned int y=0;y<_cellsY;y++)
  {
    for(unsigned int x=0;x<_cellsX;x++)
    {
      _closedNodesMap[y][x] = 0;
      _openNodesMap[y][x]   = 0;
      _map[y][x]            = 1;
      _mapInflated[y][x]    = 1;
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
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapBuf);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapInflated);
  obvious::System<char>::allocate(_cellsY, _cellsX, _mapObstacle);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY,  _cellsX, _dirMap);

  memcpy(*_closedNodesMap, *(map._closedNodesMap), _cellsX*_cellsY*sizeof(**_closedNodesMap));
  memcpy(*_openNodesMap, *(map._openNodesMap), _cellsX*_cellsY*sizeof(**_openNodesMap));
  memcpy(*_map, *(map._map), _cellsX*_cellsY*sizeof(**_map));
  memcpy(*_mapInflated, *(map._mapInflated), _cellsX*_cellsY*sizeof(**_mapInflated));
  memcpy(*_mapObstacle, *(map._mapObstacle), _cellsX*_cellsY*sizeof(**_mapObstacle));

  _mapIsDirty = map._mapIsDirty;
}

AStarMap::~AStarMap()
{
  obvious::System<char>::deallocate(_map);
  obvious::System<char>::deallocate(_mapBuf);
  obvious::System<char>::deallocate(_mapInflated);
  obvious::System<char>::deallocate(_mapObstacle);
  obvious::System<int>::deallocate(_closedNodesMap);
  obvious::System<int>::deallocate(_openNodesMap);
  obvious::System<int>::deallocate(_dirMap);
}

void AStarMap::setData(char* data)
{
  pthread_mutex_lock(&_mutex);
  memcpy(*_map, data, _cellsX*_cellsY*sizeof(**_map));
  pthread_mutex_unlock(&_mutex);
}

unsigned int AStarMap::getWidth()
{
  return _cellsX;
}

unsigned int AStarMap::getHeight()
{
  return _cellsY;
}

obfloat AStarMap::getCellSize()
{
  return _cellSize;
}

void AStarMap::addObstacle(Obstacle &obstacle)
{
  Obstacle o(obstacle);
  _obstacles.push_back(o);
  _mapIsDirty = true;
}

void AStarMap::removeObstacle(Obstacle* obstacle)
{
  for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
  {
    if((*it).getID()==obstacle->getID())
    {
      it = _obstacles.erase(it);
      _mapIsDirty = true;
    }
  }
}

void AStarMap::removeAllObstacles()
{
  _obstacles.clear();
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

void AStarMap::inflate(obfloat robotRadius)
{
//  pthread_mutex_lock(&_mutex);
  memcpy(*_mapInflated, *_map, _cellsX*_cellsY*sizeof(**_mapInflated));

  int radius = static_cast<unsigned int>(robotRadius / _cellSize + 0.5);

  for(unsigned int y = 0; y < _cellsY; y++)
  {
    for(unsigned int x = 0; x < _cellsX; x++)
    {
      if(_map[y][x]!=0)  //obstacle
      {
        for(unsigned int v = y - radius; v < y + radius; v++)
        {
          if(v<_cellsY)
          {
            for(unsigned int u = x - radius; u < x + radius; u++)
            {
              if(u < _cellsX)
                _mapInflated[v][u]=1;
            }
          }
        }
      }
    }
  }
//  pthread_mutex_unlock(&_mutex);
}

char** AStarMap::getMap(bool inflated)
{
  char** map = _map;
  if(inflated) map = _mapInflated;
  pthread_mutex_lock(&_mutex);
  memcpy(*_mapBuf, *map, _cellsX*_cellsY*sizeof(**_mapBuf));
  pthread_mutex_unlock(&_mutex);
  return _mapBuf;
}

char** AStarMap::getMapWithObstacles(bool inflated)
{
  pthread_mutex_lock(&_mutex);
  if(_mapIsDirty)
  {
    if(inflated)
      memcpy(*_mapObstacle, *_mapInflated, _cellsX*_cellsY*sizeof(**_mapInflated));
    else
      memcpy(*_mapObstacle, *_map, _cellsX*_cellsY*sizeof(**_map));

    std::cout << __PRETTY_FUNCTION__  << "test 1" << std::endl;

    for(list<Obstacle>::iterator it=_obstacles.begin(); it!=_obstacles.end(); ++it)
    {
      ObstacleBounds* bounds = it->getBounds();
      int xmin = int(bounds->xmin/_cellSize+0.5)+_cellsX/2;
      int xmax = int(bounds->xmax/_cellSize+0.5)+_cellsX/2;
      int ymin = int(bounds->ymin/_cellSize+0.5)+_cellsY/2;
      int ymax = int(bounds->ymax/_cellSize+0.5)+_cellsY/2;

      std::cout << __PRETTY_FUNCTION__  << "test 2" << std::endl;

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
  pthread_mutex_unlock(&_mutex);
  return _mapObstacle;
}

void AStarMap::convertToImage(unsigned char* buffer, bool inflated)
{
  char** map = _map;
  if(inflated) map = _mapInflated;
  pthread_mutex_lock(&_mutex);
  for(unsigned int y=0;y<_cellsY;y++)
  {
    for(unsigned int x=0;x<_cellsX;x++)
    {
      unsigned int idx = 3 * (y * _cellsX + x);
      if(map[y][x] == 0)
      {
        buffer[idx + 0] = 0;
        buffer[idx + 1] = 255;
        buffer[idx + 2] = 0;
      }
      else if(map[y][x] == 50)
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
  pthread_mutex_unlock(&_mutex);
}

void AStarMap::translatePixelToCoord(Pixel pixel, Point2D* coord)
{
  coord->x = ((obfloat)(((int)pixel.u) - ((int)_cellsX/2)))*_cellSize;
  coord->y = ((obfloat)(((int)pixel.v) - ((int)_cellsY/2)))*_cellSize;
}

void AStarMap::translateCoordToPixel(Point2D coord, Pixel* pixel)
{
  (*pixel).u = (coord.x/_cellSize) + _cellsX/2;
  (*pixel).v = (coord.y/_cellSize) + _cellsY/2;
}

std::vector<unsigned int> AStarMap::translatePathToMapIndices(std::vector<unsigned int> path, Point2D coordStart)
{
  Pixel pixel;

  translateCoordToPixel(coordStart, &pixel);

  std::vector<unsigned int> index;
  unsigned int currentPos = pixel.v*_cellsX + pixel.u;

  index.push_back(currentPos);
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
    index.push_back(currentPos);
  }
  return index;
}

std::vector<Point2D> AStarMap::translatePathToCoords(std::vector<unsigned int> path, Point2D coordStart)
{
  std::vector<Point2D> coords;
  Point2D pos;
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
    obfloat resolution;
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

AStarMap* AStarMap::create(const char* data, obfloat cellSize, unsigned int width, unsigned int height)
{
  //Creates a map form a occupancy grid
  AStarMap* map = new AStarMap(cellSize, width, height);
  memcpy(*(map->_map), data, width*height*sizeof(**(map->_map)));

  return map;
}

void AStarMap::serialize(std::string path)
{
  pthread_mutex_lock(&_mutex);
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
  pthread_mutex_unlock(&_mutex);
}
} /* namespace obvious */
