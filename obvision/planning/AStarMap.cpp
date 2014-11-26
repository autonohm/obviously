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

  obvious::System<int>::allocate(_cellsY, _cellsX, _map);
  obvious::System<int>::allocate(_cellsY, _cellsX, _mapWork);
  obvious::System<int>::allocate(_cellsY, _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _dirMap);

  for(unsigned int y=0;y<_cellsY;y++)
  {
    for(unsigned int x=0;x<_cellsX;x++)
    {
      _closedNodesMap[y][x] = 0;
      _openNodesMap[y][x]   = 0;
      _map[y][x]            = 1;
    }
  }
}

AStarMap::AStarMap(AStarMap &map)
{
  _cellsX = map._cellsX;
  _cellsY = map._cellsY;
  _cellSize = map._cellSize;

  obvious::System<int>::allocate(_cellsY, _cellsX, _map);
  obvious::System<int>::allocate(_cellsY, _cellsX, _mapWork);
  obvious::System<int>::allocate(_cellsY, _cellsX, _closedNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _openNodesMap);
  obvious::System<int>::allocate(_cellsY, _cellsX, _dirMap);

  memcpy(*_closedNodesMap, *(map._closedNodesMap), _cellsX*_cellsY*sizeof(**_closedNodesMap));
  memcpy(*_openNodesMap, *(map._openNodesMap), _cellsX*_cellsY*sizeof(**_openNodesMap));
  memcpy(*_map, *(map._map), _cellsX*_cellsY*sizeof(**_map));
}

AStarMap::~AStarMap()
{
  obvious::System<int>::deallocate(_map);
  obvious::System<int>::deallocate(_mapWork);
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
  vector<double> xcoords;
  vector<double> ycoords;
  obstacle.getCoords(xcoords, ycoords);
  for (unsigned i =0; i < xcoords.size(); i++)
  {
    _map[int(ycoords[i]/_cellSize+0.5)+_cellsY/2][int(xcoords[i]/_cellSize+0.5)+_cellsX/2]=50;
  }
}

void AStarMap::removeObstacle(Obstacle obstacle)
{
  vector<double> xcoords;
  vector<double> ycoords;
  obstacle.getCoords(xcoords, ycoords);
  for (unsigned i =0; i < xcoords.size(); i++)
  {
    _map[int(ycoords[i]/_cellSize+0.5)+_cellsY/2][int(xcoords[i]/_cellSize+0.5)+_cellsX/2]--;
  }
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
                _map[v][u]++;
            }
          }
        }
      }
    }
  }
}

void AStarMap::deinflate()
{
  memcpy(*_map, *_mapWork, _cellsY*_cellsX*sizeof(**_map));
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
}

void AStarMap::translateIndexToCoord(unsigned int xIdx, unsigned int yIdx, double* x, double* y)
{
  *x = ((double)(xIdx - _cellsX/2))*_cellSize;
  *y = ((double)(yIdx - _cellsY/2))*_cellSize;
}

void AStarMap::translateCoordToIndex(double x, double y, unsigned int* xIdx, unsigned int* yIdx)
{
  *xIdx = (x/_cellSize) + _cellsX/2;
  *yIdx = (y/_cellSize) + _cellsY/2;
}

std::vector<unsigned int> AStarMap::translatePathToMapIndices(std::vector<unsigned int> path, double xStart, double yStart)
{
  unsigned int xIdx;
  unsigned int yIdx;

  translateCoordToIndex(xStart, yStart, &xIdx, &yIdx);

  std::vector<unsigned int> pixel;
  pixel.push_back(xIdx);

  unsigned int currentPos = yIdx*_cellsX + xIdx;
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

std::vector<AStarCoord> AStarMap::translatePathToCoords(std::vector<unsigned int> path, double xStart, double yStart)
{
  std::vector<AStarCoord> coords;
  AStarCoord pos;
  pos.x = xStart;
  pos.y = yStart;
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
    int** buffer = map->_map;

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

AStarMap AStarMap::create(char* data, double cellSize, unsigned int width, unsigned int height)
{
  AStarMap map(cellSize, width, height);
  memcpy(*(map._map), data, width*height*sizeof(*data));

  return map;
}

} /* namespace obvious */
