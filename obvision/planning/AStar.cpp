#include "AStar.h"

#include <queue>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>

#include "AStarNode.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

using namespace std;

AStar::AStar(int width, int height, int resolution, bool obstacleInflation, int inflationFactor, double robotRadius)
{
  _initial     = true;
  _width  = width;
  _height = height;
  _resolution = resolution;
  _obstacleInflation = obstacleInflation;
  _inflationFactor = inflationFactor;
  _robotRadius = robotRadius;
  _dir    = 8;

  _gridOffsetX = NAN;
  _gridOffsetY = NAN;

  obvious::System<int>::allocate(_width, _height, _map);
  obvious::System<int>::allocate(_width, _height, _map_tmp);
  obvious::System<int>::allocate(_width, _height, _closed_nodes_map);
  obvious::System<int>::allocate(_width, _height, _open_nodes_map);
  obvious::System<int>::allocate(_width, _height, _dir_map);
  obvious::System<int>::allocate(_width, _height, _originalMap);

  //initialization of the maps
  for(int y=0;y<_height;y++)
  {
    for(int x=0;x<_width;x++)
    {
      _closed_nodes_map[x][y] = 0;
      _open_nodes_map[x][y]   = 0;
      _map[x][y]              = 1;
      _map_tmp[x][y]          = 1;
    }
  }

  //  int _dx[8]= {1, 1, 0, -1, -1, -1, 0, 1};
  //  int _dy[8]= {0, 1, 1, 1, 0, -1, -1, -1};
  for (int i=0;i<8;i++ )
  {
    if (i==0) {_dx[i]=1;  _dy[i]=0;}
    if (i==1) {_dx[i]=1;  _dy[i]=1;}
    if (i==2) {_dx[i]=0;  _dy[i]=1;}
    if (i==3) {_dx[i]=-1; _dy[i]=1;}
    if (i==4) {_dx[i]=-1; _dy[i]=0;}
    if (i==5) {_dx[i]=-1; _dy[i]=-1;}
    if (i==6) {_dx[i]=0;  _dy[i]=-1;}
    if (i==7) {_dx[i]=1;  _dy[i]=-1;}
  }

}

AStar::~AStar()
{
  obvious::System<int>::deallocate(_map);
  obvious::System<int>::deallocate(_map_tmp);
  obvious::System<int>::deallocate(_closed_nodes_map);
  obvious::System<int>::deallocate(_open_nodes_map);
  obvious::System<int>::deallocate(_dir_map);
  obvious::System<int>::deallocate(_originalMap);
}

void AStar::reset()
{
  for (int j=0; j < _height; j++)
  {
    for( int i= 0; i < _width; i++)
    {
      _map[i][j] = _originalMap[i][j];
    }
  }
}

void AStar::load(std::string path)
{
  //Create a map from a File .txt
  string line;
  ifstream myfile;

  myfile.open(path.c_str(), std::ifstream::in);
  int x= 0, y =0;
  if (myfile.is_open())
  {
    while(getline (myfile,line))
    {
      //Wall or Unknown
      if ((atoi(line.c_str())==100) || (atoi(line.c_str())==-1))
      {
        _map[x][y] = 1;
        _map_tmp[x][y] = 1;
      }
      //empty space
      else if (atoi(line.c_str())==0)
      {
        _map[x][y] = 0;
        _map_tmp[x][y] = 0;
      }
      if (x==1023)
      {
        y++;
        x= 0;
      }
      else
        x++;
    }
    myfile.close();
  }
  else
    cout << "Opening of file " << path << " failed!\n";
}

void AStar::create(char* data, int width, int height, int offsetX, int offsetY, int resolution)
{
  _gridOffsetX = offsetX;
  _gridOffsetY = offsetY;
  int robotRadiusCells = static_cast<unsigned int>(_robotRadius / resolution + 0.5);

  // reset obstacle map
  for(int y = 0; y < height; y++)
    for(int x = 0; x < width; x++)
      _map[x][y] = 0;


  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      char d = data[y * width + x];

      if((d == 100) || (d == -1))  //obstacle
      {
        _map[x][y] = 1;
        if(_obstacleInflation)
        {
          for(int v = y - robotRadiusCells; v < y + robotRadiusCells; v++)
          {
            for(int u = x - robotRadiusCells; u < x + robotRadiusCells; u++)
            {
              if((u >= _width) || (v >= _height))
                continue;
              else
              {
                _map[u][v] = 1;
              }
            }
          }
        }
      }
    }
  }
  for (int i=0; i < _width; i++)
  {
    for( int j= 0; j < _height; j++)
    {
      _originalMap[j][i]=_map[j][i];
    }
  }
  _initial = false;
}

void AStar::serialize(char* path)
{
  // visualization of the _map I created
  unsigned char* buffer = new unsigned char[_width * _height * 3];
  for(int y=0;y<_height;y++)
  {
    for(int x=0;x<_width;x++)
    {
      unsigned int idx = 3 * (y * _height + x);
      if(_map[x][y] == 1)
      {
        buffer[idx + 0] = 255;
        buffer[idx + 1] = 0;
        buffer[idx + 2] = 0;
      }
      else if(_map[x][y] == 0)
      {
        buffer[idx + 0] = 0;
        buffer[idx + 1] = 255;
        buffer[idx + 2] = 0;
      }
    }
  }
  obvious::serializePPM(path, buffer, _height, _width, false);
  delete buffer;
}

void AStar::addObstacle(Obstacle obstacle)
{
  vector<double> xcoords;
  vector<double> ycoords;
  obstacle.getCoords(xcoords, ycoords);
  for (unsigned i =0; i < xcoords.size(); i++)
  {
    _map[int(xcoords[i]*_resolution)][int(ycoords[i]*_resolution)] = 1;
  }

  //obstacle_inflation X veces
  for(int x=0; x < 10; x++)   //toDo: remove magic number -> launch parameter
  {
    for(int j=0; j < _height; j++)
    {
      for(int i= 0; i< _width; i++)
      {
        if((_map[i][j]== 1) && (j != (_height-1)) && (j != 0) && (i != (_width-1)) && (i != 0))
        {
          _map_tmp[i+1][j]=1;
          _map_tmp[i-1][j]=1;
          _map_tmp[i][j+1]=1;
          _map_tmp[i][j-1]=1;
          _map_tmp[i+1][j+1]=1;
          _map_tmp[i+1][j-1]=1;
          _map_tmp[i-1][j+1]=1;
          _map_tmp[i-1][j-1]=1;
        }
      }
    }

    memcpy(*_map, *_map_tmp, _height*_width*sizeof(**_map));
  }
}

void AStar::removeObstacle(Obstacle obstacle)
{
  cout << "To be done!" << endl;
}

bool operator<(const AStarNode & a, const AStarNode & b)
{
  return a.getPriority() > b.getPriority();
}

std::list<int> AStar::pathFind(const int & xStart, const int & yStart, const int & xFinish, const int & yFinish)
{
  static priority_queue<AStarNode> pq[2]; // list of open (not-yet-tried) MapNodes
  static int pqi; // pq index
  static AStarNode* n0;
  static AStarNode* m0;
  static int i, j, x, y, xdx, ydy;

  pqi=0;

  // reset the Node maps
  for(y=0;y<_height;y++)
  {
    for(x=0;x<_width;x++)
    {
      _closed_nodes_map[x][y]=0;
      _open_nodes_map[x][y]=0;
    }
  }

  // create the start Node and push into list of open Nodes
  n0=new AStarNode(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);

  _open_nodes_map[xStart][yStart]=n0->getPriority(); // mark it on the open Nodes map

  // A* search
  while(!pq[pqi].empty())
  {
    // get the current Node w/ the highest priority from the list of open Nodes
    n0=new AStarNode( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
        pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x=n0->getxPos(); y=n0->getyPos();

    pq[pqi].pop(); // remove the Node from the open list
    _open_nodes_map[x][y]=0;
    // mark it on the closed Nodes map
    _closed_nodes_map[x][y]=1;

    // quit searching when the goal state is reached if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==xFinish && y==yFinish)
    {
      // generate the path from finish to start by following the directions
      //string path="";
      list<int> path;
      while(!(x==xStart && y==yStart))
      {
        j=_dir_map[x][y];
        path.push_front((j+_dir/2)%_dir);
        x+=_dx[j];
        y+=_dy[j];
      }

      // garbage collection
      delete n0;
      // empty the leftover Nodes
      while(!pq[pqi].empty()) pq[pqi].pop();
      return path;
    }

    // generate moves (child Nodes) in all possible directions
    for(i=0;i<_dir;i++)
    {
      xdx=x+_dx[i]; ydy=y+_dy[i];

      if(!(xdx<0 || xdx>_width-1 || ydy<0 || ydy>_height-1 || _map[xdx][ydy]==1
          || _closed_nodes_map[xdx][ydy]==1))
      {
        // generate a child Node
        m0=new AStarNode( xdx, ydy, n0->getLevel(),
            n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if(_open_nodes_map[xdx][ydy]==0)
        {
          _open_nodes_map[xdx][ydy]=m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent Node direction
          _dir_map[xdx][ydy]=(i+_dir/2)%_dir;
        }
        else if(_open_nodes_map[xdx][ydy]>m0->getPriority())
        {
          // update the priority info
          _open_nodes_map[xdx][ydy]=m0->getPriority();
          // update the parent direction info
          _dir_map[xdx][ydy]=(i+_dir/2)%_dir;

          // replace the node by emptying one pq to the other one except the node to be replaced will be ignored and the new node will be pushed in instead
          while(!(pq[pqi].top().getxPos()==xdx &&
              pq[pqi].top().getyPos()==ydy))
          {
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pq[pqi].pop(); // remove the wanted Node

          // empty the larger size pq to the smaller one
          if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
          while(!pq[pqi].empty())
          {
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pqi=1-pqi;
          pq[pqi].push(*m0); // add the better Node instead
        }
        else delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return list<int>(); // no route found
}


} /* namespace obvious */
