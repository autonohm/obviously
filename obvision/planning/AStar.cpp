#include "AStar.h"

#include <queue>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <algorithm>

#include "AStarNode.h"
#include "obcore/base/System.h"

namespace obvious
{

using namespace std;

bool operator<(const AStarNode & a, const AStarNode & b)
{
  return a.getPriority() > b.getPriority();
}

std::vector<unsigned int> AStar::pathFind(AStarMap* map, const Point2D coordStart, const Point2D coordTarget)
{
  Pixel start;
  Pixel target;
	start.u = (unsigned int)((coordStart.x / (obfloat)map->getCellSize()) + map->getWidth()/2  + 0.5);
	start.v = (unsigned int)((coordStart.y / (obfloat)map->getCellSize()) + map->getHeight()/2 + 0.5);
	target.u = (unsigned int)((coordTarget.x / (obfloat)map->getCellSize()) + map->getWidth()/2 + 0.5);
	target.v = (unsigned int)((coordTarget.y / (obfloat)map->getCellSize()) + map->getHeight()/2 + 0.5);

	return pathFind(map, start, target);
}

std::vector<unsigned int> AStar::pathFind(AStarMap* map, const Pixel start, const Pixel target)
{
  cout << __PRETTY_FUNCTION__ << endl;
  priority_queue<AStarNode> pq[2]; // list of open (not-yet-tried) MapNodes
  int pqi; // pq index
  AStarNode* n0;
  AStarNode* m0;
  unsigned int i, j, x, y;
  int xdx, ydy;

  int dx[8];
  int dy[8];

  dx[0]=1;  dy[0]=0;  // 1
  dx[1]=1;  dy[1]=1;  // 2
  dx[2]=0;  dy[2]=1;  // 3
  dx[3]=-1; dy[3]=1;  // 4
  dx[4]=-1; dy[4]=0;  // 5
  dx[5]=-1; dy[5]=-1; // 6
  dx[6]=0;  dy[6]=-1; // 7
  dx[7]=1;  dy[7]=-1; // 8

  pqi=0;

  unsigned int height  = map->getHeight();
  unsigned int width   = map->getWidth();

  char** buffer;
  obvious::System<char>::allocate(height, width, buffer);
  map->getMapWithObstacles(buffer);

  // check if start is valid
  if(start.u >= width || start.v>=height)
  {
    cout << "invalid u" << endl;
    return std::vector<unsigned int>();
  }
  else
  {
    if(buffer[start.v][start.u]!=0)
    {
      cout << "invalid start" << endl;
      return std::vector<unsigned int>();
    }
  }


  // check if target is valid
  if(target.u >= width || target.v>=height)
  {
    cout << "invalid v" << endl;
    return std::vector<unsigned int>();
  }
  else
  {
    if(buffer[target.v][target.u]!=0)
    {
      cout << "invalid target" << endl;
      return std::vector<unsigned int>();
    }
  }

  int** closedNodesMap;
  int** openNodesMap;
  int** dirMap;
  obvious::System<int>::allocate(height, width, closedNodesMap);
  obvious::System<int>::allocate(height, width, openNodesMap);
  obvious::System<int>::allocate(height, width, dirMap);

  // reset the Node maps
  for(y=0;y<height;y++)
  {
    for(x=0;x<width;x++)
    {
      closedNodesMap[y][x] = 0;
      openNodesMap[y][x]   = 0;
      dirMap[y][x]         = 0;
    }
  }

  // create the start Node and push into list of open Nodes
  n0=new AStarNode(start.u, start.v, 0, 0);
  n0->updatePriority(target.u, target.v);
  pq[pqi].push(*n0);

  openNodesMap[start.v][start.u]=n0->getPriority(); // mark it on the open Nodes map

  // A* search
  while(!pq[pqi].empty())
  {
    // get the current Node w/ the highest priority from the list of open Nodes
    n0=new AStarNode( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x=n0->getxPos(); y=n0->getyPos();

    pq[pqi].pop(); // remove the Node from the open list
    openNodesMap[y][x]=0;
    // mark it on the closed Nodes map
    closedNodesMap[y][x]=1;

    // quit searching when the goal state is reached if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==target.u && y==target.v)
    {
      // generate the path from finish to start by following the directions
      //string path="";
      std::vector<unsigned int> path;
      while(!(x==start.u && y==start.v))
      {
        j=dirMap[y][x];
        path.push_back((j+4)%8);
        x+=dx[j];
        y+=dy[j];
      }

      obvious::System<int>::deallocate(closedNodesMap);
      obvious::System<int>::deallocate(openNodesMap);
      obvious::System<int>::deallocate(dirMap);
      obvious::System<char>::deallocate(buffer);
      delete n0;
      // empty the leftover Nodes
      while(!pq[pqi].empty()) pq[pqi].pop();

      std::reverse(path.begin(), path.end());
      return path;
    }

    // generate moves (child Nodes) in all possible directions
    for(i=0;i<8;i++)
    {
      xdx=x+dx[i]; ydy=y+dy[i];

      if(!(xdx<0 || xdx>(int)(width-1) || ydy<0 || ydy>(int)(height-1) || buffer[ydy][xdx]!=0 || closedNodesMap[ydy][xdx]==1))
      {
        // generate a child Node
        m0=new AStarNode( xdx, ydy, n0->getLevel(), n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(target.u, target.v);

        // if it is not in the open list then add into that
        if(openNodesMap[ydy][xdx]==0)
        {
          openNodesMap[ydy][xdx]=m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent Node direction
          dirMap[ydy][xdx]=(i+4)%8;
        }
        else if(openNodesMap[ydy][xdx]>m0->getPriority())
        {
          // update the priority info
          openNodesMap[ydy][xdx]=m0->getPriority();
          // update the parent direction info
          dirMap[ydy][xdx]=(i+4)%8;

          // replace the node by emptying one pq to the other one except the node to be replaced will be ignored and the new node will be pushed in instead
          while(!(pq[pqi].top().getxPos()==xdx && pq[pqi].top().getyPos()==ydy))
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
        else delete m0;
      }
    }
    delete n0;
  }

  obvious::System<char>::deallocate(buffer);
  obvious::System<int>::deallocate(closedNodesMap);
  obvious::System<int>::deallocate(openNodesMap);
  obvious::System<int>::deallocate(dirMap);
  return std::vector<unsigned int>(); // no route found
}

} /* namespace obvious */
