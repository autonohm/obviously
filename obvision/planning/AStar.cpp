#include "AStar.h"

#include <stdexcept>
#include <map>
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

//bool operator<(const AStarNode* a, const AStarNode* b)
//{
//  return a->getPriority() > b->getPriority();
//}

class Comp{
public:
   bool operator()(const AStarNode* a, const AStarNode* b)
   {
      return a->getPriority() > b->getPriority();
   }
};


std::vector<unsigned int> AStar::pathFind(AStarMap* map, const Point2D coordStart, const Point2D coordTarget, const bool penalty, const Point2D* const offset)
{
  obfloat xOffset = 0.0;
  obfloat yOffset = 0.0;
  if(offset)
  {
    xOffset = offset->x / (obfloat)map->getCellSize();
    yOffset = offset->y / (obfloat)map->getCellSize();
  }
  else  //no offset given, assuming zero point in center of the map
  {
    xOffset = map->getWidth()/2;
    yOffset = map->getHeight()/2;
  }
  Pixel start;
  Pixel target;
	start.u = (unsigned int)((coordStart.x / (obfloat)map->getCellSize()) + xOffset + 0.5);
	start.v = (unsigned int)((coordStart.y / (obfloat)map->getCellSize()) + yOffset + 0.5);
	target.u = (unsigned int)((coordTarget.x / (obfloat)map->getCellSize()) + xOffset + 0.5);
	target.v = (unsigned int)((coordTarget.y / (obfloat)map->getCellSize()) + yOffset + 0.5);

	return pathFind(map, start, target, penalty);
}

std::vector<unsigned int> AStar::pathFind(AStarMap* map, const Pixel start, const Pixel target, const bool penalty)
{
  cout << __PRETTY_FUNCTION__ << endl;
  priority_queue<AStarNode*, std::vector<AStarNode*>, Comp> pq;//[2]; // list of open (not-yet-tried) MapNodes
  std::map<unsigned int, AStarNode*> openList_map;
  //int pqi; // pq index
  AStarNode* n0;
  AStarNode* m0;
  unsigned int i, j, x, y;
  int xdx, ydy;

  int dx[8];
  int dy[8];

  dx[0]=1;  dy[0]=0;  // 0
  dx[1]=1;  dy[1]=1;  // 1
  dx[2]=0;  dy[2]=1;  // 2
  dx[3]=-1; dy[3]=1;  // 3
  dx[4]=-1; dy[4]=0;  // 4
  dx[5]=-1; dy[5]=-1; // 5
  dx[6]=0;  dy[6]=-1; // 6
  dx[7]=1;  dy[7]=-1; // 7

  //pqi=0;

  unsigned int height  = map->getHeight();
  unsigned int width   = map->getWidth();

  char** buffer;
  obvious::System<char>::allocate(height, width, buffer);
  map->getMapWithObstacles(buffer);

  // check if start is valid
  if(start.u >= width || start.v>=height)
  {
    cout << "invalid start pixel u = " << start.u << " v = " << start.v << endl;
    return std::vector<unsigned int>();
  }
  else
  {
    if(buffer[start.v][start.u]!=0)
    {
      cout << "invalid start content in pixel u = " << start.u << " v = " << start.v << " content =  "
           << static_cast<int>(buffer[start.v][start.u]) << endl;
      return std::vector<unsigned int>();
    }
  }

  // check if target is valid
  if(target.u >= width || target.v>=height)
  {
    cout << "invalid target pixel u = " << target.u << " v = " << target.v << endl;
    return std::vector<unsigned int>();
  }
  else
  {
    if(buffer[target.v][target.u]!=0)
    {
      cout << "invalid target content pixel u = " << target.u << " v = " << target.v << " content =  "
           << static_cast<int>(buffer[target.v][target.u]) << endl;
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
  n0 = new AStarNode(start.u, start.v, 0, 0, -1);
  n0->updatePriority(target.u, target.v);
  pq.push(n0);
  openList_map.insert( std::make_pair( AStar::toId(n0->getxPos(), n0->getyPos(), map->getWidth()), n0) );

  openNodesMap[start.v][start.u]=n0->getPriority(); // mark it on the open Nodes map

  // A* search
  while(!pq.empty())
  {
    // get the current Node w/ the highest priority from the list of open Nodes
    AStarNode* prevNode;

    //std::cout << "before top" << std::endl;
    do{
       prevNode = pq.top();
       if(prevNode->_isOverwritten)
          pq.pop();
    }while(prevNode->_isOverwritten);

    //std::cout << "after top" << std::endl;

    n0 = new AStarNode( prevNode->getxPos(), prevNode->getyPos(), prevNode->getLevel(), prevNode->getPriority(), prevNode->getCurrentDir());

    x = n0->getxPos();
    y = n0->getyPos();

    pq.pop(); // remove the Node from the open list
    openList_map.erase(AStar::toId(prevNode->getxPos(), prevNode->getyPos(), map->getWidth()));

    openNodesMap[y][x]   = 0;
    // mark it on the closed Nodes map
    closedNodesMap[y][x] = 1;

    // quit searching when the goal state is reached if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==target.u && y==target.v)
    {
      // generate the path from finish to start by following the directions
      //string path="";
      std::vector<unsigned int> path;
      while(!(x==start.u && y==start.v))
      {
        j = dirMap[y][x];
        path.push_back((j+4)%8);
        x += dx[j];
        y += dy[j];
      }

      obvious::System<int>::deallocate(closedNodesMap);
      obvious::System<int>::deallocate(openNodesMap);
      obvious::System<int>::deallocate(dirMap);
      obvious::System<char>::deallocate(buffer);
      delete n0;

      // empty the leftover Nodes
      while(!pq.empty())
         pq.pop();

      openList_map.clear();

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
        m0 = new AStarNode( xdx, ydy, n0->getLevel(), n0->getPriority(), n0->getCurrentDir());
        if(penalty) m0->nextLevelPenalty(i);
        else m0->nextLevel(i);
        m0->updatePriority(target.u, target.v);

        // if it is not in the open list then add into that
        if(openNodesMap[ydy][xdx]==0)
        {
          openNodesMap[ydy][xdx] = m0->getPriority();
          m0->setCurrentDir((i+4)%8);
          pq.push(m0);
          openList_map.insert( std::make_pair(AStar::toId(m0->getxPos(), m0->getyPos(), map->getWidth()), m0) );
          // mark its parent Node direction
          dirMap[ydy][xdx]=(i+4)%8;
        }
        else if(openNodesMap[ydy][xdx]>m0->getPriority())
        {
          // update the priority info
          openNodesMap[ydy][xdx] = m0->getPriority();
          m0->setCurrentDir((i+4)%8);
          // update the parent direction info
          dirMap[ydy][xdx]=(i+4)%8;

          //set to old node overwritten flag
          try{
             openList_map.at(AStar::toId(m0->getxPos(), m0->getyPos(), map->getWidth()))->_isOverwritten = true;
          }catch (std::out_of_range& e) {
             std::cout << "Waring... try to use Node whitch is not existing" << std::endl;
         }

         // SM: Code below can be removed, since overwritten flag was added in version pushed at 17.8.2015, TODO: Delete commented code after verification
//          //replace the node by emptying one pq to the other one except the node to be replaced will be ignored and the new node will be pushed in instead
//          while(!(pq[pqi].top()->getxPos()==xdx && pq[pqi].top()->getyPos()==ydy))
//          {
//            pq[1-pqi].push(pq[pqi].top());
//            pq[pqi].pop();
//          }
//          pq[pqi].pop(); // remove the wanted Node
//
//          // empty the larger size pq to the smaller one
//          if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
//          while(!pq[pqi].empty())
//          {
//            pq[1-pqi].push(pq[pqi].top());
//            pq[pqi].pop();
//          }
//          //pqi=1-pqi;


          pq.push(m0); // add the better Node instead
          openList_map.insert( std::make_pair(AStar::toId(m0->getxPos(), m0->getyPos(), map->getWidth()), m0) );
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
