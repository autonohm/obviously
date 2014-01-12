#include "TsdSpaceBranch.h"
#include <math.h>
#include <iostream>

using namespace std;

namespace obvious
{

TsdSpaceBranch::TsdSpaceBranch(TsdSpaceComponent**** leafs, int x, int y, int z, int level) : TsdSpaceComponent(false)
{
  _edgeCoordsHom = new Matrix(8, 4);

  TsdSpaceComponent* branch;
  TsdSpaceComponent* branchRight;
  TsdSpaceComponent* branchUp;
  TsdSpaceComponent* branchUpRight;
  TsdSpaceComponent* branchBack;
  TsdSpaceComponent* branchBackRight;
  TsdSpaceComponent* branchUpBack;
  TsdSpaceComponent* branchUpBackRight;

  if(level==1)
  {
    branch            = leafs[z][y][x];
    branchRight       = leafs[z][y][x+1];
    branchUp          = leafs[z][y+1][x];
    branchUpRight     = leafs[z][y+1][x+1];
    branchBack        = leafs[z+1][y][x];
    branchBackRight   = leafs[z+1][y][x+1];
    branchUpBack      = leafs[z+1][y+1][x];
    branchUpBackRight = leafs[z+1][y+1][x+1];
  }
  else
  {
    level--;
    int step = (int)pow(2.0, level);
    branch            = new TsdSpaceBranch(leafs, x,      y,      z,      level);
    branchRight       = new TsdSpaceBranch(leafs, x+step, y,      z,      level);
    branchUp          = new TsdSpaceBranch(leafs, x,      y+step, z,      level);
    branchUpRight     = new TsdSpaceBranch(leafs, x+step, y+step, z,      level);
    branchBack        = new TsdSpaceBranch(leafs, x,      y,      z+step, level);
    branchBackRight   = new TsdSpaceBranch(leafs, x+step, y,      z+step, level);
    branchUpBack      = new TsdSpaceBranch(leafs, x,      y+step, z+step, level);
    branchUpBackRight = new TsdSpaceBranch(leafs, x+step, y+step, z+step, level);
  }

  _children.push_back(branch);
  _children.push_back(branchRight);
  _children.push_back(branchUp);
  _children.push_back(branchUpRight);
  _children.push_back(branchBack);
  _children.push_back(branchBackRight);
  _children.push_back(branchUpBack);
  _children.push_back(branchUpBackRight);

  // Calculate mean of centroids
  for(unsigned int i=0; i<_children.size(); i++)
  {
    double* c= _children[i]->getCentroid();
    _centroid[0] += c[0];
    _centroid[1] += c[1];
    _centroid[2] += c[2];
  }
  _centroid[0] /= 8.0;
  _centroid[1] /= 8.0;
  _centroid[2] /= 8.0;


  // Get outer bounds of leafs
  for(unsigned int i=0; i<_children.size(); i++)
  {
    (*_edgeCoordsHom)(i, 0) = (*(_children[i]->getEdgeCoordsHom()))(0,0);
    (*_edgeCoordsHom)(i, 1) = (*(_children[i]->getEdgeCoordsHom()))(0,1);
    (*_edgeCoordsHom)(i, 2) = (*(_children[i]->getEdgeCoordsHom()))(0,2);
    (*_edgeCoordsHom)(i, 3) = (*(_children[i]->getEdgeCoordsHom()))(0,3);
  }

  _componentSize = 2.0 * branch->getComponentSize();
  _circumradius = 2.0 * branch->getCircumradius();
}

TsdSpaceBranch::~TsdSpaceBranch()
{
  if(!_children[0]->isLeaf())
  {
    for(int i=0; i<8; i++)
      delete _children[i];
  }

  _children.clear();

  delete _edgeCoordsHom;
}

vector<TsdSpaceComponent*> TsdSpaceBranch::getChildren()
{
  return _children;
}

void TsdSpaceBranch::increaseEmptiness()
{
  for(int i=0; i<8; i++)
    _children[i]->increaseEmptiness();
}

static int level = 0;
void TsdSpaceBranch::print()
{
  for(int i=0; i<level; i++)
    cout << "   ";

  level++;

  double* c = getCentroid();
  cout << "(" << c[0] << " " << c[1] << ")" << endl;

  if(_children[0]->isLeaf())
  {
    for(int i=0; i<level; i++)
      cout << "   ";

    for(int i=0; i<8; i++)
    {
      double* c        = _children[i]->getCentroid();
      cout << "(" << c[0] << " " << c[1] << ") ";
    }
    cout << endl;
  }
  else
  {
    for(int i=0; i<8; i++)
      ((TsdSpaceBranch*)_children[i])->print();
  }

  level--;
}

void TsdSpaceBranch::printEdges()
{
  cout << "#" << level << endl;

  level++;

  Matrix* M        = getEdgeCoordsHom();
  M->print();

  if(_children[0]->isLeaf())
  {
    for(int i=0; i<8; i++)
    {
      Matrix* M        = _children[i]->getEdgeCoordsHom();
      M->print();
    }
    cout << endl;
  }
  else
  {
    for(int i=0; i<8; i++)
      ((TsdSpaceBranch*)_children[i])->printEdges();
  }

  level--;
}

}
