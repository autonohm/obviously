#ifndef TSDSPACEBRANCH_H
#define TSDSPACEBRANCH_H

#include "obvision/reconstruct/space/TsdSpaceComponent.h"
#include <vector>

namespace obvious
{

class TsdSpaceBranch : public TsdSpaceComponent
{
public:
  TsdSpaceBranch(TsdSpaceComponent**** leafs, int x, int y, int z, int level);

  virtual ~TsdSpaceBranch();

  std::vector<TsdSpaceComponent*> getChildren();

  virtual void increaseEmptiness();

  void print();

  void printEdges();

private:

  std::vector<TsdSpaceComponent*> _children;

};

}

#endif
