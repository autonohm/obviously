/**
 * Sample application showing the usage of the OBVIOUS ICP implementation.
 * @author Stefan May
 * @date 05.05.2011
 */

#include <string.h>
#include <iostream>

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Timer.h"
#include "obvision/icp/icp_def.h"
//#include "obvision/icp/assign/NaboPairAssignment.h"

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  Timer timer;
  /*obvious::Matrix M(4, 2);
  M(0,0) = 0.0;  M(0,1) = 1.0;
  M(1,0) = 0.0;  M(1,1) = 0.0;
  M(2,0) = 1.0;  M(2,1) = 0.0;
  M(3,0) = 2.0;  M(3,1) = 0.0;*/

  obvious::Matrix M(1000, 2);
  for(int i=0; i<1000; i++)
  {
    M(i,0) = ((double)(rand()%100))/100.0; M(i,1) = ((double)(rand()%100))/100.0;
  }

  obvious::Matrix* T = MatrixFactory::TransformationMatrix33(deg2rad(9.0), 0.4, 0.35);
  obvious::Matrix S = M.createTransform(*T);

  /**
   * Compose ICP modules
   */
  int iterations                 = 30;
  //PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
  PairAssignment* assigner       = (PairAssignment*)  new FlannPairAssignment(2);
  //PairAssignment* assigner       = (PairAssignment*)  new NaboPairAssignment(2);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(1.5, 0.01, iterations);
  assigner->addPostFilter(filterD);
  IRigidEstimator* estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();

  Icp* icp = new Icp(assigner, estimator);
  icp->setModel(&M, NULL);
  icp->setScene(&S, NULL);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);

  double rms;
  unsigned int pairs;
  unsigned int it;
  icp->iterate(&rms, &pairs, &it);
  obvious::Matrix* F = icp->getFinalTransformation();
  F->invert();

  cout << "Applied transformation:" << endl;
  T->print();
  cout << endl << "Estimated transformation:" << endl;
  F->print();

  cout << endl << "Error: " << estimator->getRMS() << endl;
  cout << "Iterations: " << estimator->getIterations() << endl;

  delete icp;
  delete estimator;
  delete assigner;

  cout << "elapsed: " << timer.getTime() << " ms" << endl;
  return 0;
}
