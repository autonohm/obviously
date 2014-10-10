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
#include "obvision/icp/IcpMultiInitIterator.h"
//#include "obvision/icp/assign/NaboPairAssignment.h"

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  int animationDelay = 10;

  if(argc>1)
    animationDelay = atoi(argv[1]);

  Timer timer;
  timer.start();

  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(100, 2);

  // Normals
  obvious::Matrix* N = new obvious::Matrix(100, 2);

  for(int i=0; i<100; i++)
  {
    double di = (double) i;
    (*M)(i,0) = sin(di/50.0);
    (*M)(i,1) = sin(di/10.0);
  }

  // compute mean of components build by left and right neighbors
  for(int i=1; i<99; i++)
  {
    double xleft  = (*M)(i, 0) - (*M)(i-1, 0);
    double xright = (*M)(i+1, 0) - (*M)(i, 0);
    double yleft  = (*M)(i, 1) - (*M)(i-1, 1);
    double yright = (*M)(i+1, 1) - (*M)(i, 1);

    // x-component of normal
    (*N)(i-1, 0) = -(yright+yleft)/2.0;

    // y-component of normal
    (*N)(i-1, 1) =  (xright+xleft)/2.0;
  }

  // left bound
  (*N)(0, 0) = -((*M)(1, 1) - (*M)(0, 1));
  (*N)(0, 1) =  (*M)(1, 0) - (*M)(0, 0);

  // right bound
  (*N)(99, 0) = -((*M)(99, 1) - (*M)(98, 1));
  (*N)(99, 1) =  (*M)(99, 0) - (*M)(98, 0);

  obvious::Matrix T = MatrixFactory::TransformationMatrix33(deg2rad(9.0), 0.4, 0.35);
  obvious::Matrix S = M->createTransform(T);

  /**
   * Compose ICP modules
   */
  int iterations                 = 100;
  //PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
  PairAssignment* assigner       = (PairAssignment*)  new FlannPairAssignment(2, 0.0, true);
  //PairAssignment* assigner       = (PairAssignment*)  new NaboPairAssignment(2);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(1.5, 0.01, iterations);
  assigner->addPostFilter(filterD);
  //IRigidEstimator* estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();
  IRigidEstimator* estimator     = (IRigidEstimator*) new PointToLine2DEstimator();

  Icp* icp = new Icp(assigner, estimator);
  icp->setModel(M, N);
  icp->setScene(&S, NULL);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);
  icp->activateTrace();

  double rms;
  unsigned int pairs;
  unsigned int it;
  //for(int i=0; i<1000; i++)
  icp->iterate(&rms, &pairs, &it);
  obvious::Matrix F = icp->getFinalTransformation();
  F.invert();
  cout << endl << "Error: " << estimator->getRMS() << endl;
  cout << "Iterations: " << estimator->getIterations() << endl;

  char folder[6] = "trace";
  icp->serializeTrace(folder, animationDelay);


  // Test with multiple initialization matrices
  /*vector<obvious::Matrix> vT;

  obvious::Matrix Tinit(4, 4);
  Tinit.setIdentity();
  vT.push_back(Tinit);

  Tinit = MatrixFactory::TransformationMatrix44(deg2rad(30), 0, 0, 0, 0, 0);
  vT.push_back(Tinit);

  Tinit = MatrixFactory::TransformationMatrix44(deg2rad(-30), 0, 0, 0, 0, 0);
  vT.push_back(Tinit);

  Tinit = MatrixFactory::TranslationMatrix44(1.0, 0, 0);
  vT.push_back(Tinit);

  Tinit = MatrixFactory::TranslationMatrix44(0, 1.0, 0);
  vT.push_back(Tinit);

  IcpMultiInitIterator multiIcp(vT);
  obvious::Matrix F = multiIcp.iterate(icp);*/

  cout << "Applied transformation:" << endl;
  T.print();
  cout << endl << "Estimated transformation:" << endl;
  F.print();

  delete icp;
  delete estimator;
  delete assigner;

  cout << "elapsed: " << timer.elapsed() << " s" << endl;
  return 0;
}
