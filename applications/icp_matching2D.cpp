/**
 * Sample application showing the usage of the OBVIOUS ICP implementation.
 * @author Stefan May
 * @date 05.05.2011
 */

#include <string.h>
#include <iostream>

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Timer.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/registration/icp/IcpMultiInitIterator.h"
//#include "obvision/registration/icp/assign/NaboPairAssignment.h"

#define POINTS 100

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  Timer timer;
  timer.start();

  // Model coordinates
  obvious::Matrix* M = new obvious::Matrix(POINTS, 2);

  // Normals
  obvious::Matrix* N = new obvious::Matrix(POINTS, 2);

  for(int i=0; i<POINTS; i++)
  {
    double di = (double) i;
    (*M)(i,0) = sin(di/(double)(POINTS/2));
    (*M)(i,1) = sin(di/(double)(POINTS/5));
  }

  // compute mean of components build by left and right neighbors
  for(int i=1; i<POINTS-1; i++)
  {
    double xleft  = (*M)(i, 0)   - (*M)(i-1, 0);
    double xright = (*M)(i+1, 0) - (*M)(i, 0);
    double yleft  = (*M)(i, 1)   - (*M)(i-1, 1);
    double yright = (*M)(i+1, 1) - (*M)(i, 1);

    // x-component of normal
    (*N)(i-1, 0) = -(yright+yleft)/2.0;

    // y-component of normal
    (*N)(i-1, 1) =  (xright+xleft)/2.0;
  }

  // left bound
  (*N)(0, 0) = -((*M)(1, 1) - (*M)(0, 1));
  (*N)(0, 1) =   (*M)(1, 0) - (*M)(0, 0);

  // right bound
  (*N)(POINTS-1, 0) = -((*M)(POINTS-1, 1) - (*M)(POINTS-2, 1));
  (*N)(POINTS-1, 1) =   (*M)(POINTS-1, 0) - (*M)(POINTS-2, 0);

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

  icp->iterate(&rms, &pairs, &it);
  obvious::Matrix F = icp->getFinalTransformation();
  F.invert();
  cout << endl << "Error: " << estimator->getRMS() << endl;
  cout << "Iterations: " << it << endl;

  char folder[] = "/tmp/trace";
  icp->serializeTrace(folder);


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
