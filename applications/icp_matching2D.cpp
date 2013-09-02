/**
 * Sample application showing the usage of the OBVIOUS ICP implementation.
 * @author Stefan May
 * @date 05.05.2011
 */

#include <string.h>
#include <iostream>

#include "obvision/icp/icp_def.h"

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  double mdata[] = { 0.0, 1.0,
                     0.0, 0.0,
                     1.0, 0.0,
                     2.0, 0.0 };
  double sdata[8];

  double T[] = { 0.98769, -0.1564, 0.4,
                 0.1564,  0.98769, 0.35,
                 0.0,     0.0,     1.0 };

  for(int i = 0; i<4; i++)
  {
    sdata[2*i]   = mdata[2*i] * T[0] + mdata[2*i+1] * T[1] + T[2];
    sdata[2*i+1] = mdata[2*i] * T[3] + mdata[2*i+1] * T[4] + T[5];
  }
//  for (unsigned int i=0 ; i<8 ; i++)
//    std::cout << sdata[i] << std::endl;

  gsl_matrix_view model = gsl_matrix_view_array(mdata, 4, 2);
  gsl_matrix_view scene = gsl_matrix_view_array(sdata, 4, 2);

  /**
   * Compose ICP modules
   */
  int iterations                 = 30;
  PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(1.5, 0.01, iterations);
  assigner->addPostFilter(filterD);
  IRigidEstimator* estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();

  Icp* icp = new Icp(assigner, estimator);
  icp->setModel(&model.matrix);
  icp->setScene(&scene.matrix);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);

  double rms;
  unsigned int pairs;
  unsigned int it;
  icp->iterate(&rms, &pairs, &it);
  Matrix* F = icp->getFinalTransformation();

  gsl_matrix_fprintf(stdout, F->getBuffer(), "%f");
  std::cout << "Error: " << estimator->getRMS() << std::endl;
  std::cout << "Iterations: " << estimator->getIterations() << std::endl;

  delete icp;
  delete estimator;
  delete assigner;

  return 0;
}
