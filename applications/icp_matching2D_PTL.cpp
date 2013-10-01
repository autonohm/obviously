/**
 * Sample application showing the usage of the OBVIOUS ICP implementation.
 * @author Stefan May
 * @date 05.05.2011
 */

#include <string.h>
#include <iostream>
#include <math.h>
#include "obvision/icp/icp_def.h"

using namespace std;
using namespace obvious;

int main(int argc, char** argv)
{
  //double dist_between_points = 0.1;
  unsigned int nr_of_points = 4;

//  double mdata[nr_of_points * 2];
//  double mnorm[nr_of_points * 2];
//
//  for (unsigned int i = 0; i < nr_of_points; i++)
//  {
//    if (i < 5)
//    {
//      mdata[2 * i] = i * dist_between_points;
//      mdata[2 * i + 1] = 1.2;
//      mnorm[2 * i] = 0.0;
//      mnorm[2 * i + 1] = -1.0;
//    }
//    else
//    {
//      mdata[2 * i] = 3.0;
//      mdata[2 * i + 1] = 1.2 - i * dist_between_points;
//      mnorm[2 * i] = -1.0;
//      mnorm[2 * i + 1] = 0.0;
//    }
//  }

  double mdata[] = { 0.0, 1.0,
                     0.0, 0.0,
                     1.0, 0.0,
                     2.0, 0.0 };
  double mnorm[] = { 0.0, 1.0,
                     0.0, 1.0,
                     sqrt(2), sqrt(2),
                     1.0, 0.0};

  double sdata[nr_of_points * 2];
  double snorm[nr_of_points * 2];

//  double T[] = { 0.98769, -0.1564, -0.0,
//                 0.1564,  0.98769, -0.0,
//                 0.0,     0.0,     1.0 };

  double T[] = { 0.9396926208, -0.3420201433, -0.3,
                 0.3420201433,  0.9396926208, -0.2,
                 0.0,     0.0,     1.0 };

//  double T[] = { 1.0,   0.0, -0.4,
//                 0.0,   1.0, -0.35,
//                 0.0,   0.0,  1.0 };


  for (unsigned int i = 0; i < nr_of_points; i++) {
    sdata[2 * i]     = mdata[2 * i] * T[0] + mdata[2 * i + 1] * T[1] + T[2];
    sdata[2 * i + 1] = mdata[2 * i] * T[3] + mdata[2 * i + 1] * T[4] + T[5];
    snorm[2 * i]     = mnorm[2 * i] * T[0] + mnorm[2 * i + 1] * T[1] + T[2];
    snorm[2 * i + 1] = mnorm[2 * i] * T[3] + mnorm[2 * i + 1] * T[4] + T[5];
  }

  gsl_matrix_view model       = gsl_matrix_view_array(mdata, nr_of_points, 2);
  gsl_matrix_view modelNormal = gsl_matrix_view_array(mnorm, nr_of_points, 2);
  gsl_matrix_view scene       = gsl_matrix_view_array(sdata, nr_of_points, 2);
  gsl_matrix_view sceneNormal = gsl_matrix_view_array(snorm, nr_of_points, 2);

  /**
   * Compose ICP modules
   */
  int iterations = 12;
  PairAssignment* assigner = (PairAssignment*)new AnnPairAssignment(2);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(1.5, 0.01, iterations);
  assigner->addPostFilter(filterD);

  IRigidEstimator* estimator = (IRigidEstimator*)new PointToLine2DEstimator();
//  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPointEstimator3D();


  Icp* icp = new Icp(assigner, estimator);
  icp->setModel(&model.matrix, &modelNormal.matrix);
  icp->setScene(&scene.matrix, &sceneNormal.matrix);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);

  double rms;
  unsigned int pairs;
  unsigned int it;
  icp->iterate(&rms, &pairs, &it);
  Matrix* F = icp->getFinalTransformation4x4();

  gsl_matrix_fprintf(stdout, F->getBuffer(), "%f");
  std::cout << "Error: " << estimator->getRMS() << std::endl;
  std::cout << "Iterations: " << estimator->getIterations() << std::endl;

  delete icp;
  delete estimator;
  delete assigner;

  return 0;
}
