/**
 * Sample application showing the reprojection of Kinect data to depth image plane.
 * @author Stefan May
 * @date 22.04.2011
 */

#include <string.h>
#include <iostream>

#include "obcore/base/CartesianCloudFactory.h"
#include "obcore/base/tools.h"
#include "obcore/math/geometry.h"
#include "obgraphic/VtkCloud.h"

#include <gsl/gsl_blas.h>

using namespace obvious;


int main(int argc, char** argv)
{	

  if(argc != 4)
  {
    printf("usage: %s <cloud> <image.ppm> <mask.pbm>\n", argv[0]);
    exit(1);
  }

  //CartesianCloud3D* cloud = CartesianCloudFactory::load(argv[1], eFormatAscii);

  VtkCloud* vcloud        = VtkCloud::load(argv[1], VTKCloud_AUTO);
  vcloud->removeInvalidPoints();
  CartesianCloud3D* cloud = new CartesianCloud3D(vcloud->getSize(), true);
  vcloud->copyData(cloud->getCoords(), cloud->getNormals(), cloud->getColors());

  double perspective[12]  = {585.05108211, 0.00000000, 315.83800193, 0., 0.00000000, 585.05108211, 242.94140713, 0., 0.00000000, 0.00000000, 1.00000000, 0.};

  gsl_matrix* P = gsl_matrix_alloc(3,4);
  for(int r=0; r<3; r++)
    for(int c=0; c<4; c++)
      gsl_matrix_set(P, r, c, perspective[r*4+c]);

  unsigned char *buf = (unsigned char *) malloc(3 * 640 * 480 * sizeof(unsigned char));
  unsigned char *msk = (unsigned char *) malloc(640 * 480 * sizeof(unsigned char));
  cloud->createProjection(buf, msk, P, 640, 480);
  serializePPM(argv[2], buf, 640, 480);
  serializePBM(argv[3], msk, 640, 480);
  free(buf);
  free(msk);
  gsl_matrix_free(P);
	
}
