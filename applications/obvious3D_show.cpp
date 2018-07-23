#include "obgraphic/Obvious3D.h"
#include <vector>

using namespace obvious;
using namespace std;

int main(int argc, char *argv[])
{
  VtkCloud* cloud = NULL;

  /**
   * Try to determine file type by extension
   */
  if(argc>1)
    cloud = VtkCloud::load(argv[1], VTKCloud_AUTO);

  /**
   * If file was not specified or could not be loaded, show example
   */
  if(cloud == NULL)
    cloud = VtkCloud::createExample();

  Obvious3D* viewer = new Obvious3D();

  viewer->addCloud(cloud);
  viewer->startRendering();

  delete viewer;
  delete cloud;
}
