#include "obcore/base/CartesianCloud.h"
#include "obcore/base/CartesianCloudFactory.h"
#include "obgraphic/Obvious3D.h"

#include <sstream>

using namespace obvious;

int main(int argc, char *argv[])
{
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <mapfile> [inc]" << endl;
    cout << " inc (optional): incremental poses (default: 0)" << endl;
  }

  ifstream file;
  file.open(argv[1], ios::in);
  if(!file)
  {
    cout << "File not found: " << argv[1] << endl;
    return -1;
  }

  Obvious3D* viewer = new Obvious3D();

  bool inc = 0;
  if(argc==3)
    inc = atol(argv[2]);

  Matrix T(4, 4);
  T.identity();

  int cnt = 0;

  while (1)
  {
    char filename[1024];
    file.getline(filename, 1024);
    if(!file.good()) break;

    cout << filename << endl;

    CartesianCloud3D* cloud = CartesianCloudFactory::load(filename, eFormatAscii);
    cloud->removeInvalidPoints();

    double Tdata[16];
    Matrix Tcur(4, 4);
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    for(int i=0; i<16; i++)
      iss >> Tdata[i];
    Tcur.setData(Tdata);
    if(inc)
      T = T * Tcur;
    else
      T = Tcur;
    cloud->transform(&T);
    gsl_matrix* coords = cloud->getCoords();
    VtkCloud* vcloud = new VtkCloud();
    vcloud->setCoords(coords->data, coords->size1, 3, NULL);
    vcloud->setColors(cloud->getColors(), coords->size1, 3);

    viewer->addCloud(vcloud);
  }

  viewer->startRendering();
}
