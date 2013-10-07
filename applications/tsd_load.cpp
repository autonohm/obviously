#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include "obvision/reconstruct/RayCast3D.h"
#include "obvision/reconstruct/SensorPolar3D.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

using namespace std;
using namespace obvious;


Obvious3D* _viewer    = NULL;

class vtkTimerCallback : public vtkCommand
{
public:
  static vtkTimerCallback *New()
  {
    vtkTimerCallback *cb = new vtkTimerCallback;
    return cb;
  }

  virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,  void *vtkNotUsed(callData))
  {

  }

private:

};

int main(int argc, char* argv[])
{
  // small error handling
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <tsd_file>" << endl;
    return -1;
  }

  // init viewer
  _viewer = new Obvious3D((char*) "TSD Space Viewer", 1024, 768, 0, 0);
  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);

  LOGMSG_CONF("tsd_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);

  // configuration of space
  ///@todo set up method for tsd configuration
  double height    = 6.0;
  double width     = 6.0;
  double depth     = 4.0;
  double voxelSize = 0.04;
  TsdSpace* _space = new TsdSpace(height, width, depth, voxelSize);

  // load space from file
  _space->load(argv[1]);

  // configure raycast
  unsigned int   size;
//  double*        coords ;
//  double*        normals;
//  unsigned char* rgb;
  RayCast3D raycaster(_space);
  double thetaRes = deg2rad(0.25);
  double thetaMin = deg2rad(-135.0);
  double phiRes   = deg2rad(1.0);
  unsigned int _beams  = 1081;

  double tf[16]={1, 0, 0, 3,
                 0, 1, 0, 3,
                 0, 0, 1, 2,
                 0, 0, 0, 1};
  Matrix T(4, 4);
  T.setData(tf);


  SensorPolar3D* _sensor = new SensorPolar3D(_beams, thetaRes, thetaMin, phiRes);
  double* coords      = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  double* normals     = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  unsigned char* rgb  = new unsigned char[_sensor->getWidth() * _sensor->getHeight() * 3];
  _sensor->transform(&T);
//  raycaster.calcCoordsFromCurrentPose(_sensor, coords, normals, rgb, &size);

  raycaster.generatePointCloudPositive(&coords, &normals, &rgb, &size);
  std::cout << "Raycast returned: " << size << "points. " << std::endl;

  // set up cloud
  VtkCloud* _cloud = new VtkCloud();
  _viewer->addCloud(_cloud);
  _cloud->setCoords(coords,   size/3, 3);
  _cloud->setNormals(normals, size/3, 3);
  _viewer->update();

  _viewer->startRendering();

  // collect garbage
  delete _viewer;
}
