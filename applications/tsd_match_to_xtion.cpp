/*
 * tsd_match_to_xtion.cpp
 *
 *  Created on: 07.10.2013
 *      Author: chris
 */



#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include "obvision/reconstruct/RayCast3D.h"
#include "obcore/base/Logger.h"
#include "obdevice/Xtion.h"
#include "obcore/math/mathbase.h"

using namespace std;
using namespace obvious;

Obvious3D* _viewer     = NULL;
VtkCloud* _cloudTsd    = NULL;
VtkCloud* _cloudXtion  = NULL;
Xtion*    _xtion       = NULL;
Sensor*   _sensor      = NULL;
bool      _pause       = false;
bool      _showNormals = false;

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
    if(!_pause)
    {
      if(_xtion->grab())
      {
        unsigned int cols = _xtion->getCols();
        unsigned int rows = _xtion->getRows();

        double* coords = _xtion->getCoords();
        // Convert to Euklidean distances
        double* dist = new double[cols*rows];
        for(unsigned int i=0; i<cols*rows; i++)
          dist[i] = abs3D(&coords[3*i]);

        // flix y coords
        for(unsigned int i=0 ; i<cols*rows ; i+=3)
          coords[i+1] = -coords[i+1];

        _sensor->setRealMeasurementData(dist);
        _sensor->setRealMeasurementMask(_xtion->getMask());
        _sensor->setRealMeasurementRGB(_xtion->getRGB());

        _cloudXtion->setCoords(_xtion->getCoords(), cols*rows, 3);
//        _cloudXtion->setColors(_xtion->getRGB(),    cols*rows, 3);
        double P[16];
        _sensor->getPose()->getData(P);
        _cloudXtion->transform(P);
        _viewer->update();
      }
      else
      {
        LOGMSG(DBG_WARN, "Can't grab Xtion.");
      }
    }
  }
private:
};

// --------------- Callback functions -------------------
void _upPressed(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  Matrix T(4, 4);
  double tf3[16] ={1,  0, 0, 0,
                   0,  1, 0, 0,
                   0,  0, 1, 0.1,
                   0,  0, 0, 1  };
  T.setData(tf3);
  _sensor->transform(&T);
}
void _downPressed(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  Matrix T(4, 4);
  double tf3[16] ={1,  0, 0, 0,
                   0,  1, 0, 0,
                   0,  0, 1, -0.1,
                   0,  0, 0, 1  };
  T.setData(tf3);
  _sensor->transform(&T);
}
void _leftPressed(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  Matrix T(4, 4);
  double alpha = 0.1;
  double tf3[16] ={cos(alpha),  0, sin(alpha),  0,
                   0,           1,       0,     0,
                   -sin(alpha), 0, cos(alpha),  0,
                   0,  0,       0,              1 };
  T.setData(tf3);
  _sensor->transform(&T);
}
void _rightPressed(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  Matrix T(4, 4);
  double alpha = -0.1;
  double tf3[16] ={cos(alpha),  0, sin(alpha),  0,
                   0,           1,       0,     0,
                   -sin(alpha), 0, cos(alpha),  0,
                   0,  0,       0,              1 };
  T.setData(tf3);
  _sensor->transform(&T);
}

// --------------- main -------------------------------
int main(int argc, char* argv[])
{
  // small error handling
  if(argc!=3)
  {
    cout << "usage: " << argv[1] << " <tsd_file>"         << endl;
    cout << "usage: " << argv[2] << " <xtionConfig.xml>" << endl;
    return -1;
  }
  // configuration of space
  ///@todo set up method for tsd configuration
  double height    = 6.0;
  double width     = 6.0;
  double depth     = 4.0;
  double voxelSize = 0.04;
  TsdSpace* _space = new TsdSpace(height, width, depth, voxelSize);

  // set up sensor
  double perspective[12]  = {585.05108211, 0.00000000, 315.83800193,
                             0., 0.00000000, 585.05108211, 242.94140713,
                             0., 0.00000000, 0.00000000, 1.00000000, 0.};
  _xtion  = new Xtion(argv[2]);

  if(!_xtion->grab())
  {
    LOGMSG(DBG_ERROR, "Error grabbing first image!");
    delete _xtion;
    exit(1);
  }

  // Dismiss first images in order to clear buffer
  // ------------------------------------------------------------------
  unsigned int pre = 0;
  while(pre<5)
  {
    if(_xtion->grab()) pre++;
  }

  _sensor = new SensorProjective3D(_xtion->getCols(), _xtion->getRows(), perspective, voxelSize);
  double tf[16]={1, 0, 0, 3,
                 0, 1, 0, 3,
                 0, 0, 1, 2,
                 0, 0, 0, 1};
  Matrix T(4, 4);
  T.setData(tf);
  _sensor->transform(&T);

  // rotate around x
  double tf2[16] ={ 1,  0,  0, 0,
                    0,  0, -1, 0,
                    0,  1,  0, 0,
                    0,  0,  0, 1};
  T.setData(tf2);
  _sensor->transform(&T);

  // rotate around z
  double tf3[16] ={0, -1, 0, 0,
                   1,  0, 0, 0,
                   0,  0, 1, 0,
                   0,  0, 0, 1};
  T.setData(tf3);
  _sensor->transform(&T);

  T.setData(tf3);
  _sensor->transform(&T);

  // init viewer
  _viewer = new Obvious3D((char*) "TSD Space Viewer", 1024, 768, 0, 0);
  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);

  LOGMSG_CONF("tsd_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);


  // load space from file
  _space->load(argv[1]);

  // configure raycast
  unsigned int   size;
  double*        coords;
  double*        normals;
  unsigned char* rgb;
  RayCast3D raycaster(_space);
  raycaster.generatePointCloudPositive(&coords, &normals, &rgb, &size);
  std::cout << "Raycast returned: " << size << "points. " << std::endl;

  // set up cloud
  _cloudTsd   = new VtkCloud();
  _cloudXtion = new VtkCloud();
  _viewer->addCloud(_cloudTsd);
  _viewer->update();
  _viewer->addCloud(_cloudXtion);
  _cloudTsd->setCoords(coords,   size/3, 3);
  _cloudTsd->setNormals(normals, size/3, 3);

  // configure keyboard callbacks
  _viewer->registerKeyboardCallback("Up",    _upPressed);
  _viewer->registerKeyboardCallback("Down",  _downPressed);
  _viewer->registerKeyboardCallback("Left",  _leftPressed);
  _viewer->registerKeyboardCallback("Right", _rightPressed);
  _viewer->addAxisAlignedCube(0, width, 0, height, 0, depth);
  _viewer->showAxes();
  double P[16];
  _sensor->getPose()->getData(P);
  _viewer->showSensorPose(P);
  _viewer->startRendering();

  // collect garbage
  delete _viewer;
  delete _xtion;
  delete _sensor;
}
