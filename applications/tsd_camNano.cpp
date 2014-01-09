/**
* @file LaserTSDNode.cpp
* @autor christian
* @date  24.09.2013
*
*
*/
#include "obvision/reconstruct/space/SensorProjective3D.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/icp/icp_def.h"

#include "obvision/reconstruct/space/RayCast3D.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"
#include "obcore/base/tools.h"
#include "obcore/math/mathbase.h"
#include "obgraphic/Obvious3D.h"
#include "obdevice/CamNano.h"

using namespace obvious;
using namespace std;

Obvious3D* _viewer    = NULL;
CamNano*   _nano      = NULL;

VtkCloud*                        _cloud        = NULL;
VtkCloud*                        _cloudScene   = NULL;
VtkCloud*                        _liveSensor   = NULL;
obvious::SensorProjective3D*     _sensor       = NULL;
obvious::TsdSpace*               _space        = NULL;
obvious::Icp*                    _icp          = NULL;
obvious::OutOfBoundsFilter3D*    _filterBounds = NULL;
double*                         _dists        = NULL;
unsigned int                   _beams;

bool _push;
bool _reg;
bool _showSpace;

void calc(void);

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
    if(_nano->grab())
    {
      unsigned int cols = _nano->getCols();
      unsigned int rows = _nano->getRows();

      double* coords = _nano->getCoords();
      bool*   mask   = _nano->getMask();

      // Convert to Euklidean distances
      double* dist = new double[cols*rows];
      for(unsigned int i=0; i<cols*rows; i++)
        dist[i] = abs3D(&coords[3*i]);

      // flip y coords for correct visualization in vtk
      for(unsigned int i=0 ; i<cols*rows*3 ; i+=3)
      {
        coords[i]   = -coords[i];
        coords[i+1] = -coords[i+1];
      }

      _liveSensor->setCoords(_nano->getValidCoords(), _nano->getValidSize(), 3);
      double P[16];
      _sensor->getPose()->getData(P);
      _liveSensor->transform(P);
      _viewer->update();

      calc();

      delete [] dist;
    }
    else
    {
      LOGMSG(DBG_WARN, "Can't grab Kinect.");
    }
  }

private:

};

void _newReg(void)
{

  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _reg = true;
}


void _newPush(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _push = true;
}

void _showSpaceFromAxesRayCast(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  unsigned int   size;
  double*        coords;
  double*        normals;
  unsigned char* rgb;
  RayCast3D raycaster;

  raycaster.calcCoordsFromCurrentPose(_space, _sensor, coords, normals, rgb, &size);
  _cloud->setCoords(coords,   size/3, 3);
  _cloud->setNormals(normals, size/3, 3);
  _viewer->update();
}

void _clearSpace(void)
{
  std::cout << "Space cleared." << std::endl;
  _space->reset();
}

void _saveTsdToTmp(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  //_space->serialize("/tmp/tsd_save");
}


void init(void)
{
  const double voxelSize   = 0.001;
  double d_maxIterations   = 50;
  double d_subSampling     = 1.0; // 100% -> take every sample

  const unsigned int maxIterations = (unsigned int)(d_maxIterations);

  _cloud      = new VtkCloud();
  _cloudScene = new VtkCloud();
  _liveSensor = new VtkCloud();

  // TSD Space configuration
  _space = new TsdSpace(voxelSize, LAYOUT_8x8x8, LAYOUT_128x128x128);
  _space->setMaxTruncation(3.0*voxelSize);

  std::cout << "CamNano with " << _nano->getCols() << " x " << _nano->getRows() << std::endl;

  // configure sensor
  double Pdata[12] = {90.1, 0.0,  82.0, 0.0,
                      0.0,   90.1, 59.5, 0.0,
                      0.0,   0.0,  1.0,  0.0};
  _sensor = new SensorProjective3D(_nano->getCols(), _nano->getRows(), Pdata);
  _dists  = new double[_sensor->getRealMeasurementSize()];

  std::cout << "Size: " << _sensor->getRealMeasurementSize() << std::endl;

  // translation of sensor
  double tr[3];
  _space->getCentroid(tr);
  double tf[16]={1, 0, 0, tr[0],
                  0, 1, 0, tr[1],
                  0, 0, 1, tr[2],
                  0, 0, 0, 1};
  Matrix T(4, 4);
  T.setData(tf);
  _sensor->transform(&T);

  // ICP configuration
  PairAssignment*  assigner  = (PairAssignment*) new FlannPairAssignment(3, 0.0, true);
  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPlaneEstimator3D();

  // Out-of-Bounds filter to remove measurements outside TSD space
  _filterBounds = new OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(), _space->getMinY(), _space->getMaxY(), _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&T);
  assigner->addPreFilter(_filterBounds);

  // Subsampling filter for accelartion of icp
  IPreAssignmentFilter* filterS = (IPreAssignmentFilter*) new SubsamplingFilter(d_subSampling);
  assigner->addPreFilter(filterS);

  // Decreasing threshhold filter
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.10, 0.01, maxIterations);
  assigner->addPostFilter(filterD);

  // Deactivate early termination
  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.00);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);

  // configure viewer
  _cloud      = new VtkCloud();
  _cloudScene = new VtkCloud();
  _viewer->addCloud(_cloud);
  _viewer->addCloud(_liveSensor);
//  _viewer->addCloud(_cloudScene);

  double P[16];
  _sensor->getPose()->getData(P);
  _viewer->showSensorPose(P);
  _viewer->addAxisAlignedCube(0, _space->getMaxX(), 0, _space->getMaxY(), 0, _space->getMaxZ());
  _viewer->showAxes();

  // initialize flags
  _showSpace = false;
  _reg       = false;
  _push      = false;
}

void calc(void)
{
  obvious::Timer t;
  double P[16];

  // in case of new push or registration
  if(_push || _reg)
  {
    // update sensor
    _nano->grab();
    _sensor->setRealMeasurementData(_nano->getDistImage());
    _sensor->setRealMeasurementMask(_nano->getMask());
//    _sensor->setRealMeasurementAccuracy()                     --> what is this?

    // only push if not registration
    if(_push) _space->push(_sensor);
    _push = false;

    // get model from space
    RayCast3D raycaster;
    unsigned int size;

    double* coords      = new double        [_sensor->getWidth() * _sensor->getHeight() * 3];
    double* normals     = new double        [_sensor->getWidth() * _sensor->getHeight() * 3];
    unsigned char* rgb = new unsigned char[_sensor->getWidth() * _sensor->getHeight() * 3];

    raycaster.calcCoordsFromCurrentPose(_space, _sensor, coords, normals, rgb, &size);

    cout << "Received " << size/3 << " from TSD space" << endl;
    _cloud->setCoords(coords,   size/3, 3);
//    _cloud->setNormals(normals, size/3, 3);
    _cloud->serialize("model.vtp");

    // update pose from sensor
    _sensor->getPose()->getData(P);
    _cloud->transform(P);

    // registration
    if (_reg)
    {
      unsigned int cols = _nano->getCols();
      unsigned int rows = _nano->getRows();

      double* coordsT = _nano->getCoords();

      // flip y coords for correct visualization in vtk
      for(unsigned int i=0 ; i<cols*rows*3 ; i+=3)
      {
        coordsT[i]   = -coordsT[i];
        coordsT[i+1] = -coordsT[i+1];
      }


      _reg = false;
      LOGMSG(DBG_DEBUG, "Current Transformation: ");
      _sensor->getPose()->print();
      _filterBounds->setPose(_sensor->getPose());

//      _cloudScene->setCoords(_nano->getValidCoords(), _nano->getValidSize(), 3);
      _cloudScene->setCoords(coordsT, _nano->getCols()*_nano->getRows(), 3);
      _sensor->getPose()->getData(P);
      _cloudScene->transform(P);
      _cloudScene->serialize("scene.vtp");

      _icp->reset();
//
      _icp->setScene(coordsT, NULL, _nano->getCols()*_nano->getRows());
      _icp->setModel(coords, normals, size/3);

      // Perform ICP registration
      double       rms        = 0;
      unsigned int pairs      = 0;
      unsigned int iterations = 0;

      double timeIcpStart = t.getTime();
      EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
      LOGMSG(DBG_DEBUG, "Elapsed ICP: " << t.getTime()-timeIcpStart << "ms, state " << state << ", pairs: " << pairs << ", rms: " << rms <<", iterations: " << iterations);
      if(((state == ICP_SUCCESS)       && (rms < 0.1)) ||
         ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
      {
        Matrix* T = _icp->getFinalTransformation();
        T->print();
        _sensor->transform(T);
        _sensor->getPose()->getData(P);
        _viewer->showSensorPose(P);
//        _space->push(_sensor);
      }
      else
      {
        LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);
      }
    }
  delete [] coords;
  delete [] normals;
  delete [] rgb;
  }
  _viewer->update();
}


int main(int argc, char* argv[])
{
  _viewer = new Obvious3D((char*) "Hokuyo TSD space", 1024, 768, 0, 0);
  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);

  _nano = new CamNano();
  _nano->setIntegrationTime(400);

  init();

  _viewer->registerKeyboardCallback("space", _newPush);
  _viewer->registerKeyboardCallback("m",     _newReg);
  _viewer->registerKeyboardCallback("a",     _showSpaceFromAxesRayCast);
  _viewer->registerKeyboardCallback("c",     _saveTsdToTmp);
  _viewer->registerKeyboardCallback("x",     _clearSpace);

  std::cout << "#####################################" << std::endl;
  std::cout << "# space     ->      new push         " << std::endl;
  std::cout << "# m         ->      new registration " << std::endl;
  std::cout << "# h         ->      show whole space " << std::endl;
  std::cout << "# x         ->      clear space      " << std::endl;
  std::cout << "#####################################" << std::endl;

  _viewer->startRendering();

  delete    _sensor;
  delete    _space;
  delete    _icp;
  delete    _filterBounds;
  delete [] _dists;
  delete    _viewer;
}
