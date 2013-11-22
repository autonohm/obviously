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
#include "obdevice/Kinect.h"
#include "obcore/math/mathbase.h"
#include "obvision/icp/icp_def.h"
#include "obcore/math/transformationbase.h"

using namespace std;
using namespace obvious;

Obvious3D* _viewer      = NULL;
Obvious3D* _icp_viewer  = NULL;

Icp* _icp               = NULL;
OutOfBoundsFilter3D*    _filterBounds = NULL;

TsdSpace* _space        = NULL;

VtkCloud* _cloudTsd     = NULL;
VtkCloud* _cloudXtion   = NULL;
VtkCloud* _cloudModel   = NULL;
VtkCloud* _cloudScene   = NULL;

Kinect*   _xtion        = NULL;
Sensor*   _sensor       = NULL;

bool      _pause        = false;
bool      _showNormals  = false;
bool      _rotationMode = false;

bool      _debug        = true;
double    _alpha        = 0.03;


void assignmentCallback(double** m, double** s, unsigned int size)
{
  ofstream fm;
  ofstream fs;
  fm.open("/tmp/model.txt");
  fs.open("/tmp/scene.txt");
  for(unsigned int i=0; i<size; i++)
  {
    fm << m[i][0] << " " << m[i][1] << " " << m[i][2] << endl;
    fs << s[i][0] << " " << s[i][1] << " " << s[i][2] << endl;
  }
  fm.close();
  fs.close();
  abort();
}

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

        // flix y coords for correct visualization in vtk
        for(unsigned int i=0 ; i<cols*rows*3 ; i+=3)
          coords[i] = -coords[i];

        _cloudXtion->setCoords(coords,              cols*rows, 3);
        _cloudXtion->setColors(_xtion->getRGB(),    cols*rows, 3);
        double P[16];
        _sensor->getPose()->getData(P);
        _cloudXtion->transform(P);
        _viewer->update();
        delete [] dist;
      }
      else
      {
        LOGMSG(DBG_WARN, "Can't grab Kinect.");
      }
    }
  }
private:
};

void updatePose(double* P)
{
  Matrix T(4, 4);
  T.setData(P);
  _sensor->transform(&T);
  _sensor->getPose()->getData(P);
  _viewer->showSensorPose(P);
}

// --------------- Callback functions -------------------

void _changeMode(void)
{
  if(_rotationMode) _rotationMode = false;
  else              _rotationMode = true;
}

void _upPressed(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    rotateXAxis(tf, _alpha);
  else
    translateZAxis(tf, _alpha);

  updatePose(tf);
  delete [] tf;
}

void _downPressed(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    rotateXAxis(tf, -_alpha);
  else
    translateZAxis(tf, -_alpha);

  updatePose(tf);
  delete [] tf;
}

void _leftPressed(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    rotateYAxis(tf, _alpha);
  else
    translateXAxis(tf, _alpha);

  updatePose(tf);
  delete [] tf;
}
void _rightPressed(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    rotateYAxis(tf, -_alpha);
  else
    translateXAxis(tf, -_alpha);

  updatePose(tf);
  delete [] tf;
}

void _moveUp(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    translateZAxis(tf, _alpha);
  else
    translateYAxis(tf, _alpha);
  updatePose(tf);
  delete [] tf;
}

void _moveDown(void)
{
  double* tf = new double[16];
  if (_rotationMode)
    translateZAxis(tf, -_alpha);
  else
    translateYAxis(tf, -_alpha);
  updatePose(tf);
  delete [] tf;
}

void _match(void)
{
  // get model from space
  RayCast3D raycaster(_space);
  unsigned int size;
  double* coords      = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  double* normals     = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  unsigned char* rgb  = new unsigned char[_sensor->getWidth() * _sensor->getHeight() * 3];
  double P[16];

  raycaster.calcCoordsFromCurrentPose(_sensor, coords, normals, rgb, &size);

  cout << "Received " << size/3 << " from TSD space" << endl;
  _cloudModel->setCoords(coords,   size/3, 3);
  _cloudModel->setNormals(normals, size/3, 3);
  _cloudModel->setColors(rgb,      size/3, 3);
  _cloudModel->serialize("/tmp/model.vtp");

  LOGMSG(DBG_DEBUG, "Current Transformation: ");
  _sensor->getPose()->print();
  _filterBounds->setPose(_sensor->getPose());

  /*
   * get sensor information
   */
   double* sceneCoords  = new double[_xtion->getCols()*_xtion->getRows() * 3];
   sceneCoords = _xtion->getCoords();

  _sensor->getPose()->getData(P);
  _cloudScene->setCoords(sceneCoords, _xtion->getCols()*_xtion->getRows(), 3);
  _cloudScene->setColors(_xtion->getRGB(), _xtion->getCols()*_xtion->getRows(), 3);
  _cloudScene->transform(P);
  _cloudScene->serialize("/tmp/scene.vtp");

  _icp->reset();
  _icp->setScene(sceneCoords, NULL, _xtion->getRows()*_xtion->getCols());
  _icp->setModel(coords, NULL, size/3);

  // Perform ICP registration
  double       rms        = 0;
  unsigned int pairs      = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  if(((state == ICP_SUCCESS)       && (rms < 0.1)) ||
     ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    Matrix* T = _icp->getFinalTransformation();
    T->print();
    _sensor->transform(T);
    _sensor->getPose()->getData(P);
    _viewer->showSensorPose(P);
    _space->push(_sensor);
  }
  else
  {
    LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);
  }
  delete [] normals;
  delete [] rgb;
  delete [] coords;
}

void _push(void)
{
  static unsigned int i = 0;
  std::cout << "Push to tsd space" << std::endl;

  double P[16];

  // Convert to Euklidean distances
  double* coordsxtion = _xtion->getCoords();
  double* dist   = new double[_xtion->getCols()*_xtion->getRows()];

  for(unsigned int j=0; j<_xtion->getCols()*_xtion->getRows(); j++)
    dist[j] = abs3D(&coordsxtion[3*j]);

  // update sensor
  _sensor->setRealMeasurementData(dist);
  _sensor->setRealMeasurementMask(_xtion->getMask());
  _sensor->setRealMeasurementRGB( _xtion->getRGB());
  _space->push(_sensor);

  // get model from space
  RayCast3D raycaster(_space);
  unsigned int size;
  double* coords      = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  double* normals     = new double       [_sensor->getWidth() * _sensor->getHeight() * 3];
  unsigned char* rgb  = new unsigned char[_sensor->getWidth() * _sensor->getHeight() * 3];

  raycaster.calcCoordsFromCurrentPose(_sensor, coords, normals, rgb, &size);

  _cloudTsd->setCoords( coords,  size/3, 3);
  _cloudTsd->setColors( rgb,     size/3, 3);
  _cloudTsd->setNormals(normals, size/3, 3);

  // update pose from sensor
  _sensor->getPose()->getData(P);
  _cloudTsd->transform(P);

  // save space to file
  std::stringstream filename;
  filename << "/tmp/tsd_matched_" << i++;
  _space->serialize(filename.str().c_str());
  std::cout << "Saved tsd_space to " << filename.str() << std::endl;
  delete [] dist;
  delete [] coords;
  delete [] rgb;
  delete [] normals;
}

// --------------- main -------------------------------
int main(int argc, char* argv[])
{
  // small error handling
  if(argc!=3)
  {
    cout << "usage: " << argv[1] << " <tsd_file>"        << endl;
    cout << "usage: " << argv[2] << " <xtionConfig.xml>" << endl;
    return -1;
  }
  // configuration of space
  ///@todo set up method for tsd configuration with load
  double height    = 2.0;
  double width     = 2.0;
  double depth     = 2.5;
  double voxelSize = 0.02;
  _space = new TsdSpace(height, width, depth, voxelSize);

  const unsigned int maxIterations = 44;
  const unsigned int subSampling   = 44;

  // set up sensor parameters
  double perspective[12]  = {585.05108211, 0.00000000, 315.83800193,
                             0., 0.00000000, 585.05108211, 242.94140713,
                             0., 0.00000000, 0.00000000, 1.00000000, 0.};
  _xtion = new Kinect(argv[2]);

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

  _sensor = new SensorProjective3D(_xtion->getCols(), _xtion->getRows(), perspective);

  double tf[16]={1, 0, 0, 1.0,
                 0, 1, 0, 1.0,
                 0, 0, 1, 0.8,
                 0, 0, 0, 1};

  Matrix T(4, 4);
  T.setData(tf);
  _sensor->transform(&T);

  // rotate around x for correct coordinate system of kinect
  double tf2[16] ={ 1,  0,  0, 0,
                    0,  0, -1, 0,
                    0,  1,  0, 0,
                    0,  0,  0, 1};
  T.setData(tf2);
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
  raycaster.calcCoordsAxisParallel(&coords, &normals, &rgb, &size);
  std::cout << "Raycast returned: " << size << "points. " << std::endl;

  // ICP configuration
  PairAssignment*  assigner  = (PairAssignment*) new FlannPairAssignment(3, 0.0, true);
  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPointEstimator3D();

  // Out-of-Bounds filter to remove measurements outside TSD space
  _filterBounds = new OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(), _space->getMinY(), _space->getMaxY(), _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&T);
  assigner->addPreFilter(_filterBounds);

  // Subsampling filter for accelartion of icp
  IPreAssignmentFilter* filterS = (IPreAssignmentFilter*) new SubsamplingFilter(subSampling);
  assigner->addPreFilter(filterS);

  // Decreasing threshhold filter
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.3, 0.01, maxIterations);
  assigner->addPostFilter(filterD);

  IPostAssignmentFilter* filterR = (IPostAssignmentFilter*)new ReciprocalFilter();
  assigner->addPostFilter(filterR);

  // Deactivate early termination
  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.00);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);
//  _icp->setAssignmentCallback(assignmentCallback);

  // set up cloud
  _cloudTsd   = new VtkCloud();
  _cloudXtion = new VtkCloud();

  _cloudModel = new VtkCloud();
  _cloudScene = new VtkCloud();

  _viewer->addCloud(_cloudTsd);
  _viewer->addCloud(_cloudXtion);
  _viewer->update();

  _cloudTsd->setCoords( coords,  size/3, 3);
  _cloudTsd->setColors( rgb,     size/3, 3);
  _cloudTsd->setNormals(normals, size/3, 3);

  // configure keyboard callbacks
  _viewer->registerKeyboardCallback("space", _changeMode);
  _viewer->registerKeyboardCallback("l",     _moveUp);
  _viewer->registerKeyboardCallback("j",     _moveDown);
  _viewer->registerKeyboardCallback("Up",    _upPressed);
  _viewer->registerKeyboardCallback("Down",  _downPressed);
  _viewer->registerKeyboardCallback("Left",  _leftPressed);
  _viewer->registerKeyboardCallback("Right", _rightPressed);
  _viewer->registerKeyboardCallback("m",     _match);
  _viewer->registerKeyboardCallback("t",     _push);

  std::cout << "############################################"        << std::endl;
  std::cout << "# space     ->      rotation / translation"          << std::endl;
  std::cout << "# m         ->      new registration "               << std::endl;
  std::cout << "# t         ->      push to grid from current pose"  << std::endl;
  std::cout << "############################################"        << std::endl;

  // init visualizer
//  _viewer->addAxisAlignedCube(0, width, 0, height, 0, depth);
  _viewer->showAxes();
  double P[16];
  _sensor->getPose()->getData(P);
  _viewer->showSensorPose(P);
  _viewer->startRendering();

  // collect garbage

  delete _cloudTsd;
  delete _cloudModel;
  delete _cloudScene;
  delete _cloudScene;
  delete _viewer;
  delete _xtion;
  delete _sensor;
}
