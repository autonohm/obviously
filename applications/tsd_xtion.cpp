/**
 * Kinect fusion example for Xtion
 * @date 30.1.2014
 * @author Stefan May
 */

#include <cstdio>

#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obvision/mesh/TriangleMesh.h"
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obdevice/OpenNiDevice.h"

#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/reconstruct/space/SensorProjective3D.h"
#include "obvision/reconstruct/space/SensorPolar3D.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"

#include "obcore/math/TransformationWatchdog.h"


using namespace obvious;

#define VXLDIM 0.005
#define LAYOUTPARTITION LAYOUT_8x8x8
#define LAYOUTSPACE LAYOUT_256x256x256

Matrix* _T;
Matrix _Tinit(4, 4);
OpenNiDevice _xtion;
TsdSpace* _space;
RayCast3D* _rayCaster;
SensorProjective3D* _sensor;
VtkCloud* _vModel;
VtkCloud* _vScene;
Obvious3D* _viewer3D;
Icp* _icp;
OutOfBoundsFilter3D* _filterBounds;

TransformationWatchdog _TFwatchdog;

bool _contiuousRegistration = false;

void _cbRegNewImage(void);


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
	if(_contiuousRegistration)
	  _cbRegNewImage();
  }

private:

};

void _cbContinuous(void)
{
	if(_contiuousRegistration)
		_contiuousRegistration = false;
	else
		_contiuousRegistration = true;
}

void _cbGenPointCloud(void)
{
  unsigned int maxSize = _space->getXDimension()*_space->getYDimension()*_space->getZDimension() / 6;
  double* cloud = new double[maxSize*3];
  double* normals = new double[maxSize*3];
  //unsigned char* rgb=NULL;
  unsigned int size;

  //RayCastAxisAligned3D rayCasterMap;
  //rayCasterMap.calcCoords(_space, cloud, normals, &size);

  SensorPolar3D sensor(2162, deg2rad(0.125), deg2rad(-135.0), 1440, 6.0, 0.3);
  sensor.transform(&_Tinit);
  _rayCaster->calcCoordsFromCurrentPose(_space, &sensor, cloud, normals, NULL, &size);


  LOGMSG(DBG_DEBUG, "Cloud generated with " << size << " points";);
  _vModel->setCoords(cloud, size/3, 3, normals);
  //_vModel->setColors(rgb, size/3, 3 );
  double P[16];
  _Tinit.getData(P);
  _vModel->transform(P);

  _viewer3D->update();

  delete cloud;
  delete normals;
  //delete rgb;
}

void _cbRegNewImage(void)
{
  obvious::Timer t;
  obvious::Timer tComplete;

  unsigned int cols = _xtion.width();
  unsigned int rows = _xtion.height();

  double* normals    = new double[cols * rows * 3];
  double* coords     = new double[cols * rows * 3];

  unsigned int size = cols*rows*3;

  LOGMSG(DBG_DEBUG, "Current Transformation: ");
  Matrix T = _sensor->getTransformation();
  T.print();

  _filterBounds->setPose(&T);

  // Extract model from TSDF space
  _rayCaster->calcCoordsFromCurrentPose(_space, _sensor, coords, normals, NULL, &size);

  if(size==0)
  {
    delete[] coords;
    delete[] normals;
    return;
  }

  t.start();
  tComplete.start();

  // Transform model in world coordinate system in order to display it
  _vModel->setCoords(coords, size / 3, 3, normals);
  double P[16];
  T.getData(P);
  _vModel->transform(P);

  _icp->reset();
  _icp->setModel(coords, normals, _vModel->getSize(), 0.1);
  cout << "Set Model: " << t.reset() << " s" << endl;

  // Acquire scene image
  //for(unsigned int i=0; i<5; i++)
  _xtion.grab();
  cout << "Grab: " << t.reset() << " s" << endl;

  vector<float> coordsScene     = _xtion.coords();
  bool* maskScene         = new bool[cols*rows];//_xtion.getMask();
  double* dist = new double[cols*rows];

  // Assort invalid scene points
  unsigned int idx = 0;
  for(unsigned int i=0; i<cols*rows; i++)
  {
    dist[i] = sqrt(coordsScene[3*i]*coordsScene[3*i]+coordsScene[3*i+1]*coordsScene[3*i+1]+coordsScene[3*i+2]*coordsScene[3*i+2]);
    if(dist[i]>10e-3) maskScene[i] = true;
    else maskScene[i] = false;

    if(maskScene[i])
    {
      coords[3*idx] = coordsScene[3*i];
      coords[3*idx+1] = coordsScene[3*i+1];
      coords[3*idx+2] = coordsScene[3*i+2];
      idx++;
    }
  }
  cout << idx << endl;
  cout << "Filter: " << t.reset() << " s" << endl;

  if(idx==0)
  {
    LOGMSG(DBG_ERROR, "Invalid scene");
    delete[] coords;
    delete[] normals;
    return;
  }

  //_vScene->setCoords(coords, idx, 3, normals);

  _icp->setScene(coords, NULL, idx, 0.04);
  cout << "Set Scene: " << t.reset() << " s" << endl;

  // Perform ICP registration
  double rms = 0;
  unsigned int pairs = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  LOGMSG(DBG_DEBUG, "Elapsed ICP: " << t.reset() << "ms, state " << state << ", pairs: " << pairs << ", rms: " << rms);

  if(((state == ICP_SUCCESS) && (rms < 0.1)) || ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    // Obtain scene-to-model registration
    cout << "Scene-to-model registration" << endl;
    Matrix T = _icp->getFinalTransformation();
    T.print();

    double Tdata[16];
    T.getData(Tdata);
    //_vScene->transform(Tdata);

    _sensor->transform(&T);
    cout << "Current sensor transformation" << endl;
    Matrix Tmp = _sensor->getTransformation();
    Tmp.print();

    if(_TFwatchdog.checkWatchdog(Tmp))
    {
			_viewer3D->showSensorPose(Tmp);
			_sensor->setRealMeasurementData(dist);
			_sensor->setRealMeasurementMask(maskScene);
			_space->push(_sensor);
    }
  }
  else
    LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);

  _viewer3D->update();

  delete[] coords;
  delete[] normals;
  delete[] maskScene;
  delete[] dist;

  LOGMSG(DBG_ERROR, ": time elapsed = " << tComplete.elapsed() << " s");
}

void _cbReset(void)
{
  _space->reset();
  _sensor->resetTransformation();
  _sensor->transform(&_Tinit);
  _space->push(_sensor);
}

int main(void)
{
  LOGMSG_CONF("mapper3D.log", Logger::screen_on | Logger::file_off, DBG_DEBUG, DBG_DEBUG);

  // Projection matrix (needs to be determined by calibration) (tx smaller leftward -> ty smaller -> upwards
  // ------------------------------------------------------------------
//  double Pdata[12] = {565.891785/2.0, 0.0, 310.214327/2.0, 0.0, 0.0, 571.655457/2.0, 246.672372/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double Pdata[12] = {561.065430/2, 0.0, 318.726308/2.0, 0.0,
  										0.0, 562.793762/2.0, 249.564153/2, 0.0,
  										0.0, 0.0, 1.0, 0.000000};

  //double Pdata[12] = {528.785767/2.0, 0.0, 319.327807/2.0, 0.0, 0.0, 533.362122/2.0, 240.647637/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  Matrix P(3, 4, Pdata);



  _xtion.init();
  // Check access to kinect device
  // ------------------------------------------------------------------
  if((_xtion.grab()) != true)
  {
    LOGMSG(DBG_ERROR, "Error grabbing first image!");
    exit(1);
  }

  // Dismiss first images in order to clear buffer
  // ------------------------------------------------------------------
  unsigned int pre = 0;
  while(pre<5)
  {
    if(_xtion.grab()) pre++;
  }

  unsigned int cols = _xtion.width();
  unsigned int rows = _xtion.height();

  _space = new TsdSpace(VXLDIM, LAYOUTPARTITION, LAYOUTSPACE);
  _space->setMaxTruncation(3.0 * VXLDIM);

  // Initial transformation of sensor
  // ------------------------------------------------------------------
  double tr[3];
  _space->getCentroid(tr);
  tr[2] = 0.0;
  double tf[16]={1,  0, 0, tr[0],
                 0,  1, 0, tr[1],
                 0,  0, 1, tr[2],
                 0,  0, 0, 1};
  _Tinit.setData(tf);

  _TFwatchdog.setInitTransformation(_Tinit);
  _TFwatchdog.setRotationThreshold(0.08);
  _TFwatchdog.setTranslationThreshold(0.03);

  // ICP configuration
  // ------------------------------------------------------------------
  unsigned int maxIterations = 25;

  PairAssignment* assigner = (PairAssignment*)new FlannPairAssignment(3, 0.0, true);
  //PairAssignment* assigner = (PairAssignment*)new AnnPairAssignment(3);
  //PairAssignment* assigner = (PairAssignment*)new ProjectivePairAssignment(Pdata, cols, rows);

  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPlaneEstimator3D();
  //IRigidEstimator* estimator = (IRigidEstimator*)new PointToPointEstimator3D();

  // Out-of-Bounds filter to remove measurements outside TSD space
  _filterBounds = new OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(), _space->getMinY(), _space->getMaxY(), _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&_Tinit);
  assigner->addPreFilter(_filterBounds);

  // Decreasing threshold filter
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.5, 0.01, maxIterations-3);
  IPostAssignmentFilter* filterR = (IPostAssignmentFilter*)new ReciprocalFilter();
  assigner->addPostFilter(filterD);
  assigner->addPostFilter(filterR);

  _icp = new Icp(assigner, estimator);

  // Deactivate early termination
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);
  // ------------------------------------------------------------------

  _sensor = new SensorProjective3D(cols, rows, Pdata, 4.0, 0.4);
  _sensor->transform(&_Tinit);

  cout << "Initial Pose" << endl;
  Matrix Tmp = _sensor->getTransformation();
  Tmp.print();

  // Push first data set at initial pose
  // ------------------------------------------------------------------
  vector<float> coords = _xtion.coords();
  // Convert to Euklidean distances
  double* dist = new double[cols*rows];
  bool* mask = new bool[cols*rows];
  for(unsigned int i=0; i<cols*rows; i++)
  {
    dist[i] = sqrt(coords[3*i]*coords[3*i]+coords[3*i+1]*coords[3*i+1]+coords[3*i+2]*coords[3*i+2]);
    if(dist[i]>10e-3) mask[i] = true;
    else mask[i] = false;
  }
  _sensor->setRealMeasurementData(dist);
  _sensor->setRealMeasurementMask(mask);
  _space->push(_sensor);
  delete [] dist;
  delete [] mask;

  _rayCaster = new RayCast3D();

  // Displaying stuff
  // ------------------------------------------------------------------
  _vModel = new VtkCloud();
  _vScene = new VtkCloud();
  _viewer3D = new Obvious3D("3DMapper");

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer3D->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);

  _viewer3D->addCloud(_vModel);
  _viewer3D->addAxisAlignedCube(0, _space->getMaxX(), 0, _space->getMaxY(), 0, _space->getMaxZ());
  _viewer3D->addCloud(_vScene);
  _viewer3D->registerKeyboardCallback("space", _cbRegNewImage, 	"Register new image");
  _viewer3D->registerKeyboardCallback("c", _cbGenPointCloud, 		"Generate point cloud");
  _viewer3D->registerKeyboardCallback("i", _cbReset, 						"Reset TSD space");
  _viewer3D->registerKeyboardCallback("g", _cbContinuous, 		  "Continuous Registration");
  _viewer3D->startRendering();

  delete _icp;
  delete _vModel;
  delete _vScene;
  delete _viewer3D;
  delete _rayCaster;
  delete _space;
}
