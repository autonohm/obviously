/**
 * Camboard nano fusion example
 * @date 24.1.2014
 * @author Christian Pfitzner, Stefan May
 */

#include <cstdio>
#include <pthread.h>

#include "obdevice/CamNano.h"
#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obvision/icp/icp_def.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obvision/mesh/TriangleMesh.h"
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/TransformationWatchdog.h"

#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/reconstruct/space/SensorProjective3D.h"
#include "obvision/reconstruct/space/SensorPolar3D.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"
//#include "obvision/reconstruct/space/RayCast"

using namespace obvious;

#define VXLDIM 0.004
#define TRUNCATION 10
#define LAYOUTPARTITION LAYOUT_8x8x8
//#define LAYOUTPARTITION LAYOUT_128x128x128
#define LAYOUTSPACE LAYOUT_256x256x256
//#define LAYOUTSPACE LAYOUT_512x512x512

Matrix* 		_T;
Matrix 			_Tinit(4, 4);
CamNano* 		_camNano;
TsdSpace* 	_space;
RayCast3D* 	_rayCaster;
SensorProjective3D* _sensor;
VtkCloud* 	_vModel;
VtkCloud* 	_vScene;
Obvious3D* 	_viewer3D;
Icp* 				_icp;
OutOfBoundsFilter3D* 		_filterBounds;
TransformationWatchdog 	_TFwatchdog;
NormalsEstimator* _nestimator;
bool _contiuousRegistration = false;
bool _thread_finished       = false;
bool _threat_running 				= true;

ofstream protocol;

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond = PTHREAD_COND_INITIALIZER;
int play 						 = 0;

double* coordsScene;      //= _camNano->getCoords();
bool* maskScene;          //= _camNano->getMask();



//PROTOTYPE
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

void* grabData(void* params);

void _cbSwitchContinuous(void)
{
	if(_contiuousRegistration)
		_contiuousRegistration = false;
	else
		_contiuousRegistration = true;
}

void _cbStoreModel(void)
{
	static unsigned int id=0;
	char path[40];
	std::sprintf(path, "model%04d.vtp",id);
	_vModel->serialize(path, VTKCloud_XML);
	id++;
}

void _cbStoreScene(void)
{
  static unsigned int id=0;
  char path[40];
  std::sprintf(path, "scene%04d.vtp",id);
  _vScene->serialize(path, VTKCloud_XML);
  id++;
}

void _cbBuildSliceViews(void)
{
	unsigned char* buffer = new unsigned char[_space->getXDimension()*_space->getYDimension()*3];
	for(unsigned int i=0; i<_space->getZDimension(); i++)
	{
		char path[128];
		std::sprintf(path, "/tmp/slice%04d.ppm", i);

		// not implemented yet
		//if((_space->buildSliceImage(i, buffer))!=true)
		LOGMSG(DBG_ERROR, "Error building sliceImage #" << i);
		return;

		obvious::serializePPM(path, buffer, _space->getXDimension(), _space->getYDimension(), 0);
	}
	delete[] buffer;
}

void _cbGenPointCloud(void)
{
  unsigned int maxSize = _space->getXDimension()*_space->getYDimension()*_space->getZDimension() / 6;
  double* cloud = new double[maxSize*3];
  double* normals = new double[maxSize*3];
  unsigned int size;

//  RayCastAxisAligned3D rayCasterMap;
//  rayCasterMap.calcCoords(_space, cloud, normals, &size);

  SensorPolar3D sensor(2162, deg2rad(0.125), deg2rad(-180.0), 1440, 4.0, 0.0);
  sensor.transform(&_Tinit);
  _rayCaster->calcCoordsFromCurrentPose(_space, &sensor, cloud, normals, NULL, &size);

  LOGMSG(DBG_DEBUG, "Cloud generated with " << size << " points";);
  _vModel->setCoords(cloud, size/3, 3, normals);

  _viewer3D->update();

  delete cloud;
  delete normals;
}


void _cbGenMesh(void)
{
  unsigned int cols = _camNano->getCols();
  unsigned int rows = _camNano->getRows();

  double* normals    = new double[cols * rows * 3];
  double* coords     = new double[cols * rows * 3];
  unsigned char* rgb = new unsigned char[cols * rows * 3];
  bool* mask         = new bool[cols * rows];

  unsigned int size = cols*rows*3;

  _rayCaster->calcCoordsFromCurrentPoseMask(_space, _sensor, coords, normals, NULL, mask, &size);

  TriangleMesh* mesh       = new TriangleMesh(rows*cols);
  mesh->createMeshFromOrganizedCloud(coords, rows, cols, NULL, mask);

  if(mesh->getNumberOfTriangles()==0)
  {
    delete[] coords;
    delete[] normals;
    delete[] rgb;
    delete[] mask;
    return;
  }

  for(unsigned int i=0 ; i<size ; i++)
  	rgb[i] = 255;


  std::cout << "what" << std::endl;

  // Filter model to ensure proper normal vectors
  _vModel->setCoords(coords, size / 3, 3, normals);
  _vModel->setColors(rgb, size / 3, 3);
  _vModel->removeInvalidPoints();
  _vModel->copyCoords(coords);
  _vModel->copyNormals(normals);

  // Display model as mesh -> use calcCoordsFromCurrentViewMask above!!!
  double** mCoords         = mesh->getCoords();
//  unsigned char** mRGB     = mesh->getRGB();
//  unsigned char** mRGB		 = new unsigned char[cols][rows];
  unsigned int** mIndices  = mesh->getIndices();
  unsigned int points      = mesh->getNumberOfPoints();
  unsigned int triangles   = mesh->getNumberOfTriangles();

  _vModel->setTriangles(mCoords, NULL, points, mIndices, triangles);

  // Transform model in order to display it in space coordinates
  double P[16];
  Matrix tmp = _sensor->getTransformation();
  tmp.getData(P);
  _vModel->transform(P);

  double lightPos[3];
  double lightLookAt[3];
  lightPos[0] = _space->getMaxX() / 2.0;
  lightPos[1] = _space->getMaxY() / 2.0;
  lightPos[2] = 0;
  lightLookAt[0] = _space->getMaxX() / 2.0;
  lightLookAt[1] = _space->getMaxY() / 2.0;
  lightLookAt[2] = _space->getMaxZ() / 2.0;

  _viewer3D->addLight(lightPos, lightLookAt);
  _viewer3D->update();

  delete[] coords;
  delete[] normals;
  delete[] rgb;
  delete[] mask;
  delete mesh;
}

void pushToSpace(void)
{
	double* coords = _camNano->getCoords();
	unsigned int cols = _camNano->getCols();
	unsigned int rows = _camNano->getRows();
	double* dist = new double[cols*rows];
	for(unsigned int i=0; i<cols*rows; i++)
		dist[i] = abs3D(&coords[3*i]);
	_sensor->setRealMeasurementData(dist);
	_sensor->setRealMeasurementMask(_camNano->getMask());
	_space->push(_sensor);
	delete[] dist;
}

void* grabData(void* params)
{
	while(1){
		_camNano->grab();
	  coordsScene     = _camNano->getCoords();
	  maskScene       = _camNano->getMask();
		_thread_finished = true;
//		pthread_exit(NULL);
	}
  return(0);
}

void _cbRegNewImage(void)
{
  obvious::Timer t;

  unsigned int cols = _camNano->getCols();
  unsigned int rows = _camNano->getRows();

  double * coordsScene_tmp = new double[cols*rows*3];
  bool*    maskScene_tmp   = new bool[cols*rows];

  memcpy(coordsScene_tmp, coordsScene, sizeof(double)*cols*rows*3);
  memcpy(maskScene_tmp,   maskScene,   sizeof(bool)*cols*rows);

  double* normals    = new double[cols * rows * 3];
  double* coords     = new double[cols * rows * 3];
  unsigned char* rgb = new unsigned char[cols * rows * 3];

  unsigned int size = cols*rows*3;

  LOGMSG(DBG_DEBUG, "Current Transformation: ");
  Matrix T = _sensor->getTransformation();
//  T.print();

  _filterBounds->setPose(&T);

  // Extract model from TSDF space
  _rayCaster->calcCoordsFromCurrentPose(_space, _sensor, coords, normals, rgb, &size);

  protocol << t.getTime() << "; ";
  if(size==0)
  {
    LOGMSG(DBG_ERROR, "Not enough points after raycasting");
    delete[] coords;
    delete[] normals;
    delete[] rgb;
    return;
  }

  double timeIcpStart = t.getTime();

  // Filter model to ensure proper normal vectors
  _vModel->setCoords(coords, size / 3, 3, normals);
  _vModel->removeInvalidPoints();
  _vModel->copyCoords(coords);
  _vModel->copyNormals(normals);

  // Transform model in world coordinate system in order to display it
  double P[16];
  T.getData(P);
  _vModel->transform(P);

  _icp->reset();
  _icp->setModel(coords, normals, _vModel->getSize(), 0.1);
  cout << "Set Model: " << t.getTime() - timeIcpStart << "ms" << endl;
  //_icp->setModel(coords, normals, size, 0.2);

  // Acquire scene image
//  _camNano->grab();
//  cout << "Grab: " << t.getTime() - timeIcpStart << "ms" << endl;

//  double* coordsScene     = _camNano->getCoords();
//  bool* maskScene         = _camNano->getMask();

  // Assort invalid scene points
  unsigned int idx = 0;
  for(unsigned int i=0; i<cols*rows; i++)
  {
    if(maskScene[i])
    {
      coords[3*idx] = -coordsScene[3*i];
      coords[3*idx+1] = -coordsScene[3*i+1];
      coords[3*idx+2] = coordsScene[3*i+2];
      idx++;
    }
  }
  cout << "Filter: " << t.getTime() - timeIcpStart << "ms" << endl;

  if(idx==0)
  {
    LOGMSG(DBG_ERROR, "Invalid scene");
    delete[] coords;
    delete[] normals;
    delete[] rgb;
    return;
  }

  bool _useNormals = true;
  if(_useNormals)
  {
  	_nestimator->estimateNormals3DGrid(cols, rows, coords, maskScene, normals);
  	_icp->setScene(coords, normals, idx, 0.04);
  }
  else
  {
  	_icp->setScene(coords, NULL, idx, 0.04);
  }
  cout << "Set Scene: " << t.getTime() - timeIcpStart << "ms" << endl;

  // Perform ICP registration
  double rms = 0;
  unsigned int pairs = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  LOGMSG(DBG_DEBUG, "Elapsed ICP: " << t.getTime()-timeIcpStart << "ms, state " << state << ", pairs: " << pairs << ", rms: " << rms);
  protocol << t.getTime()-timeIcpStart << "; ";

  if(((state == ICP_SUCCESS) && (rms < 0.1)) || ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    // Obtain scene-to-model registration
    cout << "Scene-to-model registration" << endl;
    Matrix T = *(_icp->getFinalTransformation());
//    T.print();

    _sensor->transform(&T);

    cout << "Current sensor transformation" << endl;
    Matrix Tmp = _sensor->getTransformation();
//    Tmp.print();
    _viewer3D->showSensorPose(Tmp);

    // check if transformation is big enough to push
    if(_TFwatchdog.checkWatchdog(Tmp))
    {
    	double startPush = t.getTime();
    	pushToSpace();
    	protocol << t.getTime() - startPush << "; ";
    }
    else
    {
    	protocol << "0.0" << "; ";
    }
  }
  else
    LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);

  _viewer3D->showSensorPose(P);
  _viewer3D->update();

  delete[] coords;
  delete[] normals;
  delete[] rgb;

  LOGMSG(DBG_ERROR, ": time elapsed = " << t.getTime() << " ms");
  protocol << t.getTime() << "; " << std::endl;
  LOGMSG(DBG_DEBUG, ": frame rate = " << 1/t.getTime()*1000 << " FPS");
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
  LOGMSG_CONF("mapper3D.log", Logger::screen_off | Logger::file_off, DBG_DEBUG, DBG_DEBUG);

  protocol.open("/tmp/protocol.csv");

  protocol << "Time Raycasting; Time ICP; Time Push; Total" << std::endl;

  // Projection matrix (needs to be determined by calibration) (tx smaller leftward -> ty smaller -> upwards
  // ------------------------------------------------------------------
  double Pdata[12] = {90.1, 0.0,  82.0, 0.0,
                      0.0,   90.1, 59.5, 0.0,
                      0.0,   0.0,  1.0,  0.0};

  Matrix P(3, 4, Pdata);
  _camNano = new CamNano();
  _camNano->setIntegrationAuto();
  _camNano->activeBilinearFilter(true);



  // Check access to cam nano device
  // ------------------------------------------------------------------
  if((_camNano->grab()) != true)
  {
    LOGMSG(DBG_ERROR, "Error grabbing first image!");
    delete _camNano;
    exit(1);
  }

  unsigned int cols = _camNano->getCols();
  unsigned int rows = _camNano->getRows();

  _space = new TsdSpace(VXLDIM, LAYOUTPARTITION, LAYOUTSPACE);
  _space->setMaxTruncation(TRUNCATION * VXLDIM);

  // Initial transformation of sensor
  // ------------------------------------------------------------------
  double tr[3];
  _space->getCentroid(tr);
//  tr[2] = 0.0;
  double tf[16]={1,  0, 0, tr[0],
                 0,  1, 0, tr[1],
                 0,  0, 1, tr[2]/*/1.5*/,
                 0,  0, 0, 1};
  _Tinit.setData(tf);

  // ICP configuration
  // ------------------------------------------------------------------
  unsigned int maxIterations = 5;

  PairAssignment* assigner = (PairAssignment*)new FlannPairAssignment(3, 0.0, true);
  //PairAssignment* assigner = (PairAssignment*)new AnnPairAssignment(3);
//  PairAssignment* assigner = (PairAssignment*)new ProjectivePairAssignment(Pdata, cols, rows);

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

  _sensor = new SensorProjective3D(cols, rows, Pdata, 2.0, 0.1);
  _sensor->transform(&_Tinit);

  cout << "Initial Pose" << endl;
  Matrix Tmp = _sensor->getTransformation();
  Tmp.print();

  _nestimator = new NormalsEstimator();

  _TFwatchdog.setInitTransformation(Tmp);
  _TFwatchdog.setRotationThreshold(0.08);
  _TFwatchdog.setTranslationThreshold(0.03);

  // Push first data set at initial pose
  // ------------------------------------------------------------------
  _camNano->grab();
  double* coords = _camNano->getCoords();
  // Convert to Euklidean distances
  double* dist = new double[cols*rows];
  for(unsigned int i=0; i<cols*rows; i++)
      dist[i] = abs3D(&coords[3*i]);

  _sensor->setRealMeasurementData(dist);
  _sensor->setRealMeasurementMask(_camNano->getMask());
  _space->push(_sensor);
  delete [] dist;

  _rayCaster = new RayCast3D();

  // create publisher thread
  pthread_t handle;
  int ret = pthread_create(&handle, NULL, grabData, NULL);
  if(ret != 0)
  {
		printf("Error: pthread_create() failed\n");
		exit(EXIT_FAILURE);
  }

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
  _viewer3D->registerKeyboardCallback("space", _cbRegNewImage, "Register new image");
  _viewer3D->registerKeyboardCallback("c", _cbGenPointCloud, "Generate point cloud");
  _viewer3D->registerKeyboardCallback("d", _cbGenMesh, "Generate mesh");
  _viewer3D->registerKeyboardCallback("v", _cbBuildSliceViews, "Build slice views");
  _viewer3D->registerKeyboardCallback("m", _cbStoreModel, "Save model");
  _viewer3D->registerKeyboardCallback("n", _cbStoreScene, "Save scene");
  _viewer3D->registerKeyboardCallback("i", _cbReset, "Reset TSD space");
  _viewer3D->registerKeyboardCallback("g", _cbSwitchContinuous, "Continuous Registration");

  _viewer3D->showAxes();
  _viewer3D->startRendering();

  delete _icp;
  delete _camNano;
  delete _vModel;
  delete _vScene;
  delete _viewer3D;
  delete _rayCaster;
  delete _space;

  protocol.close();
}
