/**
 * Kinect fusion example
 * @date 23.8.2013
 * @author Philipp Koch, Stefan May
 */

#include <cstdio>

#include "obdevice/Kinect.h"
#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obvision/icp/icp_def.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obvision/mesh/TriangleMesh.h"
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include "obvision/reconstruct/RayCastProjective3D.h"

using namespace obvious;

#define X_DIM 1
#define Y_DIM 1
#define Z_DIM 1
#define VXLDIM 0.005

Matrix* _T;
Matrix _Tinit(4, 4);
Kinect* _kinect;
TsdSpace* _space;
RayCastProjective3D* _rayCaster;
SensorProjective3D* _sensor;
VtkCloud* _vModel;
VtkCloud* _vScene;
Obvious3D* _viewer3D;
Icp* _icp;
OutOfBoundsFilter3D* _filterBounds;

void _cbStoreModel(void)
{
	static unsigned int id=0;
	char path[40];
	std::sprintf(path,"model%04d.vtp",id);
	_vModel->serialize(path,VTKCloud_XML);
	id++;
}

void _cbStoreScene(void)
{
  static unsigned int id=0;
  char path[40];
  std::sprintf(path,"scene%04d.vtp",id);
  _vScene->serialize(path,VTKCloud_XML);
  id++;
}

void _cbBuildSliceViews(void)
{
	unsigned char* buffer = new unsigned char[_space->getXDimension()*_space->getYDimension()*3];
	for(unsigned int i=0; i<_space->getZDimension(); i++)
	{
		char path[128];
		std::sprintf(path, "/tmp/slice%04d.ppm", i);
		if((_space->buildSliceImage(i, buffer))!=true)
		  LOGMSG(DBG_ERROR, "Error building sliceImage #" << i);
		obvious::serializePPM(path, buffer, _space->getXDimension(), _space->getYDimension(), 0);
	}
	delete[] buffer;
}

void _cbGenPointCloud(void)
{
  double* cloud = NULL;
  double* normals = NULL;
  unsigned char* rgb=NULL;
  unsigned int size;

  if(!(_rayCaster->generatePointCloud(&cloud, &normals, &rgb, &size)))
  {
    LOGMSG(DBG_ERROR, "Error generating global point cloud");
    return;
  }

  LOGMSG(DBG_DEBUG, "Cloud generated with " << size << " points";);
  _vModel->setCoords(cloud, size/3, 3, normals);
  _vModel->setColors(rgb, size/3, 3 );

  _viewer3D->update();

  delete cloud;
  delete normals;
  delete rgb;
}

void _cbRegNewImage(void)
{
  obvious::Timer t;

  unsigned int cols = _kinect->getCols();
  unsigned int rows = _kinect->getRows();

  double* normals    = new double[cols * rows * 3];
  double* coords     = new double[cols * rows * 3];
  unsigned char* rgb = new unsigned char[cols * rows * 3];
  bool* mask         = new bool[cols * rows];

  unsigned int size = cols*rows*3;

  LOGMSG(DBG_DEBUG, "Current Transformation: ");
  _sensor->getPose()->print();
  _filterBounds->setPose(_sensor->getPose());

  // Extract model from TSDF space
  unsigned int subsamplingModel = 20;

  _rayCaster->calcCoordsFromCurrentView(coords, normals, rgb, &size, subsamplingModel);

  if(size==0)
  {
    delete[] coords;
    delete[] normals;
    delete[] rgb;
    delete[] mask;
    return;
  }

  //_rayCaster->calcCoordsFromCurrentViewMask(coords, normals, rgb, mask);
  //TriangleMesh* mesh       = new TriangleMesh(rows*cols);
  //mesh->createMeshFromOrganizedCloud(coords, rows, cols, rgb, mask);

  double timeIcpStart = t.getTime();

  // Filter model to ensure proper normal vectors
  _vModel->setCoords(coords, size / 3, 3, normals);
  _vModel->setColors(rgb, size / 3, 3);
  _vModel->removeInvalidPoints();
  _vModel->copyCoords(coords);
  _vModel->copyNormals(normals);
  size = _vModel->getSize();

  // Display model as mesh -> use calcCoordsFromCurrentViewMask above!!!
  /*double** mCoords         = mesh->getCoords();
  unsigned char** mRGB     = mesh->getRGB();
  unsigned int** mIndices  = mesh->getIndices();
  unsigned int points      = mesh->getNumberOfPoints();
  unsigned int triangles   = mesh->getNumberOfTriangles();
  _vModel->setTriangles(mCoords, mRGB, points, mIndices, triangles);*/

  // Transform model in order to display it in space coordinates
  double P[16];
  _sensor->getPose()->getData(P);
  _vModel->transform(P);

  _icp->reset();
  _icp->setModel(coords, normals, size);

  // Acquire scene image
  _kinect->grab();

  double* coordsScene     = _kinect->getCoords();
  bool* maskScene         = _kinect->getMask();

  // Subsample and filter scene
  unsigned int subsamplingScene = 20;
  unsigned int idx = 0;
  for(unsigned int i=0; i<cols*rows; i+=subsamplingScene)
  {
    if(maskScene[i])
    {
      coords[3*idx] = coordsScene[3*i];
      coords[3*idx+1] = coordsScene[3*i+1];
      coords[3*idx+2] = coordsScene[3*i+2];
      idx++;
    }
  }

  _icp->setScene(coords, NULL, idx);

  // Perform ICP registration
  double rms = 0;
  unsigned int pairs = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  LOGMSG(DBG_DEBUG, "Elapsed ICP: " << t.getTime()-timeIcpStart << "ms, state " << state << ", pairs: " << pairs << ", rms: " << rms);

  if(((state == ICP_SUCCESS) && (rms < 0.1)) || ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    Matrix* T = _icp->getFinalTransformation();
    T->print();
    _vScene->transform(T->getBuffer()->data);
    _sensor->transform(T);
    double* coords = _kinect->getCoords();
    double* dist = new double[cols*rows];
    for(unsigned int i=0; i<cols*rows; i++)
      dist[i] = abs3D(&coords[3*i]);
    _sensor->setRealMeasurementData(dist);
    _sensor->setRealMeasurementMask(_kinect->getMask());
    _sensor->setRealMeasurementRGB(_kinect->getRGB());
    _space->push(_sensor);
    delete[] dist;
  }
  else
    LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);

  _viewer3D->update();

  delete[] coords;
  delete[] normals;
  delete[] rgb;
  delete[] mask;

  //delete mesh;
  std::cout << __PRETTY_FUNCTION__ << ": time ellapsed = " << t.getTime() << " ms" << std::endl;
}

void _cbReset(void)
{
  _space->reset();
  _sensor->setPose(&_Tinit);
  _space->push(_sensor);
}

int main(void)
{
  LOGMSG_CONF("mapper3D.log", Logger::screen_on | Logger::file_off, DBG_DEBUG, DBG_DEBUG);

  // Projection matrix (needs to be determined by calibration)
  // ------------------------------------------------------------------
  double Pdata[12] = {585.05108211, 0.0, 315.83800193, 0.0, 0.0, 585.05108211, 242.94140713, 0., 0.0, 0.0, 1.0, 0.0};
  Matrix P(3, 4, Pdata);
  _kinect = new Kinect("kinect.xml");

  // Check access to kinect device
  // ------------------------------------------------------------------
  if((_kinect->grab()) != true)
  {
    LOGMSG(DBG_ERROR, "Error grabbing first image!");
    delete _kinect;
    exit(1);
  }

  // Dismiss first images in order to clear buffer
  // ------------------------------------------------------------------
  unsigned int pre = 0;
  while(pre<5)
  {
    if(_kinect->grab()) pre++;
  }

  unsigned int cols = _kinect->getCols();
  unsigned int rows = _kinect->getRows();

  // Initial transformation of sensor
  // ------------------------------------------------------------------
  double tx = X_DIM/2.0;
  double ty = Y_DIM/2.0;
  double tz = 0;
  double tf[16]={1,  0, 0, tx,
                 0,  1, 0, ty,
                 0,  0, 1, tz,
                 0,  0, 0, 1};
  _Tinit.setData(tf);

  _space = new TsdSpace(Y_DIM, X_DIM, Z_DIM, VXLDIM);
  _space->setMaxTruncation(2.0 * VXLDIM);

  // ICP configuration
  // ------------------------------------------------------------------
  unsigned int maxIterations = 35;

  PairAssignment* assigner = (PairAssignment*)new FlannPairAssignment(3, 0.0, true);
  //PairAssignment* assigner = (PairAssignment*)new ProjectivePairAssignment(Pdata, cols, rows);

  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPlaneEstimator3D();

  // Out-of-Bounds filter to remove measurements outside TSD space
  _filterBounds = new OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(), _space->getMinY(), _space->getMaxY(), _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&_Tinit);
  assigner->addPreFilter(_filterBounds);

  // Decreasing threshhold filter
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.5, 0.01, 10);
  assigner->addPostFilter(filterD);

  _icp = new Icp(assigner, estimator);

  // Deactivate early termination
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);
  // ------------------------------------------------------------------

  _sensor = new SensorProjective3D(cols, rows, Pdata, _space->getVoxelSize());
  _sensor->setPose(&_Tinit);

  // Push first data set at initial pose
  // ------------------------------------------------------------------
  double* coords = _kinect->getCoords();
  // Convert to Euklidean distances
  double* dist = new double[cols*rows];
  for(unsigned int i=0; i<cols*rows; i++)
    dist[i] = abs3D(&coords[3*i]);
  _sensor->setRealMeasurementData(dist);
  _sensor->setRealMeasurementMask(_kinect->getMask());
  _sensor->setRealMeasurementRGB(_kinect->getRGB());
  _space->push(_sensor);
  delete [] dist;

  _rayCaster = new RayCastProjective3D(cols, rows, _sensor, _space);

  // Displaying stuff
  // ------------------------------------------------------------------
  _vModel = new VtkCloud();
  _vScene = new VtkCloud();
  _viewer3D = new Obvious3D("3DMapper");
  _viewer3D->addCloud(_vModel);
  _viewer3D->addAxisAlignedCube(0, X_DIM, 0, Y_DIM, 0, Z_DIM);
  //_viewer3D->addCloud(_vScene);
  _viewer3D->registerKeyboardCallback("space", _cbRegNewImage);
  _viewer3D->registerKeyboardCallback("c", _cbGenPointCloud);
  _viewer3D->registerKeyboardCallback("v", _cbBuildSliceViews);
  _viewer3D->registerKeyboardCallback("m", _cbStoreModel);
  _viewer3D->registerKeyboardCallback("s", _cbStoreScene);
  _viewer3D->registerKeyboardCallback("i", _cbReset);
  _viewer3D->startRendering();

  delete _icp;
  delete _kinect;
  delete _vModel;
  delete _vScene;
  delete _viewer3D;
  delete _rayCaster;
  delete _space;
}
