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
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"

#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include "obvision/reconstruct/RayCastProjective3D.h"

using namespace obvious;

#define X_DIM 2
#define Y_DIM 2
#define Z_DIM 2
#define VXLDIM 0.01

Matrix* _T;
Kinect* _kinect;
TsdSpace* _space;
RayCastProjective3D* _rayCaster;
SensorProjective3D* _sensor;
VtkCloud* _vModel;
VtkCloud* _vScene;
Obvious3D* _viewer3D;
Icp* _icp;
double* _dist;
bool _recON;                    //set to true to store all Kinect Clouds on disk


void StoreKinCloud(const unsigned int id,VtkCloud *kinVcloud)
{
	char path[40];
	std::sprintf(path,"kinClouds/Cloud%04d.vtp",id);
	kinVcloud->serialize(path,VTKCloud_XML);
}

void _cbStoreCurrentCloud(void)
{
	static unsigned int id=0;
	char path[40];
	std::sprintf(path,"curClouds/curCloud%04d.vtp",id);
	_vModel->serialize(path,VTKCloud_XML);
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
  obvious::Timer tNewImage;

  static int id=0;
  unsigned int cols = _kinect->getCols();
  unsigned int rows = _kinect->getRows();

  double* normals = new double[cols * rows * 3];
  double* coords = new double[cols * rows * 3];
  unsigned char* rgb = new unsigned char[cols * rows * 3];

  unsigned int size = 0;


  LOGMSG(DBG_DEBUG, "Current Transformation: ");
  _sensor->getPose()->print();

  // Extract model from TSDF space
  unsigned int subsamplingModel = 5;
  RayCastProjective3D rayCaster(cols, rows, _sensor, _space);
  rayCaster.calcCoordsFromCurrentView(coords, normals, rgb, &size, subsamplingModel);

  if(size == 0)
  {
    LOGMSG(DBG_ERROR, "RayCasting returned with no coordinates");
    return;
  }

  obvious::Timer t;

  // Filter model to ensure proper normal vectors
  _vModel->setCoords(coords, size / 3, 3, normals);
  _vModel->setColors(rgb, size / 3, 3);
  _vModel->removeInvalidPoints();
  _vModel->copyCoords(coords);
  _vModel->copyNormals(normals);
  size = _vModel->getSize();for(int i=0; i<cols*rows; i++)

  _icp->reset();
  _icp->setModel(coords, normals, size);

  // Acquire scene image
  _kinect->grab();

  // Estimate scene normal vectors
  NormalsEstimator nestimator;
  nestimator.estimateNormals3DGrid(cols, rows, _kinect->getCoords(), _kinect->getMask(), normals);

  // Filter scene to ensure proper normal vectors
  unsigned int subsamplingScene = 10;
  _vScene->setCoords(_kinect->getCoords(), cols * rows, 3, normals);
  _vScene->setColors(_kinect->getRGB(), cols * rows, 3);
  _vScene->removeInvalidPoints();
  _vScene->copyCoords(coords, subsamplingScene);
  _vScene->copyNormals(normals, subsamplingScene);
  size = _vScene->getSize();

  _icp->setScene(coords, normals, size/subsamplingScene);

  if(_recON)
  {
    StoreKinCloud(id,_vScene);
    id++;
  }


  // Perform ICP registration
  double rms = 0;
  unsigned int pairs = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  LOGMSG(DBG_DEBUG, "Elapsed ICP: " << t.getTime() << "ms, state " << state << ", pairs: " << pairs << ", rms: " << rms);

  if(((state == ICP_SUCCESS) && (rms < 0.1)) || ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    Matrix* T = _icp->getFinalTransformation();
    T->print();
    _vScene->transform(T->getBuffer()->data);
    _sensor->transform(T);
    double* coords = _kinect->getCoords();
    for(int i=0; i<cols*rows; i++)
      _dist[i] = sqrt(coords[3*i]*coords[3*i]+coords[3*i+1]*coords[3*i+1]+coords[3*i+2]*coords[3*i+2]);
    _sensor->setRealMeasurementData(_dist);
    _sensor->setRealMeasurementMask(_kinect->getMask());
    _sensor->setRealMeasurementRGB(_kinect->getRGB());
    _space->push(_sensor);
  }
  else
    LOGMSG(DBG_DEBUG, "RMS " << rms);

  _viewer3D->update();

  delete[] coords;
  delete[] normals;
  delete[] rgb;
  std::cout << __PRETTY_FUNCTION__ << ": time ellapsed = " << tNewImage.getTime() << " ms" << std::endl;
}

int main(void)
{
	_recON=0;           //initialize with no recording

  LOGMSG_CONF("mapper3D.log", Logger::screen_on | Logger::file_off, DBG_DEBUG, DBG_DEBUG);

  double Pdata[12] = {585.05108211, 0.0, 315.83800193, 0.0, 0.0, 585.05108211, 242.94140713, 0., 0.0, 0.0, 1.0, 0.0};
  Matrix P(3, 4, Pdata);
  _kinect = new Kinect("kinect.xml");

  if((_kinect->grab()) != true)
  {
    LOGMSG(DBG_ERROR, "Error grabbing first image!");
    delete _kinect;
    exit(1);
  }

  unsigned int cols = _kinect->getCols();
  unsigned int rows = _kinect->getRows();

  _dist = new double[cols*rows];

  PairAssignment* assigner = (PairAssignment*)new FlannPairAssignment(3, 0.0);
//  PairAssignment* assigner = (PairAssignment*)new ProjectivePairAssignment(Pdata, cols, rows);
  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPlaneEstimator3D();
  IPreAssignmentFilter* filterS = (IPreAssignmentFilter*)new SubsamplingFilter(50);
  //assigner->addPreFilter(filterS);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.25, 0.05, 20);
  assigner->addPostFilter(filterD);
  //ProjectionFilter* filterP = new ProjectionFilter(P, cols, rows);
  //assigner->addPreFilter(filterP);
  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(25);

  _space = new TsdSpace(Y_DIM, X_DIM, Z_DIM, VXLDIM);
  _space->setMaxTruncation(2.0 * VXLDIM);

  // translation of sensor
  double tx = X_DIM/2.0;
  double ty = Y_DIM/2.0;
  double tz = 0.1;

  double tf[16]={1,  0, 0, tx,
                 0,  1, 0, ty,
                 0,  0, 1, tz,
                 0,  0, 0, 1};
  Matrix Tinit(4, 4);
  Tinit.setData(tf);

  _sensor = new SensorProjective3D(cols, rows, Pdata);
  _sensor->transform(&Tinit);
  double* coords = _kinect->getCoords();
  for(int i=0; i<cols*rows; i++)
    _dist[i] = sqrt(coords[3*i]*coords[3*i]+coords[3*i+1]*coords[3*i+1]+coords[3*i+2]*coords[3*i+2]);
  _sensor->setRealMeasurementData(_dist);
  _sensor->setRealMeasurementMask(_kinect->getMask());
  _sensor->setRealMeasurementRGB(_kinect->getRGB());

  _space->push(_sensor);

  _vModel = new VtkCloud();
  _vScene = new VtkCloud();
  _viewer3D = new Obvious3D("3DMapper");
  _viewer3D->addCloud(_vModel);
  //_viewer3D->addCloud(_vScene);
  _viewer3D->registerKeyboardCallback("space", _cbRegNewImage);
  _viewer3D->registerKeyboardCallback("i", _cbGenPointCloud);
  _viewer3D->registerKeyboardCallback("s",_cbBuildSliceViews);
  _viewer3D->registerKeyboardCallback("c",_cbStoreCurrentCloud);
  _viewer3D->registerFlipVariable("k",&_recON);
  _viewer3D->startRendering();

  delete _icp;
  delete _kinect;
  delete _vModel;
  delete _vScene;
  delete _viewer3D;
  delete _rayCaster;
  delete _space;
}
