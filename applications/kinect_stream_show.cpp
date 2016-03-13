#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/Kinect.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obcore/base/Timer.h"
#include "obcore/base/CartesianCloudFactory.h"
using namespace std;
using namespace obvious;

Obvious3D*        _viewer;
VtkCloud*         _cloud;
Kinect*           _kinect;
NormalsEstimator* _nestimator;
double*           _normals;
bool              _pause       = false;
bool              _showNormals = false;

void serializePLY();
void serializeXML();

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
      if(_kinect->grab())
      {
        int rows           = _kinect->getRows();
        int cols           = _kinect->getCols();
        int size           = rows * cols;
        double* coords     = _kinect->getCoords();
        unsigned char* rgb = _kinect->getRGB();
        bool* mask         = _kinect->getMask();
        if(_showNormals)
        {
          _nestimator->estimateNormals3DGrid(cols, rows, coords, mask, _normals);
          _cloud->setCoords(coords, size, 3, _normals);
        }
        else
        {
          _cloud->setCoords(coords, size, 3);
        }
        _cloud->setColors(rgb, size, 3);
        _viewer->update();
      }
    }
  }

private:

};

static int cnt = 0;
char filename[64];
void serializePLY()
{
  sprintf(filename, "cloud%05d.ply", cnt++);
  cout << "serializing " << filename << endl;
  _cloud->serialize(filename, VTKCloud_PLY);
}

void serializeXML()
{
  sprintf(filename, "cloud%05d.vtp", cnt++);
  cout << "serializing " << filename << endl;
  _cloud->serialize(filename, VTKCloud_XML);

  CartesianCloud3D* cloud = new CartesianCloud3D(640*480, _kinect->getCoords(), _kinect->getRGB(), NULL);
  //for(int i=0; i<640*480; i++)
  //  if(fabs((*(cloud->getCoords()))(i,2))<1e-6) (cloud->getAttributes())[i] &= ~ePointAttrValid;
  //cloud->removeInvalidPoints();
  char path[] = "/tmp/serialize.txt";
  CartesianCloudFactory::serialize(path, cloud, eFormatAscii);
}

void recordCallback()
{
  static bool record = false;

  char filename[64] = "kinect.rec";

  if(!record)
    _kinect->startRecording(filename);
  else
    _kinect->stopRecording();

  record = !record;
}

int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <config.xml>" << endl;
    return -1;
  }

  _kinect     = new Kinect(argv[1]);
  _nestimator = new NormalsEstimator();
  _normals    = new double[3*_kinect->getRows()*_kinect->getCols()];
  _cloud      = new VtkCloud();
  _viewer     = new Obvious3D("KinectStreamShow", 1024, 768, 0, 0);
  //_kinect->useBilinearFilter(true);

  _viewer->addCloud(_cloud);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->registerKeyboardCallback("y", serializePLY);
  _viewer->registerKeyboardCallback("x", serializeXML);
  _viewer->registerKeyboardCallback("w", recordCallback);
  _viewer->registerFlipVariable("space", &_pause);
  _viewer->registerFlipVariable("n",     &_showNormals);

  _viewer->startRendering();

  delete _cloud;
  delete _normals;
  delete _nestimator;
  delete _kinect;

  return 0;
}
