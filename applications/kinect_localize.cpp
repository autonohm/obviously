#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/Kinect.h"
#include "obvision/icp/icp_def.h"
#include "obvision/normals/NormalsEstimator.h"

using namespace std;
using namespace obvious;

Obvious3D* _viewer;
VtkCloud*  _vmodel;
VtkCloud*  _vscene;
Kinect*    _kinect;

Icp*                   _icp;
NormalsEstimator*      _nestimator;
double*                _normals;
ProjectionFilter*      _filterP;
IPostAssignmentFilter* _filterD;

bool _reference = false;

void referenceCallback();

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
    if(_kinect->grab())
    {
      int rows           = _kinect->getRows();
      int cols           = _kinect->getCols();
      int size           = rows * cols;
      double* coords     = _kinect->getCoords();
      unsigned char* rgb = _kinect->getRGB();
      bool* mask         = _kinect->getMask();
      _vscene->setCoords(coords, size, 3);
      _vscene->setColors(rgb,    size, 3);

      if(_reference)
      {
        _nestimator->estimateNormals3DGrid(cols, rows, coords, mask, _normals);
        CartesianCloud3D*  scene = new CartesianCloud3D(size, coords, rgb,  _normals);
        scene->maskPoints(mask);
        scene->maskEmptyNormals();
        scene->removeInvalidPoints();
        scene->subsample(25);

        _filterD->reset();
        _icp->setScene(scene->getCoords(), scene->getNormals());

        double rms;
        unsigned int pairs;
        unsigned int it;
        EnumIcpState state  = _icp->iterate(&rms, &pairs, &it);

        obvious::Matrix T           = _icp->getFinalTransformation();
        //cout << "print T" << endl;
        //T->print();
        cout << "Success: " << state << ", found " << pairs << " pairs" << endl;
        cout << "Needed iterations: " << it << endl;

        double Tdata[16];
        T.getData(Tdata);

        _vscene->transform(Tdata);

        delete scene;
      }
      _viewer->update();
    }
  }

private:

};

void referenceCallback()
{

  int rows           = _kinect->getRows();
  int cols           = _kinect->getCols();
  int size           = rows * cols;
  double* coords     = _kinect->getCoords();
  unsigned char* rgb = _kinect->getRGB();
  bool* mask         = _kinect->getMask();

  _nestimator->estimateNormals3DGrid(cols, rows, coords, mask, _normals);
  CartesianCloud3D*  model = new CartesianCloud3D(size, coords, rgb, _normals);
  model->maskPoints(mask);
  model->maskEmptyNormals();
  model->removeInvalidPoints();

  _filterP->setModel(model);

  _vmodel->setCoords(coords, size, 3, _normals);
  _vmodel->setColors(rgb,    size, 3);

  _icp->setModel(model->getCoords(), model->getNormals(), 0.5);

  cout << "Reference set, size: " << model->size() << endl;
  _reference = true;

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

  /**
   * Configure ICP module
   */
  int iterations = 25;
  double P[12]  = {585.05108211, 0.00000000, 315.83800193, 0., 0.00000000, 585.05108211, 242.94140713, 0., 0.00000000, 0.00000000, 1.00000000, 0.};

  //PairAssignment* assigner  = (PairAssignment*)  new ProjectivePairAssignment(P, 640, 480);
  PairAssignment* assigner  = (PairAssignment*)  new FlannPairAssignment(3, 0.0);
  //PairAssignment* assigner  = (PairAssignment*)  new AnnPairAssignment(3);
  //IRigidEstimator* estimator = (IRigidEstimator*) new PointToPointEstimator3D();
  IRigidEstimator* estimator = (IRigidEstimator*) new PointToPlaneEstimator3D();
  //IRigidEstimator* estimator = (IRigidEstimator*) new PlaneToPlaneEstimator3D();

  _filterP = new ProjectionFilter(P, 640, 480);
  assigner->addPreFilter(_filterP);

  _filterD = (IPostAssignmentFilter*) new DistanceFilter(0.25, 0.05, iterations);
  assigner->addPostFilter(_filterD);

  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(iterations);

  _vscene  = new VtkCloud();
  _vmodel  = new VtkCloud();
  _viewer  = new Obvious3D();

  _viewer->addCloud(_vmodel);
  _viewer->addCloud(_vscene);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->registerKeyboardCallback("space", referenceCallback);
  _viewer->registerFlipVariable("d", &_filterD->_active);
  _viewer->registerFlipVariable("g", &_filterP->_active);
  _viewer->startRendering();

  delete _vmodel;
  delete _vscene;
  delete _viewer;
  delete _icp;
  delete _nestimator;
  delete _normals;
  delete _kinect;

  return 0;
}
