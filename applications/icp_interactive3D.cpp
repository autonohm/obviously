#include "obcore/base/CartesianCloudFactory.h"
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obcore/math/geometry.h"

#include "obvision/icp/icp_def.h"

#include <sstream>

#include "obgraphic/Obvious3D.h"

using namespace obvious;

void iterateCallback();
void stepCallback();
void saveCallback();
void exportCallback();

VtkCloud* _vmodel                  = NULL;
VtkCloud* _vscene                  = NULL;

Icp* _icp;

Obvious3D*        _viewer;
Matrix            _Tfinal(4, 4);


int main(int argc, char* argv[])
{
  /**
   * Read data
   */
  if ( argc != 6 )
  {
    std::cout << "Usage: " << argv[0]  << " <model> <scene> <pre> <post> <est>" << std::endl;
    return EXIT_FAILURE;
  }

  int pre       = atoi(argv[3]);
  int post      = atoi(argv[4]);
  int est       = atoi(argv[5]);

  _Tfinal.setIdentity();

  _vmodel       = VtkCloud::load(argv[1], VTKCloud_AUTO);
  _vscene       = VtkCloud::load(argv[2], VTKCloud_AUTO);

  _vmodel->removeInvalidPoints();
  _vscene->removeInvalidPoints();

  CartesianCloud3D* model = new CartesianCloud3D(_vmodel->getSize(), true);
  _vmodel->copyData(model->getCoords(), model->getNormals(), model->getColors());
  CartesianCloud3D* scene = new CartesianCloud3D(_vscene->getSize(), true);
  _vscene->copyData(scene->getCoords(), scene->getNormals(), scene->getColors());

  /**
   * Configure ICP module
   */
  unsigned int iterations = 35;
  PairAssignment* assigner  = (PairAssignment*)  new FlannPairAssignment(3, 0.0);
  IRigidEstimator* estimator;
  if(est==0)
    estimator = (IRigidEstimator*) new PointToPointEstimator3D();
  else
    estimator = (IRigidEstimator*) new PointToPlaneEstimator3D();

  //IRigidEstimator* estimator = (IRigidEstimator*) new PlaneToPlaneEstimator3D();

  //double P[12] = {-213382.14771, -95.17028, 118747.14929, 368.71458, 37.18902, -213520.94495, 89038.49951, 213.42864, 0.17192, -0.26473, 371.03830, 1.00000};
  double P[12]  = {585.05108211, 0.00000000, 315.83800193, 0., 0.00000000, 585.05108211, 242.94140713, 0., 0.00000000, 0.00000000, 1.00000000, 0.};

  OcclusionFilter* filterO = new OcclusionFilter(P, 640, 480);
  if(pre == 2 || pre == 3)
    assigner->addPreFilter(filterO);

  IPreAssignmentFilter* filterS = (IPreAssignmentFilter*) new SubsamplingFilter(25);
  assigner->addPreFilter(filterS);

  ProjectionFilter* filterP = new ProjectionFilter(P, 640, 480);
  filterP->setModel(model);
  if(pre == 1 || pre == 3)
    assigner->addPreFilter(filterP);

  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(1.5, 0.03, iterations);
  if(post == 1)
    assigner->addPostFilter(filterD);

  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(iterations);
  _icp->setModel(model->getCoords(), model->getNormals());
  _icp->setScene(scene->getCoords(), scene->getNormals());

  /**
   * Configure visualization
   */
  _viewer = new Obvious3D();
  _viewer->addCloud(_vmodel, false);
  _viewer->addCloud(_vscene);

  _viewer->registerKeyboardCallback("space", stepCallback);
  _viewer->registerKeyboardCallback("i", iterateCallback);
  _viewer->registerKeyboardCallback("e", exportCallback);

  _viewer->startRendering();

}

void iterateCallback()
{
  double rms;
  unsigned int pairs;
  unsigned int iterations;

  _icp->reset();
  Timer timer;
  EnumIcpState state  = _icp->iterate(&rms, &pairs, &iterations);
  cout << "ICP state: " << state << ", elapsed (" << iterations << " iterations): "  << timer.getTime() << " ms , rms = " << rms << ", # of pairs: " << pairs << endl;

  Matrix* T           = _icp->getFinalTransformation();
  double Tdata[16];
  T->getData(Tdata);
  _vscene->transform(Tdata);
  _viewer->update();
}

void stepCallback()
{
  double rms;
  unsigned int pairs;

  Timer timer;
  EnumIcpState state  = _icp->step(&rms, &pairs);
  cout << "ICP state: " << state << ", elapsed (step): " << timer.getTime() << " ms , rms = " << rms << ", # of pairs: " << pairs << endl;

  Matrix* T           = _icp->getLastTransformation();
  double Tdata[16];
  T->getData(Tdata);
  _vscene->transform(Tdata);
  _viewer->update();
}

void exportCallback()
{
  char mfilename[] = "/tmp/model.vtp";
  char sfilename[] = "/tmp/scene.vtp";
  _vmodel->serialize(mfilename, VTKCloud_XML);
  _vscene->serialize(sfilename, VTKCloud_XML);
}
