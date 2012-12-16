/**
* @file   filter_test.cpp
* @author Christian Pfitzner
* @date   11.12.2012
*
*
*/

#include <iostream>
#include "obcore/filter/Filter.h"
#include "obcore/filter/CartesianFilter.h"
#include "obcore/filter/EuclideanFilterD.h"
#include "obcore/filter/BoundingBoxFilter.h"
#include "obgraphic/Obvious3D.h"
#include <vector>

using namespace obvious;
using namespace std;

// enum for state control
enum State{
  RESET,
  CART,
  EUC,
  BB
};

//~~~~~~~~~~~~~~~~~~~~~~~~~ init global variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int size = 200000;
Obvious3D*  viewer;
VtkCloud*   cloud;

IFilter*            _filter;
CartesianFilter*    filterCartesian;
EuclideanFilterD*   filterEuclidean;
BoundingBoxFilter*  filterBB;

double* coords    = new double[size*3];
double* coordsOut = new double[size*3];
State state;


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
    unsigned int validSize;
      if (state == CART)
      {
        _filter = filterCartesian;
        filterCartesian->setAxis(IFilter::x);   // set axis to filter
        filterCartesian->setThreshold(2.0);             // specifiy threshold
        filterCartesian->setInput(coords, size);        // set input and size of data
        filterCartesian->setOutput(coordsOut);          // get output address
        filterCartesian->setFilterDirection(FILTER_BIGGER);
        filterCartesian->applyFilter();                 // start filtering
        validSize = filterCartesian->getValidSize();
      }
      else if (state == EUC)
      {
        _filter = filterEuclidean;
        filterEuclidean->setThreshold(4.0);            // set threshold to filter
        filterEuclidean->setInput(coords, size);       // set input and size of data
        filterEuclidean->setOutput(coordsOut);         // get output address
        filterEuclidean->setFilterDirection(FILTER_SMALLER);
        filterEuclidean->applyFilter();                // start filtering
        validSize = filterEuclidean->getValidSize();
      }
      else if (state == BB)
      {
        _filter = filterBB;
        filterBB->setCentroid(Point3D(0.0, 0.0, 0.0));
        filterBB->setDimension(2.0f, 1.0f, 5.0f);
        filterBB->setInput(coords, size);        // set input and size of data
        filterBB->setOutput(coordsOut);         // get output address
        filterBB->applyFilter();
        validSize = filterBB->getValidSize();
      }
      else // state == RESET
      {
        for (unsigned int i=0 ; i<size ; i++)
          coordsOut[i] = coords[i];
        validSize = size;
      }

      cloud->setCoords(coordsOut, validSize, 3);
      viewer->update();
   }
};

//~~~~~~~~~~~~~~~~~~~~~~~~~ functions to switch states ~~~~~~~~~~~~~~~~~~~~~~~~~

void state0(void)
{
  state = RESET;
}

void state1(void)
{
  state = CART;
}

void state2(void)
{
  state = EUC;
}

void state3(void)
{
  state = BB;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ main ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int main(int argc, char *argv[])
{
  std::cout << "*** Test for filters of obcore ***" << std::endl;
  std::cout << "Use keyboard for switching filters" << std::endl;
  std::cout << "\t" << "a: Cartesian Filter" << std::endl;
  std::cout << "\t" << "s: Euclidean Filter" << std::endl;
  std::cout << "\t" << "d: Bouding Box Filter" << std::endl;
  std::cout << "\t" << "r: Reset" << std::endl;
  std::cout << "\t" << "q: Quit" << std::endl;
  cloud = VtkCloud::createRandom(size, 5.0);
  cloud->copyCoords(coords);
  state = RESET;

  IFilter* _filter = NULL;
  filterCartesian = new CartesianFilter;
  filterEuclidean = new EuclideanFilterD;
  filterBB        = new BoundingBoxFilter;

  viewer = new Obvious3D();
  viewer->addCloud(cloud);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  viewer->registerKeyboardCallback("r", state0);
  viewer->registerKeyboardCallback("a", state1);
  viewer->registerKeyboardCallback("s", state2);
  viewer->registerKeyboardCallback("d", state3);

  viewer->startRendering(); // loop

  delete viewer;
  delete cloud;
  delete filterCartesian;
  delete filterEuclidean;
  delete filterBB;
}



