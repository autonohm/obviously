#ifndef OBVIOUS3D_H
#define OBVIOUS3D_H

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include "VtkCloud.h"
#include <vtkExtractSelectedFrustum.h>
#include <vtkAxesActor.h>
#include "../obcore/math/Matrix.h"



#include "Obvious.h"

namespace obvious
{

/**
 * VTK-based 3D viewer
 * @author Stefan May
 */
class Obvious3D
{
public:
  /**
   * Default Constructor
   */
  Obvious3D(const char* windowName = NULL, unsigned int sizx = 1024, unsigned int sizy = 768, unsigned int posx = 0, unsigned int posy = 0);

  /**
   * Destructor
   */
  ~Obvious3D();

  /**
   * Add point cloud for visualization
   * @param cloud point cloud
   * @param pickable flag stating whether cloud can be picked and moved within the viewer
   * @param pointsize size of points
   */
  void addCloud(VtkCloud* cloud, bool pickable=true, unsigned int pointsize=1);

  /**
   * Update visualization (needed to be called after modifying added point clouds
   */
  void update();

  /**
   * Start rendering loop. The control is taken over by VTK.
   * In order to execute user-defined code one can use callback functions.
   */
  void startRendering();

  /**
   * Function to show coordinate axes in viewer
   * @param[in]     show      set to TRUE for visualization (default:=TRUE)
   */
  void showAxes(bool show = true);

  /**
   * Function to show position of sensor in viewer
   * @param[in]     position    position in x,y,z
   */
  void showSensorPosition(const double* position);

  /**
   * Function to set transformation matrix to view sensor
   * @param[in]     T           Transformation matrix with size 16
   */
  void showSensorPose(const double* T);

  void showSensorPose(Matrix T);

  /**
   * Register a callback function for a specific keypress event
   * @param key key code
   * @param fptr function pointer
   */
  void registerKeyboardCallback(const char key[], fptrKeyboardCallback fptr);

  void registerFlipVariable(const char key[], bool* flip);

  void registerIncVariable(const char key[], double* var, double inc);

  vtkSmartPointer<vtkRenderer> getRenderer();

  vtkSmartPointer<vtkRenderWindowInteractor> getWindowInteractor();

  void setFrustum(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

  void addAxisAlignedCube(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

private:

  vtkSmartPointer<vtkRenderer>                _renderer;
  vtkSmartPointer<vtkRenderWindow>            _renderWindow;
  vtkSmartPointer<vtkRenderWindowInteractor>  _renderWindowInteractor;
  vtkSmartPointer<vtkExtractSelectedFrustum>  _frust;
  vtkSmartPointer<vtkAxesActor>               _sensor_axes;
};

}

#endif //OBVIOUS3D_H
