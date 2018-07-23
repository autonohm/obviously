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
#include "obcore/math/linalg/linalg.h"

#include "obcore/math/Trajectory.h"

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
   * Standard Constructor
   * @param windowName name of window
   * @param sizex width of window in pixel
   * @param sizey height of window in pixel
   * @param posx x-position of window origin
   * @param posy y-position of window origin
   * @param rgb background color
   */
  Obvious3D(const char* windowName=NULL, unsigned int sizx=1024, unsigned int sizy=768, unsigned int posx=0, unsigned int posy=0, double rgb[3]=0);

  /**
   * Destructor
   */
  ~Obvious3D();

  /**
   * Add point cloud
   * @param cloud point cloud
   * @param pickable flag stating whether cloud can be picked and moved within the viewer
   * @param pointsize size of points
   * @param opacity opacity of points (0.0 = transparent, 1.0 = solid)
   */
  void addCloud(VtkCloud* cloud, bool pickable=true, unsigned int pointsize=1, double opacity=0.7);

  /**
   * Add array of lines
   * @param coordsStart coordinates (xyz-triples) of starting points
   * @param coordsEnd coordinates (xyz-triples) of end points
   * @param size number of lines
   * @param rgb line color
   */
  void addLines(double** coordsStart, double** coordsEnd, unsigned int size, double rgb[]=0);

  /**
   * Add plane
   * @param origin coordinates of origin
   * @param axis1 coordinates of end point of first axis
   * @param axis2 coordinates of end point of second axis
   * @param resX resolution in x-direction, i.e., number of cells
   * @param resY resolution in y-direction, i.e., number of cells
   * @param rgb surface color
   */
  void addPlane(double origin[3], double axis1[3], double axis2[3], unsigned int resX = 0, unsigned int resY = 0, double rgb[]=0);

  /**
   * Add sphere
   * @param center coordinates of center
   * @param radius sphere radius
   * @param rgb surface color
   */
  void addSphere(double center[3], double radius, double rgb[]=0);

  /**
   * Add light source
   * @param pos position of light source
   * @param focal coordinates of focal point
   */
  void addLight(double pos[3], double focal[3]);

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
   * @param show set to TRUE for visualization (default:=TRUE)
   */
  void showAxes(bool show = true);

  /**
   * Function to show position of sensor in viewer
   * @param position position in x,y,z
   */
  void showSensorPosition(const double* position);

  /**
   * Function to set transformation matrix to view sensor
   * @param T Transformation matrix with size 16
   */
  void showSensorPose(const double* T);

  /**
   * Function to set transformation with Matrix from obcore
   * @param T Transformation matrix
   */
  void showSensorPose(Matrix& T);

  void showTrajectory(std::vector<Matrix> trajectory);

  /**
   * Save screenshot to /tmp directory. This method is also executed with the key 's'
   */
  void screenshot();

  /**
   * Register a callback function for a specific keypress event
   * @param key key code
   * @param fptr function pointer
   * @return success if registration is accepted. Method fails when key specifies a standard keycode
   */
  bool registerKeyboardCallback(const char key[], fptrKeyboardCallback fptr, const char desc[] = "no description");

  /**
   * Register variable to be toggled by keypress event
   * @param key key code
   * @param flip reference for variable toggling
   * @return success if registration is accepted. Method fails when key specifies a standard keycode
   */
  bool registerFlipVariable(const char key[], bool* flip);

  /**
   * Register variable to be increased by keypress event
   * @param key key code
   * @param var reference to target variable
   * @param inc incrementation parameter
   * @return success if registration is accepted. Method fails when key specifies a standard keycode
   */
  bool registerIncVariable(const char key[], double* var, double inc);

  /**
   * Access internal renderer
   * @return reference to internal renderer
   */
  vtkSmartPointer<vtkRenderer> getRenderer();

  /**
   * Access internal window interactor
   * @return reference to internal window interactor
   */
  vtkSmartPointer<vtkRenderWindowInteractor> getWindowInteractor();

  /**
   * Set viewing frustum
   * @param xmin minimum x coordinate
   * @param xmax maximum x coordinate
   * @param ymin minimum y coordinate
   * @param ymax maximum y coordinate
   * @param zmin minimum z coordinate
   * @param zmax maximum z coordinate
   */
  void setFrustum(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

  /**
   * Add cube composed of lines
   * @param xmin minimum x coordinate
   * @param xmax maximum x coordinate
   * @param ymin minimum y coordinate
   * @param ymax maximum y coordinate
   * @param zmin minimum z coordinate
   * @param zmax maximum z coordinate
   */
  void addAxisAlignedCube(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

private:

  bool checkVariableRegistration(std::string key);

  vtkSmartPointer<vtkRenderer>                _renderer;
  vtkSmartPointer<vtkRenderWindow>            _renderWindow;
  vtkSmartPointer<vtkRenderWindowInteractor>  _renderWindowInteractor;
  vtkSmartPointer<vtkExtractSelectedFrustum>  _frust;
  vtkSmartPointer<vtkAxesActor>               _sensor_axes;
};

}

#endif //OBVIOUS3D_H
