#ifndef VTKCLOUD_H
#define VTKCLOUD_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkDoubleArray.h>

#include <vector>
#include "obcore/base/CartesianCloud.h"

namespace obvious
{

enum EnumVtkCloudFileFormat { VTKCloud_AUTO = -1, VTKCloud_XML = 0, VTKCloud_PLY = 1/*, VTKCloud_MNI = 2*/ };

/**
 * VTK point cloud abstraction
 * @author Stefan May
 */
class VtkCloud
{
public:

  /**
   * Constructor
   */
  VtkCloud();

  /**
   * Destructor
   */
  ~VtkCloud();

  /**
   * Set coordinates
   * @param data Cartesian data set
   * @param points number of points
   * @param tda trailing dimension, i.e., size of row as laid out in memory
   *        e.g. 00 01 02 03 xx xx xx xx 10 11 12 13 xx xx xx xx
   *             => tda = 8
   */
  void setCoords(double* data, int points, int tda, double* normals=NULL);

  void setNormals(double* ndata, int size, int tda);

  void setTriangles(double** coords, unsigned char** rgb, unsigned int points, unsigned int** indices, unsigned int triangles);

  void addCoords(double* coords, unsigned char* rgb, int points, int tda);

  /**
   * Set color data
   * @param colors color data set
   * @param points number of points
   * @param channels number of color channels
   */
  void setColors(const unsigned char* colors, int points, int channels);

  void removeInvalidPoints();

  /**
   * Get size of point cloud, i.e., the number of points
   * @return size of point cloud
   */
  unsigned int getSize();

  /**
   * Copy coordinates to buffer. The size of buffer has to be getSize * 3.
   * @param dst destination buffer (layout x1y1z1x2y2z2...)
   */
  void copyCoords(double* dst, unsigned int subsampling=1);

  /**
   * Copy color array to buffer. The size of buffer has to be getSize * 3.
   * @param dst destination buffer (layout r1g1b1r2g2b2...)
   */
  void copyColors(unsigned char* dst, unsigned int subsampling=1);

  /**
   * Copy normals to buffer. The size of buffer has to be getSize * 3.
   * @param dst destination buffer (layout x1y1z1x2y2z2...)
   */
  void copyNormals(double* dst, unsigned int subsampling=1);

  void copyData(Matrix* C, Matrix* N, unsigned char* rgb);

  void transform(double* T);

  /**
   * Lowlight points by index
   * @param indices indices of previously added points
   */
  void lowlight(std::vector<unsigned int>* indices);

  /**
   * Get pointer to VTK data set, i.e., Cartesian and color data, if previously added.
   * @return pointer to poly data container
   */
  vtkSmartPointer<vtkPolyData> getPolyData();

  //vtkSmartPointer<vtkPolyData> getNormals();

  /**
   * Serialize data to file
   */
  void serialize(char* filename, EnumVtkCloudFileFormat format = VTKCloud_XML);

  /**
   * Read file and instantiate point cloud
   * @param filename file name
   * @param format format of file
   * @return instance of loaded point cloud
   */
  static VtkCloud* load(char* filename, EnumVtkCloudFileFormat format);

  /**
   * Get actor of cloud display
   * @return actor actor
   */
  vtkSmartPointer<vtkActor> getActor();

  /**
   * Set actor for mouse- or keyboard-base mutation
   * @param actor actor
   */
  void setActor(vtkSmartPointer<vtkActor> actor);

  /**
   * Get transformation of actor
   * @param T transformation matrix
   */
  void getTransformation(double T[16]);

  /**
   * Get transformation of actor
   * @param R rotation matrix
   * @param t translation vector
   */
  void getTransformation(double R[9], double t[3]);

  /**
   * Apply the transformation of the assigned actor (if any).
   * The transformation matrix of the actor is reseted.
   */
  void applyActorTransformation();

  /**
   * Reset transformation of actor
   */
  void resetActorTransformation();

  /**
   * Instantiate sample cloud
   * @return instance of example point cloud
   */
  static VtkCloud* createExample();

  static VtkCloud* createRandom(unsigned int nrPoints, double radius);

private:

  vtkSmartPointer<vtkPolyData> _polyData;
  vtkSmartPointer<vtkPoints> _points;
  vtkSmartPointer<vtkDoubleArray> _normals;
  vtkSmartPointer<vtkUnsignedCharArray> _colors;
  vtkSmartPointer<vtkActor> _actor;
  vtkSmartPointer<vtkCellArray> _triangles;

};

}

#endif //VTKCLOUD_H
