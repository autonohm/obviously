#include "obgraphic/Obvious3D.h"
#include "obcore/math/mathbase.h"
#include <math.h>

using namespace obvious;
using namespace std;

void generateModel(double* model, unsigned char* rgb, unsigned char* mask, double* normals, unsigned int cols, unsigned int rows);

int main(int argc, char* argv[])
{

  unsigned int rows   = 100;
  unsigned int cols   = 100;
  unsigned int points = rows*cols;
  double* coords      = new double[points*3];
  unsigned char* mask = new unsigned char[points];
  unsigned char* rgb  = new unsigned char[points*3];
  double* normals     = new double[points*3];
  generateModel(coords, rgb, mask, normals, cols, rows);

  Obvious3D* viewer = new Obvious3D((char*)"Synthetic Pointcloud", 1024, 768, 0, 0);

  VtkCloud* cloud = new VtkCloud();
  cloud->setCoords(coords, points, 3);
  cloud->setColors(rgb, points, 3);
  cloud->setNormals(normals, points, 3);

  viewer->addCloud(cloud);
  viewer->startRendering();

  delete [] coords;
  delete [] mask;
  delete [] rgb;
  delete [] normals;
  delete cloud;
  delete viewer;
  return 0;
}

void generateModel(double* model, unsigned char* rgb, unsigned char* mask, double* normals, unsigned int cols, unsigned int rows)
{
  unsigned int c, r;
  double sc = (double)cols/7.0;
  double sr = (double)rows/5.0;
  for(r=0; r<rows; r++)
  {
    for(c=0; c<cols; c++)
    {
      unsigned int i = r * cols + c;
      model[3*i]   = ((double)c)/sc;
      model[3*i+1] = ((double)r)/sr;
      model[3*i+2] = 3.0*sin(((double)c)/sc)*sin(((double)r)/sr);
      mask[i] = true;
      rgb[3*i] = 255;
      rgb[3*i+1] = 0;
      rgb[3*i+2] = 0;
    }
  }

  // Calculation of normal vectors
  for(r=1; r<rows-1; r++)
  {
    for(c=1; c<cols-1; c++)
    {
      unsigned int i = r * cols + c;
      double v[3];
      double v2[3];
      double y[3];
      double y2[3];
      double n[3];
      double n2[3];
      v[0] = model[3*(i+1)] - model[3*(i)];
      v[1] = model[3*(i+1)+1] - model[3*(i)+1];
      v[2] = model[3*(i+1)+2] - model[3*(i)+2];
      v2[0] = model[3*(i-1)] - model[3*(i)];
      v2[1] = model[3*(i-1)+1] - model[3*(i)+1];
      v2[2] = model[3*(i-1)+2] - model[3*(i)+2];
      y[0] = model[3*(i-cols)] - model[3*(i)];
      y[1] = model[3*(i-cols)+1] - model[3*(i)+1];
      y[2] = model[3*(i-cols)+2] - model[3*(i)+2];
      y2[0] = model[3*(i+cols)] - model[3*(i)];
      y2[1] = model[3*(i+cols)+1] - model[3*(i)+1];
      y2[2] = model[3*(i+cols)+2] - model[3*(i)+2];
      cross3<double>(n, v, y);
      cross3<double>(n2, v2, y2);
      normals[3*i] = n[0]+n2[0];
      normals[3*i+1] = n[1]+n2[1];
      normals[3*i+2] = n[2]+n2[2];
      norm3<double>(&normals[3*i]);
    }
  }
}
