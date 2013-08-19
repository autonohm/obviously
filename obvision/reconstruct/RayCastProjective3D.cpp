#include "RayCastProjective3D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCastProjective3D::RayCastProjective3D(const unsigned int cols, const unsigned int rows, Projection* projection, TsdSpace* space)
{
  _cols = cols;
  _rows = rows;

  _space = space;
  _projection = projection;

  System<Matrix*>::allocate(_cols, _rows, _rays);
  for(int col=0; col<_cols; col++)
    for(int row=0; row<_rows; row++)
    {
      _rays[col][row] = new Matrix(4, 1);
      _projection->project2Space(col, row, 1.0, _rays[col][row]);

      // Normalize ray to size of voxel
      Matrix* M = _rays[col][row];
      double len = sqrt((*M)[0][0]*(*M)[0][0] + (*M)[1][0]*(*M)[1][0] + (*M)[2][0]*(*M)[2][0]);
      len /= _space->getVoxelSize();
      (*M)[0][0] /= len;
      (*M)[1][0] /= len;
      (*M)[2][0] /= len;
      (*M)[3][0] = 0.0;
    }

  _T = new Matrix(4, 4);
  _T->setIdentity();

  _Tinv = new Matrix(4, 4);
  _Tinv->setIdentity();

  for(int i=0; i<3; i++)
    _tr[i] = 0;
}

RayCastProjective3D::~RayCastProjective3D()
{
  delete _T;
  delete _Tinv;

  for(int col=0; col<_cols; col++)
    for(int row=0; row<_rows; row++)
      delete _rays[col][row];
  System<Matrix*>::deallocate(_rays);
}

void RayCastProjective3D::setTransformation(double *TData)
{
  Matrix T(4, 4, TData);
  (*_T)         *= T;

  (*_Tinv) = _T->getInverse();

  _tr[0] = (*_T)[0][3];
  _tr[1] = (*_T)[1][3];
  _tr[2] = (*_T)[2][3];
}

double* RayCastProjective3D::getTransformation()
{
  return _T->getBuffer()->data;
}

void RayCastProjective3D::calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* ctr, unsigned int subsampling)
{
  Timer t;
  *ctr = 0;

#pragma omp parallel
  {
    double depth = 0.0;
    double          c[3];
    double          n[3];
    unsigned char color[3];
    double* c_tmp            = new double[_rows*_cols*3];
    double* n_tmp            = new double[_rows*_cols*3];
    unsigned char* color_tmp = new unsigned char[_rows*_cols*3];
    unsigned int cnt_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for (unsigned int row = 0; row < _rows; row+=subsampling)
    {
      for (unsigned int col = 0; col < _cols; col+=subsampling)
      {
        if (rayCastFromCurrentView(row, col, c, n, color, &depth)) // Ray returned with coordinates
        {
          M[0][0] = c[0];
          M[1][0] = c[1];
          M[2][0] = c[2];
          N[0][0] = n[0];
          N[1][0] = n[1];
          N[2][0] = n[2];
          M       = *_Tinv * M;
          N       = *_Tinv * N;
          for (unsigned int i = 0; i < 3; i++)
          {
            c_tmp[cnt_tmp]      = M[i][0];
            color_tmp[cnt_tmp]  = color[i];
            n_tmp[cnt_tmp++]    = N[i][0];
          }
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[*ctr],  c_tmp,     cnt_tmp*sizeof(double));
      memcpy(&normals[*ctr], n_tmp,     cnt_tmp*sizeof(double));
      memcpy(&rgb[*ctr],     color_tmp, cnt_tmp*sizeof(unsigned char));
      *ctr += cnt_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    delete[] color_tmp;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *ctr << " coordinates");
}


bool RayCastProjective3D::rayCastFromCurrentView(const unsigned int row, const unsigned int col, double coordinates[3], double normal[3], unsigned char rgb[3], double* depth)
{
  double dirVec[3];
  double position[3];
  double position_prev[3];

  calcRayFromCurrentView(row, col, dirVec);

  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  // Interpolation weight
  double interp;

  double xmin   = ((double)(dirVec[0] > 0.0 ? 0 : (xDim-1)*voxelSize) - _tr[0]) / dirVec[0];
  double ymin   = ((double)(dirVec[1] > 0.0 ? 0 : (yDim-1)*voxelSize) - _tr[1]) / dirVec[1];
  double zmin   = ((double)(dirVec[2] > 0.0 ? 0 : (zDim-1)*voxelSize) - _tr[2]) / dirVec[2];
  double idxMin = max(max(xmin, ymin), zmin);
  idxMin        = max(idxMin, 0.0);

  double xmax   = ((double)(dirVec[0] > 0.0 ? (xDim-1)*voxelSize : 0) - _tr[0]) / dirVec[0];
  double ymax   = ((double)(dirVec[1] > 0.0 ? (yDim-1)*voxelSize : 0) - _tr[1]) / dirVec[1];
  double zmax   = ((double)(dirVec[2] > 0.0 ? (zDim-1)*voxelSize : 0) - _tr[2]) / dirVec[2];
  double idxMax = min(min(xmax, ymax), zmax);

  if (idxMin >= idxMax)
    return false;

  double tsdf_prev;
  position[0] = _tr[0] + idxMin * dirVec[0];
  position[1] = _tr[1] + idxMin * dirVec[1];
  position[2] = _tr[2] + idxMin * dirVec[2];
  _space->interpolateTrilinear(position, &tsdf_prev);

  bool found = false;
  for(int i=idxMin; i<idxMax; i++)
  {
    // calculate current position
    memcpy(position_prev, position, 3 * sizeof(*position));

    position[0] += dirVec[0];
    position[1] += dirVec[1];
    position[2] += dirVec[2];

    double tsdf;
    if (!_space->interpolateTrilinear(position, &tsdf))
      continue;

    // check sign change
    if(tsdf_prev > 0 && tsdf_prev < 0.99999 && tsdf < 0)
    {
      interp = tsdf_prev / (tsdf_prev - tsdf);
      _space->interpolateTrilinearRGB(position, rgb);
      found = true;
      break;
    }

    tsdf_prev = tsdf;
  }

  if(!found) return false;

  // interpolate between voxels when sign change happened
  for (unsigned int i = 0; i < 3; i++)
    coordinates[i] = position_prev[i] + dirVec[i] * interp;

  if(!_space->interpolateNormal(coordinates, normal))
    return false;

  return true;
}

void RayCastProjective3D::calcRayFromCurrentView(const unsigned int row, const unsigned int col, double dirVec[3])
{
  Matrix ray(4, 1);

  // bring peakpoint in map coordinate system
  ray = *_rays[col][row];
  ray = *_T * ray;

  dirVec[0] = ray[0][0];
  dirVec[1] = ray[1][0];
  dirVec[2] = ray[2][0];
}


bool RayCastProjective3D::generatePointCloud(double **pointCloud, double **cloudNormals, unsigned char **cloudRgb, unsigned int *nbr)
{
  //
  // X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
  // X_AXS_N parallel to X-Axis negative direction
  // Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
  // Y_AXS_N parallel to Y-Axis negative direction
  // Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
  // Z_AXS_N parallel to Z-Axis negative direction
  //
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  std::vector<double> pointCloudVec;
  std::vector<double> cloudNormalsVec;
  std::vector<unsigned char> cloudRgbVec;

  double depth;
  double *footPoint=new double[3];
  double *dirVec=new double[3];
  unsigned int pclSize=0;
  unsigned int cloudNormalsize=0;
  unsigned int steps;

  //x_parallel
  cout<<"\nX_AXIS\n";
  for (int row = 0; row < yDim; row++)
  {
    for (int col = 0; col < zDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,X_AXS)))
      {
        cout<<"\nGENPCL: Error calculating X-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,X_AXS_N)))
      {
        cout<<"\nGENPCL: Error calculating X-axis negative Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);

    }
  }

  //y_parallel
  cout<<"\nY_AXIS\n";
  for (int row = 0; row < zDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Y_AXS)))
      {
        cout<<"\nGENPCL: Error calculating Y-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Y_AXS_N)))
      {
        cout<<"\nGENPCL: Error calculating Y-axis negative Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
    }
  }

  //z_parallel
  cout<<"\nZ_AXIS\n";
  for (int row = 0; row < yDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Z_AXS)))
      {
        cout<<"\nGENPCL: Error calculating Z-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Z_AXS_N)))
      {
        cout<<"\nGENPCL: Error calculating Z-axis negative Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
    }
  }
  pclSize=pointCloudVec.size();
  cloudNormalsize=cloudNormalsVec.size();

  if(pclSize!=cloudNormalsize)
  {
    cout<<"GENPCL: Error! Number of normals != number of points!\n";
    return(false);
  }
  else
  {
    *pointCloud=new double[pclSize];
    *cloudNormals=new double[cloudNormalsize];
    *cloudRgb=new unsigned char[pclSize];
  }

  for(unsigned int i=0;i<pclSize;i++)
  {
    /*std::vector<double>::iterator pclIter=pointCloudVec.begin();
		std::vector<double>::iterator nrmIter=cloudNormalsVec.begin();
		std::vector<unsigned char>::iterator rgbIter=cloudRgbVec.begin();
		(*pointCloud)[i]=*pclIter;
		pclIter++;
		(*cloudNormals)[i]=*nrmIter;
		nrmIter++;
		(*cloudRgb)[i]=*rgbIter;
		rgbIter++;*/
    (*pointCloud)[i]=pointCloudVec[i];
    (*cloudNormals)[i]=cloudNormalsVec[i];
    (*cloudRgb)[i]=cloudRgbVec[i];
  }

  /**pointCloud=pointCloudVec.data();
   *cloudNormals=cloudNormalsVec.data();
   *cloudRgb=cloudRgbVec.data();*/
  *nbr=pclSize;
  return(true);
}

bool RayCastProjective3D::generatePointCloudPositive(double **pointCloud, double **cloudNormals, unsigned char **cloudRgb, unsigned int *nbr)
{
  // X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
  // X_AXS_N parallel to X-Axis negative direction
  // Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
  // Y_AXS_N parallel to Y-Axis negative direction
  // Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
  // Z_AXS_N parallel to Z-Axis negative direction
  //
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  std::vector<double> pointCloudVec;
  std::vector<double> cloudNormalsVec;
  std::vector<unsigned char> cloudRgbVec;

  double depth;
  double *footPoint=new double[3];
  double *dirVec=new double[3];
  unsigned int pclSize=0;
  unsigned int cloudNormalsize=0;
  unsigned int steps;

  //x_parallel
  cout<<"\nX_AXIS\n";
  for (int row = 0; row < yDim; row++)
  {
    for (int col = 0; col < zDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,X_AXS)))
      {
        cout<<"\nGENPCL: Error calculating X-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      /*	if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,X_AXS_N)))
			{
				cout<<"\nGENPCL: Error calculating X-axis negative Raycaster!\n";
				return(false);
			}
			rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);*/

    }
  }

  //y_parallel
  cout<<"\nY_AXIS\n";
  for (int row = 0; row < zDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Y_AXS)))
      {
        cout<<"\nGENPCL: Error calculating Y-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      /*	if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Y_AXS_N)))
			{
				cout<<"\nGENPCL: Error calculating Y-axis negative Raycaster!\n";
				return(false);
			}
			rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);*/
    }
  }

  //z_parallel
  cout<<"\nZ_AXIS\n";
  for (int row = 0; row < yDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Z_AXS)))
      {
        cout<<"\nGENPCL: Error calculating Z-axis Raycaster!\n";
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);
      /*		if(!(calcRayParallelAxis(row,col,footPoint,dirVec,&steps,Z_AXS_N)))
			{
				cout<<"\nGENPCL: Error calculating Z-axis negative Raycaster!\n";
				return(false);
			}
			rayCastParallelAxis(footPoint,dirVec,&pointCloudVec,&cloudNormalsVec,&cloudRgbVec,steps);*/
    }
  }
  pclSize=pointCloudVec.size();
  cloudNormalsize=cloudNormalsVec.size();

  if(pclSize!=cloudNormalsize)
  {
    cout<<"GENPCL: Error! Number of normals != number of points!\n";
    return(false);
  }
  else
  {
    *pointCloud=new double[pclSize];
    *cloudNormals=new double[cloudNormalsize];
    *cloudRgb=new unsigned char[pclSize];
  }

  for(unsigned int i=0;i<pclSize;i++)
  {
    /*std::vector<double>::iterator pclIter=pointCloudVec.begin();
		std::vector<double>::iterator nrmIter=cloudNormalsVec.begin();
		std::vector<unsigned char>::iterator rgbIter=cloudRgbVec.begin();
		(*pointCloud)[i]=*pclIter;
		pclIter++;
		(*cloudNormals)[i]=*nrmIter;
		nrmIter++;
		(*cloudRgb)[i]=*rgbIter;
		rgbIter++;*/
    (*pointCloud)[i]=pointCloudVec[i];
    (*cloudNormals)[i]=cloudNormalsVec[i];
    (*cloudRgb)[i]=cloudRgbVec[i];
  }


  /**pointCloud=pointCloudVec.data();
   *cloudNormals=cloudNormalsVec.data();
   *cloudRgb=cloudRgbVec.data();*/
  *nbr=pclSize;
  return(true);
}

bool RayCastProjective3D::rayCastParallelAxis(double *footPoint,double *dirVec,std::vector<double> *pointCloud,std::vector<double> *cloudNormals, std::vector<unsigned char>* cloudRgb,const unsigned int steps)
{
  double tsdf=0.0;
  double tsdfPrev=0.0;
  double curPrevInterp=0.0;
  double curPosition[3]={0.0};
  double prevPosition[3]={0.0};
  double zeroCrossing[3]={0.0};
  double normal[3]={0.0};
  unsigned char zerCrossingRgb[3]={0.0};
  memcpy(curPosition,footPoint,3*sizeof(double));   //set startposition on footpoint

  if (!_space->interpolateTrilinear(curPosition, &tsdfPrev))
    tsdfPrev=1.0;

  for(unsigned int i=0;i<=steps;i++)
  {
    //calculate new position, store old position
    memcpy(prevPosition,curPosition,3*sizeof(double));
    for(unsigned int i=0;i<3;i++)
      curPosition[i]+=dirVec[i];

    if (!_space->interpolateTrilinear(curPosition, &tsdf))
      continue;

    // check sign change
    if(tsdfPrev > 0 && tsdfPrev < 0.99999 && tsdf < 0)
    {
      // interpolate between voxels when sign change happened
      curPrevInterp = tsdfPrev / (tsdfPrev - tsdf);
      for (unsigned int i = 0; i < 3; i++)
        zeroCrossing[i] = prevPosition[i] + dirVec[i] * curPrevInterp;

      if(!_space->interpolateNormal(zeroCrossing, normal))
        continue;
      if(!_space->interpolateTrilinearRGB(curPosition, zerCrossingRgb))
      {
        continue;
      }


      for(unsigned int i=0;i<3;i++)    //found zero crossing with normal -> store in cloud
      {
        pointCloud->push_back(zeroCrossing[i]);
        cloudNormals->push_back(normal[i]);
        cloudRgb->push_back(zerCrossingRgb[i]);
      }

    }
    tsdfPrev = tsdf;

  }
  return(true);
}

bool RayCastProjective3D::calcRayParallelAxis(const unsigned int row,const unsigned int col,double *footPoint,double *dirVec,unsigned int *steps,AXSPARMODE mode)
{
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  if((mode==X_AXS)||(mode==X_AXS_N))       //parallel to x_axis
  {
    footPoint[1]=((double) row + 0.5) * voxelSize;
    footPoint[2]=((double) col + 0.5) * voxelSize;
    dirVec[1]=0.0;
    dirVec[2]=0.0;
    *steps=xDim-1;
    if(mode==X_AXS)
    {
      footPoint[0]=0.5f*voxelSize;
      dirVec[0]=voxelSize;
    }
    else
    {
      footPoint[0]=((double)(xDim-1) + 0.5) * voxelSize;
      dirVec[0]=(-1.0)*voxelSize;
    }
  }

  else if((mode==Y_AXS)||(mode==Y_AXS_N))       //parallel to x_axis
  {
    footPoint[0]=((double) col + 0.5) * voxelSize;
    footPoint[2]=((double) row + 0.5) * voxelSize;
    dirVec[0]=0.0;
    dirVec[2]=0.0;
    *steps=yDim-1;
    if(mode==Y_AXS)
    {
      footPoint[1]=0.5f*voxelSize;
      dirVec[1]=voxelSize;
    }
    else
    {
      footPoint[1]=((double)(yDim-1) + 0.5) * voxelSize;
      dirVec[1]=(-1.0)*voxelSize;
    }
  }

  else if((mode==Z_AXS)||(mode==Z_AXS_N))       //parallel to x_axis
  {
    footPoint[0]=((double) col + 0.5) * voxelSize;
    footPoint[1]=((double) row + 0.5) * voxelSize;
    dirVec[0]=0.0;
    dirVec[1]=0.0;
    *steps=zDim-1;
    if(mode==Z_AXS)
    {
      footPoint[2]=0.5*voxelSize;
      dirVec[2]=voxelSize;
    }
    else
    {
      footPoint[2]=((double)(zDim-1) + 0.5) * voxelSize;
      dirVec[2]=(-1.0)*voxelSize;
    }
  }
  else
  {
    cout<<"\nERROR. Mode "<<mode<<" not found!\n";
    return(false);
  }

  return (true);
}

}
