#include "NormalsEstimator.h"
#include <obcore/math/mathbase.h>
#include <string.h>
#include <flann/flann.hpp>
#include <ANN/ANN.h>

namespace obvious
{

NormalsEstimator::NormalsEstimator()
{

}
		 
NormalsEstimator::~NormalsEstimator()
{

}
	
/**
 * Area weighted method implemented from: K. Klasing et al., Comparison of Surface Normal Estimation Methods for Range Sensing Applications,
 * In Proceedings of the 2009 IEEE international conference on Robotics and Automation (ICRA), Kobe, Japan, 2009
 */
void NormalsEstimator::estimateNormals3DGrid(unsigned int cols, unsigned int rows, double* coords, bool* mask, double* normals)
{
  for(unsigned int i=0; i<3*rows*cols; i++)
    normals[i]   = 0.0;

  for(int r=0; r<(int)rows; r++)
  {
    for(int c=0; c<(int)cols; c++)
    {
      int idx      = r*cols + c;
      if(mask[idx])
      {
        int radius    = 5;
        int idx_up    = idx - radius*cols;
        int idx_lt    = idx - radius;
        int idx_dwn   = idx + radius*cols;
        int idx_rt    = idx + radius;
        bool isValid1 = false;
        bool isValid2 = false;
        if(r >= radius && c >= radius)
          isValid1 = mask[idx_up] && mask[idx_lt];
        if((r+radius<(int)rows) && (c+radius<(int)cols))
          isValid2 = mask[idx_dwn] && mask[idx_rt];

        double u[3];
        double v[3];
        idx     *= 3;
        idx_up  *= 3;
        idx_dwn *= 3;
        idx_lt  *= 3;
        idx_rt  *= 3;

        if(isValid1)
        {
          u[0]            = coords[idx_up]   - coords[idx];
          u[1]            = coords[idx_up+1] - coords[idx+1];
          u[2]            = coords[idx_up+2] - coords[idx+2];
          v[0]            = coords[idx]      - coords[idx_lt];
          v[1]            = coords[idx+1]    - coords[idx_lt+1];
          v[2]            = coords[idx+2]    - coords[idx_lt+2];
          cross3<double>(&normals[idx], u, v);
        }

        if(isValid2)
        {
          double n[3];
          u[0]            = coords[idx]      - coords[idx_dwn];
          u[1]            = coords[idx+1]    - coords[idx_dwn+1];
          u[2]            = coords[idx+2]    - coords[idx_dwn+2];
          v[0]            = coords[idx_rt]   - coords[idx];
          v[1]            = coords[idx_rt+1] - coords[idx+1];
          v[2]            = coords[idx_rt+2] - coords[idx+2];
          cross3<double>(n, u, v);
          normals[idx]   += n[0];
          normals[idx+1] += n[1];
          normals[idx+2] += n[2];
        }

        // Normalize
        norm3<double>(&normals[idx]);

      }
    }
  }
}

void NormalsEstimator::estimateNormalsReverseMapping(Matrix* coords, Matrix* P, int w, int h, Matrix* normals)
{
  int *buf = (int*) malloc(w * h * sizeof(int));

  int radius = 4;
  Vector xi(4);
  xi(3) = 1.0;

  for(int i=0; i<w*h; i++)
    buf[i] = -1;

  for(unsigned int i=0; i<coords->getRows(); i++)
  {
    xi(0) = (*coords)(i,0);
    xi(1) = (*coords)(i,1);
    xi(2) = (*coords)(i,2);
    Vector ni = Matrix::multiply(*P, xi, false);
    double du = ni(0);
    double dv = ni(1);
    double dw = ni(2);
    if(dw > 10e-6)
    {
      int u = (int)( du / dw + 0.5);
      int v = h-1-(int)(dv / dw + 0.5);

      if((u>=0) && (u<w) && (v>=0) && (v<h))
      {
          buf[(v*w+u)] = i;
      }
    }
  }
  normals->setZero();

  for(int r=0; r<h; r++)
  {
    for(int c=0; c<w; c++)
    {
      // horizontal vector: left/right pixel
      //double vh[3] = {0.0, 0.0, 0.0};
      int idx      = r*w + c;
      int idx_up   = idx - radius*w;
      int idx_dwn  = idx + radius*w;
      int idx_lt   = idx - radius;
      int idx_rt   = idx + radius;

      bool isValid1 = false;
      bool isValid2 = false;

      idx     = buf[idx];
      if(idx==-1) continue;

      if(r >= radius && c >= radius)
      {
        idx_up  = buf[idx_up];
        idx_lt  = buf[idx_lt];
        isValid1 = idx!=-1 && idx_up!=-1 && idx_lt!=-1;
      }

      if((r+radius) < h && (c+radius) < w)
      {
        idx_dwn = buf[idx_dwn];
        idx_rt  = buf[idx_rt];
        isValid2 = idx!=-1 && idx_dwn!=-1 && idx_rt!=-1;
      }

      double u[3];
      double v[3];
      double cr[3];
      double n[3] = {0, 0, 0};

      if(isValid1)
      {
        u[0]            = (*coords)(idx_up,0) - (*coords)(idx,0);
        u[1]            = (*coords)(idx_up,1) - (*coords)(idx,1);
        u[2]            = (*coords)(idx_up,2) - (*coords)(idx,2);
        v[0]            = (*coords)(idx,0)    - (*coords)(idx_lt,0);
        v[1]            = (*coords)(idx,1)    - (*coords)(idx_lt,1);
        v[2]            = (*coords)(idx,2)    - (*coords)(idx_lt,2);

        cross3<double>(cr, u, v);
        norm3<double>(cr);
        memcpy(n, cr, 3*sizeof(*cr));
      }

      if(isValid2)
      {
        u[0]            = (*coords)(idx,0)    - (*coords)(idx_dwn,0);
        u[1]            = (*coords)(idx,1)    - (*coords)(idx_dwn,1);
        u[2]            = (*coords)(idx,2)    - (*coords)(idx_dwn,2);
        v[0]            = (*coords)(idx_rt,0) - (*coords)(idx,0);
        v[1]            = (*coords)(idx_rt,1) - (*coords)(idx,1);
        v[2]            = (*coords)(idx_rt,2) - (*coords)(idx,2);

        cross3<double>(cr, u, v);
        norm3<double>(cr);

        n[0] += cr[0];
        n[1] += cr[1];
        n[2] += cr[2];
      }

      // Normalize
      norm3<double>(n);
      (*normals)(idx,0) = n[0];
      (*normals)(idx,1) = n[1];
      (*normals)(idx,2) = n[2];
    }
  }
}

void NormalsEstimator::estimateNormalsFLANN(Matrix* coords, Matrix* normals)
{
  unsigned int size = coords->getRows();
  unsigned int dim  = coords->getCols();
  double** buf;
  System<double>::allocate(size, dim, buf);

  double origin[3] = {0.0, 0.0, 0.0};
  unsigned int cnt = 0;
  unsigned int* map = new unsigned int[size];
  for(unsigned int i=0; i<size; i++)
  {
    double c[3];
    c[0] = (*coords)(i,0);
    c[1] = (*coords)(i,1);
    c[2] = (*coords)(i,2);
    if(distSqr3D<double>(c, origin)>10e-3)
    {
      memcpy(&buf[cnt][0], c, dim*sizeof(double));
      map[cnt++] = i;
    }
  }

  flann::Matrix<double>* dataset = new flann::Matrix<double>(&buf[0][0], cnt, dim);
  flann::KDTreeSingleIndexParams p;
  flann::Index<flann::L2<double> >* index = new flann::Index<flann::L2<double> >(*dataset, p);
  index->buildIndex();

  int nn = 150; // number of closest points to use for local covariance estimate

  flann::Matrix<int> indices(new int[1*nn], 1, nn);
  flann::Matrix<double> dists(new double[1*nn], 1, nn);

  double mean[3];

  Matrix C(dim, dim);

  // determine covariance matrix
  for(unsigned int i=0; i<cnt; i++)
  {
    flann::Matrix<double> query(&buf[i][0], 1, dim);
    flann::SearchParams sp;
    index->knnSearch(query, indices, dists, nn, sp);

    C.setZero();
    mean[0] = 0.0; mean[1] = 0.0; mean[2] = 0.0;
    for(int j=0; j<nn; j++)
    {
      int idx  = indices[0][j];

      double x = buf[idx][0];
      double y = buf[idx][1];
      double z = buf[idx][2];;

      mean[0] += x;
      mean[1] += y;
      mean[2] += z;

      C(0,0) += x*x;

      C(1,0) += y*x;
      C(1,1) += y*y;

      C(2,0) += z*x;
      C(2,1) += z*y;
      C(2,2) += z*z;
    }

    mean[0] /= (double)nn;
    mean[1] /= (double)nn;
    mean[2] /= (double)nn;

    for(int k = 0; k < 3; k++)
    {
      for(int l = 0; l <= k; l++)
      {
        C(k,l) /= (double)nn;
        C(k,l) -= mean[k]*mean[l];
        C(l,k) = C(k,l);
      }
    }

    Matrix U(dim, dim);
    Matrix V(3, 3);
    double s[3];
    C.svd(&U, s, &V);

    double normal[3];
    normal[0] = V(0,2);
    normal[1] = V(1,2);
    normal[2] = V(2,2);
    norm3<double>(normal);

    // test facing
    double tf[3];
    tf[0] = buf[i][0] + normal[0];
    tf[1] = buf[i][1] + normal[1];
    tf[2] = buf[i][2] + normal[2];
    if(distSqr3D(origin, tf) > distSqr3D(origin, buf[i]))
    {
      normal[0] = -normal[0];
      normal[1] = -normal[1];
      normal[2] = -normal[2];
    }
    (*normals)(map[i],0) = normal[0];
    (*normals)(map[i],1) = normal[1];
    (*normals)(map[i],2) = normal[2];
  }

  delete index;
  delete dataset;

  System<double>::deallocate(buf);
  delete [] map;
}

void NormalsEstimator::estimateNormalsANN(Matrix* coords, Matrix* normals)
{
  unsigned int size = coords->getRows();
  unsigned int dim  = coords->getCols();
  double** buf;
  System<double>::allocate(size, dim, buf);

  double origin[3] = {0.0, 0.0, 0.0};
  unsigned int cnt = 0;
  unsigned int* map = new unsigned int[size];
  for(unsigned int i=0; i<size; i++)
  {
    double c[3];
    c[0] = (*coords)(i,0);
    c[1] = (*coords)(i,1);
    c[2] = (*coords)(i,2);
    if(distSqr3D<double>(c, origin)>10e-3)
    {
      memcpy(&buf[cnt][0], c, dim*sizeof(double));
      map[cnt++] = i;
    }
  }

  ANNkd_tree* index = new ANNkd_tree(buf, cnt, 3, 10);

  int nn = 150; // number of closest points to use for local covariance estimate

  ANNdist *dists  = new ANNdist[nn];
  ANNidx *indices = new ANNidx[nn];
  ANNpoint query  = annAllocPt(3);

  double mean[3];
  Matrix C(dim, dim);

  // determine covariance matrix
  for(unsigned int i=0; i<cnt; i++)
  {
    query[0] = buf[i][0];
    query[1] = buf[i][1];
    query[2] = buf[i][2];

    index->annkSearch(query, nn, indices, dists, 0.0);

    C.setZero();
    mean[0] = 0.0; mean[1] = 0.0; mean[2] = 0.0;
    for(int j=0; j<nn; j++)
    {
      int idx  = indices[j];

      double x = buf[idx][0];
      double y = buf[idx][1];
      double z = buf[idx][2];;

      mean[0] += x;
      mean[1] += y;
      mean[2] += z;

      C(0,0) += x*x;

      C(1,0) += y*x;
      C(1,1) += y*y;

      C(2,0) += z*x;
      C(2,1) += z*y;
      C(2,2) += z*z;
    }

    mean[0] /= (double)nn;
    mean[1] /= (double)nn;
    mean[2] /= (double)nn;

    for(int k = 0; k < 3; k++)
    {
      for(int l = 0; l <= k; l++)
      {
        C(k,l) /= (double)nn;
        C(k,l) -= mean[k]*mean[l];
        C(l,k) = C(k,l);
      }
    }

    Matrix U(dim, dim);
    Matrix V(3, 3);
    double s[3];
    C.svd(&U, s, &V);

    double normal[3];
    normal[0] = V(0,2);
    normal[1] = V(1,2);
    normal[2] = V(2,2);
    norm3<double>(normal);

    // test facing
    double tf[3];
    tf[0] = buf[i][0] + normal[0];
    tf[1] = buf[i][1] + normal[1];
    tf[2] = buf[i][2] + normal[2];
    if(distSqr3D(origin, tf) > distSqr3D(origin, buf[i]))
    {
      normal[0] = -normal[0];
      normal[1] = -normal[1];
      normal[2] = -normal[2];
    }
    (*normals)(map[i],0) = normal[0];
    (*normals)(map[i],1) = normal[1];
    (*normals)(map[i],2) = normal[2];
  }

  delete [] indices;
  delete index;

  System<double>::deallocate(buf);
  delete [] map;
}
}
