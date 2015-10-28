#include "RansacPrimitives.h"
#include <math.h>

namespace obvious
{

RansacPrimitives::RansacPrimitives(unsigned int trials)
{
  _trials = trials;
}

RansacPrimitives::~RansacPrimitives()
{

}

#define SQR(x) (x*x)
bool RansacPrimitives::findCircle(const obvious::Matrix* D, double params[3], bool* mask)
{
  unsigned int consensus_max = 0;
  double p_best[3] = {0, 0, -1};

  unsigned int vals = D->getRows();

  const double n = 3.0;
  for(unsigned int i=1; i<_trials; i++)
  {
    // Determine indices of candidates
    unsigned int idx1 = round((rand()%1000)/1000.0 * (vals-1)+0.5);
    unsigned int idx2 = round((rand()%1000)/1000.0 * (vals-1)+0.5);
    unsigned int idx3 = round((rand()%1000)/1000.0 * (vals-1)+0.5);
    //cout << idx1 << " " << idx2 << " " << idx3 << endl;
    // Ensure, not to have chosen an index twice
    if (idx1 == idx2 ||  idx2 == idx3 || idx1 == idx3)
      continue;

    // Sample set
    double x0 = (*D)(idx1,0);
    double x1 = (*D)(idx2,0);
    double x2 = (*D)(idx3,0);
    double y0 = (*D)(idx1,1);
    double y1 = (*D)(idx2,1);
    double y2 = (*D)(idx3,1);
    double x0_2 = x0*x0;
    double x1_2 = x1*x1;
    double x2_2 = x2*x2;
    double y0_2 = y0*y0;
    double y1_2 = y1*y1;
    double y2_2 = y2*y2;
    double sumx  = x0+x1+x2;
    double sumx2 = x0_2+x1_2+x2_2;
    double sumy  = y0+y1+y2;
    double sumy2 = y0_2+y1_2+y2_2;
    double sumxy = x0*y0+x1*y1+x2*y2;
    double sumx3 = x0_2*x0+x1_2*x1+x2_2*x2;
    double sumy3 = y0_2*y0+y1_2*y1+y2_2*y2;
    double sumxy2 = x0*y0_2+x1*y1_2+x2*y2_2;
    double sumx2y = x0_2*y0+x1_2*y1+x2_2*y2;

    // Estimate model parameters
    double a1 = 2*(sumx*sumx - n * sumx2);
    double a2 = 2* (sumx*sumy - n * sumxy);
    double b1 = a2;
    double b2 = 2*(sumy*sumy- n * sumy2);
    double c1 = sumx2*sumx - n* sumx3 + sumx*sumy2 - n * sumxy2;
    double c2 = sumx2*sumy - n* sumy3 + sumy*sumy2 - n * sumx2y;

    double denom = a1*b2-a2*b1;
    double p[3];
    // x
    p[0] = (c1*b2-c2*b1) / denom;
    // y
    p[1] = (a1*c2-a2*c1) / denom;
    // r
    p[2] = sqrt(1.0/n * ( sumx2 - 2.0*p[0]*sumx + n*p[0]*p[0] + sumy2 - 2.0*p[1]*sumy + n*p[1]*p[1] ) );

    // Find consensus set and rate
    unsigned int consensus = 0;
    double thresh = 10e-2;
    for(unsigned int j=1; j<vals; j++)
    {
      double d1 = ((*D)(j,0) - p[0]);
      double d2 = (*D)(j,1)-p[1];
      double epsilon = d1*d1 + d2*d2 - p[2]*p[2];
      epsilon*=epsilon;
      if(epsilon<thresh)
        consensus++;
    }

    // Remember best match
    if(consensus>consensus_max)
    {
      consensus_max = consensus;
      p_best[0] = p[0];
      p_best[1] = p[1];
      p_best[2] = p[2];
    }
  }

  params[0] = p_best[0];
  params[1] = p_best[1];
  params[2] = p_best[2];

  return true;
}

}// namespace
