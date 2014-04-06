#include "IcpTrace.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"

#include <iostream>
#include <fstream>

namespace obvious
{

IcpTrace::IcpTrace(unsigned int dim)
{
  _dim = dim;
  _M   = NULL;
}
		 
IcpTrace::~IcpTrace()
{
  reset();
}
	
void IcpTrace::reset()
{
  if(_M)
  {
    delete _M;
    _M = NULL;
  }

  for(unsigned int i=0; i<_scenes.size(); i++)
    delete _scenes[i];
  _scenes.clear();

  for(unsigned int i=0; i<_pairs.size(); i++)
      _pairs[i].clear();
  _pairs.clear();
}
void IcpTrace::setModel(double** model, unsigned int sizeM)
{
  if(_M) delete _M;
  _M = new Matrix(sizeM, _dim, *model);
}

void IcpTrace::addAssignment(double** scene, unsigned int sizeS, vector<StrCartesianIndexPair> pairs)
{
  Matrix* s = new Matrix(sizeS, _dim, *scene);
  _scenes.push_back(s);

  _pairs.push_back(pairs);
}

void IcpTrace::serialize(char* folder, unsigned int delay)
{
  char cmd[256];
  sprintf(cmd, "mkdir %s", folder);
  int retval = system(cmd);
  if(retval==0)
  {
    ofstream file;
    char filename[512];
    double minCoord[3] = { 10e12,  10e12,  10e12};
    double maxCoord[3] = {-10e12, -10e12, -10e12};

    if(_M)
    {
      snprintf(filename, 512, "%s/model.dat", folder);
      file.open(filename, ios::out);
      for(unsigned int p=0; p<_M->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*_M)(p,j);
          if(minCoord[j]>coord) minCoord[j] = coord;
          if(maxCoord[j]<coord) maxCoord[j] = coord;
          file << coord << " ";
        }
        file << endl;
      }
      file.close();
    }

    for(unsigned int i=0; i<_scenes.size(); i++)
    {
      snprintf(filename, 512, "%s/scene_%05d.dat", folder, i);
      file.open(filename, ios::out);
      Matrix* S = _scenes[i];
      for(unsigned int p=0; p<S->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*S)(p,j);
          if(minCoord[j]>coord) minCoord[j] = coord;
          if(maxCoord[j]<coord) maxCoord[j] = coord;
          file << coord << " ";
        }
        file << endl;
      }
      file.close();
    }

    for(unsigned int i=0; i<_pairs.size(); i++)
    {
      snprintf(filename, 512, "%s/pairs_%05d.dat", folder, i);
      file.open(filename, ios::out);
      Matrix* S = _scenes[i];
      vector<StrCartesianIndexPair> pairs = _pairs[i];
      for(unsigned int p=0; p<pairs.size(); p++)
      {
        unsigned int idxM = pairs[p].indexFirst;
        unsigned int idxS = pairs[p].indexSecond;

        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*_M)(idxM,j);
          file << coord << " ";
        }
        for(unsigned int j=0; j<_dim; j++)
        {
          double coord = (*S)(idxS,j);
          file << coord << " ";
        }
        file << endl;
      }
      file.close();
    }

    if(_dim==2)
    {
      snprintf(filename, 512, "%s/animate_trace.sh", folder);
      file.open(filename, ios::out);
      file << "#!/bin/bash" << endl << "echo \"clear\" > animate_trace.gpi" << endl;
      file << "echo \"reset\" >> animate_trace.gpi" << endl << "echo \"set terminal gif animate delay " << delay << "\" >> animate_trace.gpi" << endl;
      file << "echo \"set output \\\"animate_trace.gif\\\"\" >> animate_trace.gpi" << endl;
      file << "echo \"set isosample 40\" >> animate_trace.gpi" << endl;
      file << "echo \"set xrange [" << minCoord[0] << ":" << maxCoord[0] << "]\" >> animate_trace.gpi" << endl;
      file << "echo \"set yrange [" << minCoord[1] << ":" << maxCoord[1] << "]\" >> animate_trace.gpi" << endl;
      file << "for NR in `seq -f \"%05g\" 0 " << _scenes.size()-1 << "`" << endl;
      file << "do" << endl;
      file << "echo \"plot \\\"./model.dat\\\" u 1:2 w p pt 19 ps 1 t \\\"model\\\"";
      file << ", \\\"./scene_${NR}.dat\\\" u 1:2 w p pt 19 t \\\"scene\\\"";
      file << ", \\\"./pairs_${NR}.dat\\\" u 1:2 w p pt 20 t \\\"pairs (model)\\\"";
      file << "\" >> animate_trace.gpi" << endl;
      file << "done" << endl;
      file.close();

      LOGMSG(DBG_DEBUG, "Trace serialized, execute animation script for gnuplot visualization");
    }
    else
      LOGMSG(DBG_DEBUG, "Trace serialized, animation script not available for 3D data");
  }
  else
  {
    LOGMSG(DBG_ERROR, "Delete existing directory or choose a different name for trace recording");
  }
}

}

