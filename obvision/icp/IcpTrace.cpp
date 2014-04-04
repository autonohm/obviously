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
}
		 
IcpTrace::~IcpTrace()
{

}
	
void IcpTrace::reset()
{
  for(unsigned int i=0; i<_models.size(); i++)
    delete  _models[i];
  _models.clear();

  for(unsigned int i=0; i<_scenes.size(); i++)
    delete _scenes[i];
  _scenes.clear();

  for(unsigned int i=0; i<_pairs.size(); i++)
      _pairs[i].clear();
  _pairs.clear();
}

void IcpTrace::addAssignment(double** model, unsigned int sizeM, double** scene, unsigned int sizeS, vector<StrCartesianIndexPair> pairs)
{
  Matrix* m = new Matrix(sizeM, _dim, *model);
  _models.push_back(m);

  Matrix* s = new Matrix(sizeS, _dim, *scene);
  _scenes.push_back(s);

  _pairs.push_back(pairs);
}

void IcpTrace::serialize(char* folder)
{
  char cmd[256];
  sprintf(cmd, "mkdir %s", folder);
  int retval = system(cmd);
  if(retval==0)
  {
    ofstream file;
    char filename[512];
    for(unsigned int i=0; i<_models.size(); i++)
    {
      snprintf(filename, 512, "%s/trace_%05d.dat", folder, i);
      file.open(filename, ios::out);
      Matrix* M = _models[i];
      Matrix* S = _scenes[i];
      for(unsigned int p=0; p<M->getRows(); p++)
      {
        for(unsigned int j=0; j<_dim; j++)
          file << (*M)(p,j) << " ";
        for(unsigned int j=0; j<_dim; j++)
          file << (*S)(p,j) << " ";
        file << endl;
      }
      file.close();
    }

    snprintf(filename, 512, "%s/animate_trace.sh", folder);
    file.open(filename, ios::out);
    file << "#!/bin/bash" << endl << "echo \"clear\" > plot.gpi" << endl;
    file << "echo \"reset\" >> plot.gpi" << endl << "echo \"set terminal gif animate delay 10\" >> plot.gpi" << endl;
    file << "echo \"set output \\\"animate.gif\\\"\" >> plot.gpi" << endl;
    file << "echo \"set isosample 40\" >> plot.gpi" << endl;
    file << "echo \"set autoscale fix\" >> plot.gpi" << endl;
    file << "for FILE in trace*.dat" << endl;
    file << "do" << endl;
    file << "echo \"plot \\\"./${FILE}\\\" u 1:2 w p t \\\"model\\\", \\\"./${FILE}\\\" u 3:4 w p t \\\"scene\\\"\" >> plot.gpi" << endl;
    file << "done" << endl;
    file.close();

    LOGMSG(DBG_DEBUG, "Trace serialized, execute animation script for gnuplot visualization");
  }
  else
  {
    LOGMSG(DBG_ERROR, "Delete existing directory or choose a different name for trace recording");
  }
}

}

