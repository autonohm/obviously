/**
* @file     OpenNIDevice.cpp
* @author   Christian Pfitzner
* @date     29.12.2012
*
*
*/

#include "obdevice/OpenNIDevice.h"
#include <obcore/math/mathbase.h>
#include "obcore/math/linalg/linalg.h"
#include <math.h>

using namespace obvious;

OpenNIDevice::OpenNIDevice(const char* path)
  : ParentDevice3D()
{
  _init   = false;
  _record = false;
  _path   = path;

  EnumerationErrors errors;
  ScriptNode snode;
  XnStatus status = _context.InitFromXmlFile(_path, snode, &errors);

  if (status == XN_STATUS_NO_NODE_PRESENT)
  {
    XnChar strError[1024];
    errors.ToString(strError, 1024);
    printf("%s\n", strError);
    return;
  }
  else if (status != XN_STATUS_OK)
  {
    printf("Open failed: %s\n", xnGetStatusString(status));
    return;
  }

  static xn::NodeInfoList depth_nodes;
  status = _context.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);
  if (status != XN_STATUS_OK)
    printf("Enumerating depth generators failed: %s", xnGetStatusString (status));

  bool noColor = false;
  static xn::NodeInfoList image_nodes;
  status = _context.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);
  if (status != XN_STATUS_OK)
  {
    printf("Enumerating image generators failed: %s", xnGetStatusString (status));
    noColor = true;
  }

  if (!noColor)
  {
    std::cout << "device has no color" << std::endl;
    status = _context.FindExistingNode(XN_NODE_TYPE_IMAGE, _image);
  }
  status = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depth);
  status = _context.FindExistingNode(XN_NODE_TYPE_IR, _ir);

// Check for night vision modus
  _useIR = true;
  if (status != XN_STATUS_OK)
    _useIR = false;


  _context.StartGeneratingAll();

  if(_useIR)
  {
    _depth.GetFrameSyncCap().FrameSyncWith(_ir);
    _depth.GetAlternativeViewPointCap().SetViewPoint(_ir);
  }

  DepthMetaData depthMD;
  _depth.GetMetaData(depthMD);

  _rows    = depthMD.YRes();
  _cols    = depthMD.XRes();

  _proj    = new XnPoint3D[_rows*_cols];
  _wrld    = new XnPoint3D[_rows*_cols];

  _coords  = new double[_rows*_cols*3];
  _z       = new double[_rows*_cols];
  _mask    = new bool[_rows*_cols];
  _rgb     = new unsigned char[_rows*_cols*3];

  memset(_mask, 0, _rows*_cols*sizeof(*_mask));
  _init    = true;
}

OpenNIDevice::~OpenNIDevice()
{
  if(!_init)
    return;
  _context.StopGeneratingAll();
  _context.Release();
  delete [] _proj;
  delete [] _wrld;
}

//~~~~~~~~~~~~~~~~~~~~ GRAB ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OpenNIDevice::grabDistance(void)
{
  if(!_init)
    return false;

  DepthMetaData depthMD;
  _depth.GetMetaData(depthMD);

  int nRetVal = _context.WaitAnyUpdateAll();
  _depth.WaitAndUpdateData();

  if (nRetVal != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
    return 0;
  }

  XnUInt32 size = _rows*_cols;

  xnOSMemSet(_proj, 0, size * sizeof(XnPoint3D));
  xnOSMemSet(_wrld, 0, size * sizeof(XnPoint3D));

  for (size_t r=0, i=0; r<_rows; r++) {
    for (size_t c=0; c<_cols; c++, i++)
    {
      _proj[i].X = (XnFloat) c;
      _proj[i].Y = (XnFloat) r;
      _z[i]      = depthMD(c, r);
    }
  }

  for(size_t i=0; i<size; i++)
    _proj[i].Z = _z[i];

  _depth.ConvertProjectiveToRealWorld(size, _proj, _wrld);

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X / 1000.0;
    _coords[3*i+1] = _wrld[i].Y / 1000.0;
    _coords[3*i+2] = _wrld[i].Z / 1000.0;
    _rgb[3*i]    = 0;
    _rgb[3*i+1]  = 0;
    _rgb[3*i+2]  = 0;
    _mask[i] = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);
  }

  if(_record)
    record();

  return true;
}

bool OpenNIDevice::grabIR(void)
{
  if(!_init)
    return false;
  DepthMetaData depthMD;
  IRMetaData irMD;

  _depth.GetMetaData(depthMD);
  _ir.GetMetaData(irMD);


  int nRetVal = _context.WaitAnyUpdateAll();
  _depth.WaitAndUpdateData();
  _ir.WaitAndUpdateData();

  if (nRetVal != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
    return 0;
  }

  if (!(_ir.IsValid() & _depth.IsValid())) return false;

  const XnIRPixel* pIrMap = irMD.Data();

  XnUInt32 size = _rows*_cols;

  xnOSMemSet(_proj, 0, size * sizeof(XnPoint3D));
  xnOSMemSet(_wrld, 0, size * sizeof(XnPoint3D));

  for (size_t r=0, i=0; r<_rows; r++) {
    for (size_t c=0; c<_cols; c++, i++)
    {
      _proj[i].X = (XnFloat) c;
      _proj[i].Y = (XnFloat) r;
      _proj[i].Z = depthMD(c, r);
    }
  }

  _depth.ConvertProjectiveToRealWorld(size, _proj, _wrld);

  int maxval = 0;
  for (size_t i=0; i<size; i++) {
    if(pIrMap[i] > maxval)
      maxval = pIrMap[i];
  }

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X / 1000.0;
    _coords[3*i+1] = _wrld[i].Y / 1000.0;
    _coords[3*i+2] = _wrld[i].Z / 1000.0;
    _rgb[3*i]      = pIrMap[i] * 255 / (maxval);
    _rgb[3*i+1]    = pIrMap[i] * 255 / (maxval);
    _rgb[3*i+2]    = pIrMap[i] * 255 / (maxval);
    _mask[i]       = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);
  }

  if(_record)
    record();

  return true;
}

void OpenNIDevice::record()
{
  for(size_t i=0; i<_rows*_cols; i++)
  {
    _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
    _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
    _recfile << (unsigned int)_rgb[3 * i] << " " << (unsigned int)_rgb[3 * i + 1] << " " << (unsigned int)_rgb[3 * i + 2] << endl;
  }
}




