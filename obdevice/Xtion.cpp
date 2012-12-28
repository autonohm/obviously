/**
* @file Xtion.cpp
* @autor christian
* @date  23.12.2012
*
*
*/

#include "Xtion.h"
#include <obcore/math/mathbase.h>
#include <obcore/math/Matrix.h>
#include "math.h"

namespace obvious
{

UserGenerator _user;
SceneMetaData _sceneMD;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("New User %d\n", nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("Lost user %d\n", nId);
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  printf("Pose %s detected for user %d\n", strPose, nId);
}

Xtion::Xtion(const char* path)
{
  _init   = false;
  _rows   = 0;
  _cols   = 0;
  _coords = NULL;
  _z      = NULL;
  _record = false;

  EnumerationErrors errors;
  ScriptNode snode;
  XnStatus status = _context.InitFromXmlFile(path, snode, &errors);

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

  status = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depth);
  status = _context.FindExistingNode(XN_NODE_TYPE_IR, _ir);

// Check for night vision modus
  _useIR = false;
//  if (status != XN_STATUS_OK)
//  {
//    _useIR = false;
//  }

//#ifdef __i386__
//  status = _context.FindExistingNode(XN_NODE_TYPE_USER, _user);
//  if (status != XN_STATUS_OK) status = _user.Create(_context);
//
//  if(status != XN_STATUS_OK)
//  {
//     printf("Find user generator failed: %s\n", xnGetStatusString (status));
//  }
//  else
//  {
//    XnCallbackHandle hUserCallbacks;
//    _user.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
//  }
//#endif

  _context.StartGeneratingAll();

//  if(_useIR)
//  {
//    _depth.GetFrameSyncCap().FrameSyncWith(_ir);
//    _depth.GetAlternativeViewPointCap().SetViewPoint(_ir);
//  }
//  else
//  {
//    _depth.GetFrameSyncCap().FrameSyncWith(_image);
//    _depth.GetAlternativeViewPointCap().SetViewPoint(_image);
//  }

  DepthMetaData depthMD;
  _depth.GetMetaData(depthMD);

  _rows    = depthMD.YRes();
  _cols    = depthMD.XRes();

  std::cout << "Rows: " << _rows << std::endl;
  std::cout << "Cols: " << _cols << std::endl;

  _proj    = new XnPoint3D[_rows*_cols];
  _wrld    = new XnPoint3D[_rows*_cols];

  _coords  = new double[_rows*_cols*3];
  _z       = new double[_rows*_cols];
  _mask    = new bool[_rows*_cols];
  memset(_mask, 0, _rows*_cols*sizeof(*_mask));

  _init    = true;
}

Xtion::~Xtion()
{
  if(!_init) return;

  _context.Release();
  delete [] _proj;
  delete [] _wrld;
  delete [] _coords;
  delete [] _z;
  delete [] _mask;
}

bool Xtion::grab()
{
  if(_useIR)
    return grabIR();
  else
    return grabDistance();
}


bool Xtion::grabDistance()
{
  if(!_init) return false;

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

  for (size_t r=0, i=0; r<_rows; r++)
  {
    for (size_t c=0; c<_cols; c++, i++)
    {
      _proj[i].X = (XnFloat) c;
      _proj[i].Y = (XnFloat) r;
      _z[i]      = depthMD(c, r);
    }
  }


  for(size_t i=0; i<size; i++)
  {
    _proj[i].Z = _z[i];
  }

  _depth.ConvertProjectiveToRealWorld(size, _proj, _wrld);

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X / 1000.0;
    _coords[3*i+1] = _wrld[i].Y / 1000.0;
    _coords[3*i+2] = _wrld[i].Z / 1000.0;
    _mask[i] = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);
  }

  if(_record)
  {
    for(size_t i=0; i<_rows*_cols; i++)
    {
      _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
      _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
    }
  }

  return true;
}

bool Xtion::grabIR()
{
  if(!_init) return false;

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
  for (size_t i=0; i<size; i++)
  {
    if(pIrMap[i] > maxval) maxval = pIrMap[i];
  }

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X / 1000.0;
    _coords[3*i+1] = _wrld[i].Y / 1000.0;
    _coords[3*i+2] = _wrld[i].Z / 1000.0;
    _mask[i]       = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);

  }

  if(_record)
  {
    for(size_t i=0; i<_rows*_cols; i++)
    {
      _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
      _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
    }
  }
  return true;
}

void Xtion::startRecording(char* filename)
{
  _recfile.open(filename, ios::out);
  _recfile << _cols << " " << _rows << endl;
  _record = true;
}

void Xtion::stopRecording()
{
  _recfile.close();
  _record = false;
}

unsigned int Xtion::getRows()
{
  return _rows;
}

unsigned int Xtion::getCols()
{
  return _cols;
}

double* Xtion::getCoords()
{
  return _coords;
}

bool* Xtion::getMask()
{
  return _mask;
}

double* Xtion::getZ()
{
  return _z;
}

MatD Xtion::getMatZ(void) const
{
    const double* z = _z;
    MatD mat(_rows, _cols);

    for (unsigned int row = 0; row < _rows; row++)
        for (unsigned int col = 0; col < _cols; col++)
            mat.at(row, col) = *z++;

    return mat;
}

} // namespace




