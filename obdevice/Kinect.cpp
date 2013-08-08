#include "Kinect.h"
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

Kinect::Kinect(const char* path)
{
  _init   = false;
  _rows   = 0;
  _cols   = 0;
  _coords = NULL;
  _rgb    = NULL;
  _z      = NULL;
  _record = false;

  EnumerationErrors errors;
  ScriptNode snode;
  XnStatus status = _context.Init();//InitFromXmlFile(path, snode, &errors);

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

  static xn::NodeInfoList image_nodes;
  status = _context.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);
  if (status != XN_STATUS_OK)
    printf("Enumerating image generators failed: %s", xnGetStatusString (status));

  status = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depth);
  if(status) printf("Failed to find depth generator: %s\n", xnGetStatusString(status));
  status = _depth.Create(_context);
  if(status) printf("Failed to create depth generator: %s\n", xnGetStatusString(status));

  // Set it to VGA maps at 30 FPS
  XnMapOutputMode mapMode;
  mapMode.nXRes = XN_VGA_X_RES;
  mapMode.nYRes = XN_VGA_Y_RES;
  mapMode.nFPS = 30;
  status = _depth.SetMapOutputMode(mapMode);
  if(status) printf("Failed to set parameters to depth generator: %s\n", xnGetStatusString(status));

  status = _context.FindExistingNode(XN_NODE_TYPE_IMAGE, _image);
  if(status) printf("Failed to find image generator: %s\n", xnGetStatusString(status));
  status = _image.Create(_context);
  if(status) printf("Failed to create image generator: %s\n", xnGetStatusString(status));

  status = _context.FindExistingNode(XN_NODE_TYPE_IR, _ir);
  if(status) printf("Failed to find ir generator: %s\n", xnGetStatusString(status));
  //status = _ir.Create(_context);
  //if(status) printf("Failed to create ir generator: %s\n", xnGetStatusString(status));

// Check for night vision modus
  _useIR = true;
  if (status != XN_STATUS_OK)
  {
    _useIR = false;
  }

/*#ifdef __i386__
  status = _context.FindExistingNode(XN_NODE_TYPE_USER, _user);
  if (status != XN_STATUS_OK) status = _user.Create(_context);

  if(status != XN_STATUS_OK)
  {
     printf("Find user generator failed: %s\n", xnGetStatusString (status));
  }
  else
  {
    XnCallbackHandle hUserCallbacks;
    _user.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  }
#endif*/

  if(_useIR)
  {
    _depth.GetFrameSyncCap().FrameSyncWith(_ir);
    _depth.GetAlternativeViewPointCap().SetViewPoint(_ir);
  }
  else
  {
    if (_depth.GetFrameSyncCap ().CanFrameSyncWith (_image) && !_depth.GetFrameSyncCap ().IsFrameSyncedWith (_image))
    {
      status = _depth.GetFrameSyncCap().FrameSyncWith(_image);
      if (status != XN_STATUS_OK) cout << "could not turn on frame synchronization. Reason: " << xnGetStatusString (status) << endl;
    }
    _depth.GetAlternativeViewPointCap().SetViewPoint(_image);
  }

  _context.StartGeneratingAll();

  DepthMetaData depthMD;
  _depth.GetMetaData(depthMD);

  _rows    = depthMD.YRes();
  _cols    = depthMD.XRes();

  _proj    = new XnPoint3D[_rows*_cols];
  _wrld    = new XnPoint3D[_rows*_cols];

  _coords  = new double[_rows*_cols*3];
  _z       = new double[_rows*_cols];
  _rgb     = new unsigned char[_rows*_cols*3];
  _mask    = new bool[_rows*_cols];
  memset(_mask, 0, _rows*_cols*sizeof(*_mask));

  _useBilinearFilter = false;

  _init    = true;
}

Kinect::~Kinect()
{
  if(!_init) return;
  _context.StopGeneratingAll();
  _context.Release();
  delete [] _proj;
  delete [] _wrld;
  delete [] _coords;
  delete [] _z;
  delete [] _rgb;
  delete [] _mask;
}

bool Kinect::grab()
{
  if(_useIR) return grabIR();
  else return grabRGB();
}

bool Kinect::grabRGB()
{
  if(!_init) return false;

  DepthMetaData depthMD;
  ImageMetaData imageMD;

  _depth.GetMetaData(depthMD);
  _image.GetMetaData(imageMD);

  int retval = _depth.WaitAndUpdateData();
  if (retval != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(retval));
    return 0;
  }

  retval = _image.WaitAndUpdateData();

  //int nRetVal = _context.WaitOneUpdateAll(_depth);
  if (retval != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(retval));
    return 0;
  }

  cout << "ok " << endl;

 // if (!(_image.IsValid() & _depth.IsValid())) return false;

  const XnRGB24Pixel* pImageMap = imageMD.RGB24Data();

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

  if(_useBilinearFilter)
    filterBilinear(_mask, _z);

  for(size_t i=0; i<size; i++)
  {
    _proj[i].Z = _z[i];
  }

  cout << size << endl;

  _depth.ConvertProjectiveToRealWorld(size, _proj, _wrld);

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X;
    _coords[3*i+1] = _wrld[i].Y;
    _coords[3*i+2] = _wrld[i].Z;
    if(_coords[3*i]>0.01)
      cout << _coords[3*i] << endl;
    _rgb[3*i]      = 255;//pImageMap[i].nRed;
    _rgb[3*i+1]    = 255;//pImageMap[i].nGreen;
    _rgb[3*i+2]    = 255;//pImageMap[i].nBlue;
    _mask[i] = (!isnan(_z[i])) && (fabs(_coords[3*i+2])>10e-6);
  }

  if(_record)
  {
    for(size_t i=0; i<_rows*_cols; i++)
    {
      _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
      _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
      _recfile << (unsigned int)_rgb[3 * i] << " " << (unsigned int)_rgb[3 * i + 1] << " " << (unsigned int)_rgb[3 * i + 2] << endl;
    }
  }

  return true;
}

bool Kinect::grabIR()
{
  if(!_init) return false;

  DepthMetaData depthMD;
  IRMetaData irMD;

  _depth.GetMetaData(depthMD);
  _ir.GetMetaData(irMD);

  int nRetVal = _context.WaitAnyUpdateAll();
  if (nRetVal != XN_STATUS_OK)
    {
      printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
      return 0;
    }

  nRetVal = _depth.WaitAndUpdateData();
  if (nRetVal != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
    return 0;
  }

  nRetVal = _ir.WaitAndUpdateData();
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

  for (size_t r=0, i=0; r<_rows; r++)
  {
    for (size_t c=0; c<_cols; c++, i++)
    {
      _proj[i].X = (XnFloat) c;
      _proj[i].Y = (XnFloat) r;
      _proj[i].Z = depthMD(c, r);
      _z[i]      = depthMD(c, r);
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
    _rgb[3*i]      = pIrMap[i] * 255 / (maxval);
    _rgb[3*i+1]    = _rgb[3*i];
    _rgb[3*i+2]    = _rgb[3*i];
    _mask[i]       = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);

  }

  if(_record)
  {
    for(size_t i=0; i<_rows*_cols; i++)
    {
      _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
      _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
      _recfile << (unsigned int)_rgb[3 * i] << " " << (unsigned int)_rgb[3 * i + 1] << " " << (unsigned int)_rgb[3 * i + 2] << endl;
    }
  }

  return true;
}

void Kinect::startRecording(char* filename)
{
  _recfile.open(filename, ios::out);
  _recfile << _cols << " " << _rows << endl;
  _record = true;
}

void Kinect::stopRecording()
{
  _recfile.close();
  _record = false;
}

unsigned int Kinect::getRows()
{
  return _rows;
}

unsigned int Kinect::getCols()
{
  return _cols;
}

double* Kinect::getCoords()
{
  return _coords;
}

bool* Kinect::getMask()
{
  return _mask;
}

double* Kinect::getZ()
{
  return _z;
}

unsigned char* Kinect::getRGB()
{
  return _rgb;
}

MatD Kinect::getMatZ(void) const
{
    const double* z = _z;
    MatD mat(_rows, _cols);

    for (unsigned int row = 0; row < _rows; row++)
        for (unsigned int col = 0; col < _cols; col++)
            mat.at(row, col) = *z++;

    return mat;
}

MatRGB Kinect::getMatRGB(void) const
{
    const unsigned char* rgb = _rgb;
    MatRGB mat(_rows, _cols);

    for (unsigned int row = 0; row < _rows; row++)
    {
        for (unsigned int col = 0; col < _cols; col++)
        {
            mat.at(row, col, MatRGB::Red)   = *rgb++;
            mat.at(row, col, MatRGB::Green) = *rgb++;
            mat.at(row, col, MatRGB::Blue)  = *rgb++;
        }
    }

    return mat;
}

unsigned short* Kinect::getUserMask()
{
  if(_user.GetNumberOfUsers()<1)
  {
    cout << "no users found" << endl;
    return NULL;
  }
  _user.GetUserPixels(0, _sceneMD);

  double zmean              = 0.0;
  int valid_points          = 0;
  unsigned short* userMask  = (unsigned short*) _sceneMD.Data();

  for(int i=0; i<_rows*_cols; i++)
  {
    if(userMask[i])
    {
      zmean   += _z[i];
      valid_points++;
    }
  }
  if(valid_points > 100)
  {
    zmean /= valid_points;
    for(int i=0; i<_rows*_cols; i++)
    {
      if( fabs(_z[i]-zmean)> 300)
      {
        userMask[i] = 0;
      }
    }
  }

  return (unsigned short*) _sceneMD.Data();
}

void Kinect::useBilinearFilter(bool activate)
{
  _useBilinearFilter = activate;
}

// Experimental filter taken from: C. Tomasi and R. Manduchi, "Bilateral Filtering for Gray and Color Images", In Proceedings of the 1998 IEEE International Conference on Computer Vision, Bombay, India.
void Kinect::filterBilinear(bool* mask, double* z_filtered)
{
  DepthMetaData depthMD;
  unsigned int i,j;
  int k,l;
  int mini, maxi;
  int minj, maxj;
  unsigned int radius = 5;
  double val;
  double totalw;

  double distance_dim;
  double distance_range;
  double sigmad = 1.0/(3.0*3.0);
  double sigmar = 1.0/(30.0*30.0);
  double wd, wr;

  _depth.GetMetaData(depthMD);

  for (i = 0; i < _rows; i++)
  {
     mini = i - radius;
     maxi = i + radius;
     if (mini < 0) mini = 0;
     if (maxi >= _rows) maxi = _rows - 1;

     for (j = 0; j < _cols; j++)
     {
        minj = j - radius;
        maxj = j + radius;
        if (minj < 0) minj = 0;
        if (maxj >= _cols) maxj = _cols - 1;

        if (!mask[i*_cols+j]) continue;

        val = 0;
        totalw = 0;

        double depthCenter = depthMD(j, i);
        for (k = mini; k <= maxi; k++)
        {
          double distRow2 = (i - k)*(i - k);
           for (l = minj; l <= maxj; l++)
           {
              if (!mask[k*_cols+l]) continue;
              double depthNeighbor = depthMD(l, k);

              distance_dim   = distRow2 + (j - l)*(j - l) * sigmad;
              distance_range = (depthCenter - depthNeighbor)*(depthCenter - depthNeighbor) * sigmar;

              double w = exp(-0.5 * (distance_dim + distance_range));

              val += w * depthNeighbor;
              totalw += w;
           }
        }

        val /= totalw;
        z_filtered[i*_cols+j] = val;
     }
  }
}


} // end namespace
