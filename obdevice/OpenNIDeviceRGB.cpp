/**
* @file   OpenNIDeviceRGB.cpp
* @author Christian Pfitzner
* @date   30.12.2012
*
*
*/

#include "obdevice/OpenNIDeviceRGB.h"

using namespace obvious;

OpenNIDeviceRGB::OpenNIDeviceRGB()
{
  _rgb    = NULL;

  EnumerationErrors errors;
  ScriptNode        snode;
  XnStatus          status = _context.InitFromXmlFile(_path, snode, &errors);

  static xn::NodeInfoList image_nodes;
  status = _context.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);
  if (status != XN_STATUS_OK)
    printf("Enumerating image generators failed: %s", xnGetStatusString (status));

  _rgb     = new unsigned char[_rows*_cols*3];
}

OpenNIDeviceRGB::~OpenNIDevice()
{
  delete [] _rgb;
}

bool OpenNIDeviceRGB::grabRGB()
{
  if(!_init)
    return false;

  DepthMetaData depthMD;
  ImageMetaData imageMD;

  _depth.GetMetaData(depthMD);
  _image.GetMetaData(imageMD);

  int nRetVal = _context.WaitAnyUpdateAll();
  _depth.WaitAndUpdateData();

  if (nRetVal != XN_STATUS_OK)
  {
    printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
    return 0;
  }

  if (!(_image.IsValid() & _depth.IsValid())) return false;

  const XnRGB24Pixel* pImageMap = imageMD.RGB24Data();

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

//    if(_useBilinearFilter)
//      filterBilinear(_mask, _z);

  for(size_t i=0; i<size; i++)
    _proj[i].Z = _z[i];

  _depth.ConvertProjectiveToRealWorld(size, _proj, _wrld);

  for (size_t i=0; i<size; i++)
  {
    _coords[3*i]   = _wrld[i].X / 1000.0;
    _coords[3*i+1] = _wrld[i].Y / 1000.0;
    _coords[3*i+2] = _wrld[i].Z / 1000.0;
    _rgb[3*i]      = pImageMap[i].nRed;
    _rgb[3*i+1]    = pImageMap[i].nGreen;
    _rgb[3*i+2]    = pImageMap[i].nBlue;
    _mask[i] = (!isnan(_z[i])) && (_coords[3*i+2]>10e-6);
  }

  if(_record)
    record();

  return true;
}

MatRGB OpenNIDeviceRGB::getMatRGB(void) const
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

void OpenNIDeviceRGB::record(void)
{
  for(size_t i=0; i<_rows*_cols; i++)
  {
    _recfile << _proj[i].X << " "  << _proj[i].Y << " "  << _proj[i].Z << " ";
    _recfile << _coords[3 * i] << " " << _coords[3 * i + 1] << " " << _coords[3 * i + 2] << " ";
    _recfile << (unsigned int)_rgb[3 * i] << " " << (unsigned int)_rgb[3 * i + 1] << " " << (unsigned int)_rgb[3 * i + 2] << endl;
  }
}





