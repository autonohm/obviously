#include "UvcCam.h"

/**
 * Initial implementation taken from openvolksbot library
 */

namespace obvious
{
struct buffer
{
  void * start;
  size_t length;
};

#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
#define R_FROMYV(y,v)  CLIP((y) + _LutRv[(v)])
#define G_FROMYUV(y,u,v) CLIP((y) + _LutGu[(u)] + _LutGv[(v)])
#define B_FROMYU(y,u) CLIP((y) + _LutBu[(u)])
#define CoefRv 1402
#define CoefGu 714 // 344
#define CoefGv 344 // 714
#define CoefBu 1772

UvcCam::UvcCam(const char *dev, unsigned int width, unsigned int height)
{
  _nDeviceHandle = -1;
  _dev           = dev;
  _width         = width;
  _height        = height;
  _colorMode     = CAMRGB;
  _LutRv         = (int *) malloc(256 * sizeof(int));
  _LutGu         = (int *) malloc(256 * sizeof(int));
  _LutGv         = (int *) malloc(256 * sizeof(int));
  _LutBu         = (int *) malloc(256 * sizeof(int));
  _nb_buffers    = NB_BUFFERS;
  for (int i = 0; i < 256; i++)
  {
    _LutRv[i] = (i - 128) * CoefRv / 1000;
    _LutGu[i] = (128 - i) * CoefGu / 1000;
    _LutGv[i] = (128 - i) * CoefGv / 1000;
    _LutBu[i] = (i - 128) * CoefBu / 1000;
  }
}

UvcCam::~UvcCam()
{
  disconnect();
}

unsigned int UvcCam::getWidth()
{
  return _width;
}

unsigned int UvcCam::getHeight()
{
  return _height;
}

void UvcCam::setColorMode(EnumCameraColorMode mode)
{
  _colorMode = mode;
}

EnumCameraColorMode UvcCam::getColorMode()
{
  return _colorMode;
}

unsigned int UvcCam::getChannels()
{
  if(_colorMode == CAMRGB)
    return 3;
  else if(_colorMode == CAMGRAYSCALE)
    return 1;
  else
  {
    cout << "UvcCam::getChannels(): Unknown color mode" << endl;
    return 0;
  }
}

EnumCameraError UvcCam::connect()
{

  if (_nDeviceHandle != -1)
  {
    cout << "UvcCam::connect(): Device handle already exists. A new connection will not be established." << endl;
    return CAMFAILURE;
  }

  if ((_nDeviceHandle = open(_dev, O_RDWR)) == -1)
  {
    cout << "UvcCam::connect(): Could not open device V4L2." << endl;
    return CAMERRORINIT;
  }

  struct v4l2_capability cap;
  CLEAR(cap);
  if (ioctl(_nDeviceHandle, VIDIOC_QUERYCAP, &cap) < 0)
  {
    cout << "UvcCam::connect(): unable to query device." << endl;
    return CAMERRORINIT;
  }
  if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0)
  {
    cout << "UvcCam::connect(): video capture not supported." << endl;
    return CAMERRORINIT;
  }

  setResolution(_width, _height);

  if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_BRIGHTNESS) < 0) printf("reset Brightness error\n");
  if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_SATURATION) < 0) printf("reset Saturation error\n");
  if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_CONTRAST) < 0)   printf("reset Contrast error\n");
  if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_SHARPNESS) < 0)  printf("reset Sharpness error\n");
  if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_GAIN) < 0)       printf("reset Gain error\n");

  return CAMSUCCESS;
}

EnumCameraError UvcCam::disconnect()
{
  if(_nDeviceHandle != -1)
  {
    stopStreaming();
    close (_nDeviceHandle);
    _nDeviceHandle = -1;
  }
  return CAMSUCCESS;
}

EnumCameraError UvcCam::setResolution(unsigned int width, unsigned int height)
{
  if (_nDeviceHandle == -1)
  {
    cout << "UvcCam::setResolution: Trying to set resolution of not initialized camera device." << endl;
    return CAMERRORINIT;
  }

  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = width;
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  int retval = ioctl(_nDeviceHandle, VIDIOC_S_FMT, &fmt);
  if (retval < 0)
  {
    cout << "UvcCam::setResolution(): Unable to set format." << endl;
    return CAMERRORINIT;
  }
  if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
  {
    cout << "UvcCam::setResolution(): desired format unavailable" << endl;
    cout << " next possible format - width: " << fmt.fmt.pix.width << ", height: " << fmt.fmt.pix.height << endl;
  }
  _width = width;
  _height = height;

  return CAMSUCCESS;
}

EnumCameraError UvcCam::setFramerate(unsigned int numerator, unsigned int denominator)
{
  if (_nDeviceHandle == -1)
  {
    cout << "UvcCam::setFramerate: Trying to set framerate of not initialized camera device." << endl;
    return CAMERRORINIT;
  }

  struct v4l2_streamparm setfps;
  setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps.parm.capture.timeperframe.numerator   = numerator;
  setfps.parm.capture.timeperframe.denominator = denominator;
  if (ioctl(_nDeviceHandle, VIDIOC_S_PARM, &setfps) < 0)
  {
    cout << "UvcCam::setFramerate(): Unable to set fps" << endl;
    return CAMERRORINIT;
  }
  return CAMSUCCESS;
}

EnumCameraError UvcCam::startStreaming()
{
  if (_nDeviceHandle == -1)
  {
    cout << "UvcCam::startStreaming: Trying to start streaming of not initialized camera device." << endl;
    return CAMERRORINIT;
  }

  EnumCameraError retval = mapMemory();
  if(retval != CAMSUCCESS) return retval;

  /* Queue the buffers. */
  for (int i = 0; i < _nb_buffers; i++)
  {
    CLEAR(_buf);
    _buf.index = i;
    _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    _buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(_nDeviceHandle, VIDIOC_QBUF, &_buf) < 0)
    {
      cout << "UvcCam::startStreaming(): Unable to queue buffer" << endl;
      return CAMERRORINIT;
    }
  }

  //video enable
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int err = ioctl(_nDeviceHandle, VIDIOC_STREAMON, &type);
  while(err < 0)
  {
    err = ioctl(_nDeviceHandle, VIDIOC_STREAMON, &type);
    cout << "UvcCam::startStreaming(): Unable to enable video capture" << endl;
  }

  CLEAR(_buf);
  _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  _buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(_nDeviceHandle, VIDIOC_DQBUF, &_buf) < 0)
  {
    cout << "UvcCam::startStreaming(): Unable to dequeue buffer" << endl;
    return CAMFAILURE;
  }

  return CAMSUCCESS;
}

EnumCameraError UvcCam::stopStreaming()
{
  if (_nDeviceHandle == -1)
  {
    cout << "UvcCam::stopStreaming: Trying to stop streaming of not initialized camera device." << endl;
    return CAMERRORINIT;
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_nDeviceHandle, VIDIOC_STREAMOFF, &type) < 0)
  {
    cout << "UvcCam::stopStreaming(): Unable to stop streaming" << endl;
  }
  unmapMemory();
  return CAMSUCCESS;
}

EnumCameraError UvcCam::grab(unsigned char* image)
{
  if (_nDeviceHandle == -1)
  {
    cout << "UvcCam::grab: Trying to grab image on not initialized camera device." << endl;
    return CAMERRORINIT;
  }

  if (ioctl(_nDeviceHandle, VIDIOC_QBUF, &_buf) < 0)
  {
    cout << "UvcCam::grab :Unable to requeue buffer" << endl;
    return CAMFAILURE;
  }

  CLEAR(_buf);
  _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  _buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(_nDeviceHandle, VIDIOC_DQBUF, &_buf) < 0)
  {
    cout << "UvcCam::grab(): Unable to dequeue buffer" << endl;
    return CAMFAILURE;
  }

  if (_buf.bytesused > 0)
  {
    if(_colorMode==CAMRGB)
      Pyuv422torgb24((unsigned char *) (_mem[_buf.index].start), image, _width, _height);
    else
      Pyuv422togray((unsigned char *) (_mem[_buf.index].start), image, _width, _height);

    return CAMSUCCESS;
  }

  return CAMGRABBING;
}

EnumCameraError UvcCam::printAvailableFormats()
{

  EnumCameraError retval = connect();
  if(retval!=CAMSUCCESS) return retval;

  int ret;

  struct v4l2_fmtdesc desc;
  CLEAR(desc);
  desc.index = 0;
  desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while ((ret = ioctl(_nDeviceHandle, VIDIOC_ENUM_FMT, &desc)) == 0)
  {
    desc.index++;
    printf("pixelformat = '%c%c%c%c', description = '%s'\n",
        desc.pixelformat & 0xFF,
        (desc.pixelformat >> 8) & 0xFF,
        (desc.pixelformat >> 16) & 0xFF,
        (desc.pixelformat >> 24) & 0xFF,
        desc.description);
  }

  struct v4l2_format fmt;
  int res[32] = { 90,  120,  144,  160,  176,  240,  272,  288,  320,  352,
                 480,  544,  576,  600,  640,  704,  720,  768,  800,  960,
                 1024, 1080, 1152, 1280, 1366, 1400, 1440, 1600, 1900, 1920,
                 2048, 2560 };

  CLEAR(fmt);
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;

  cout << "supported resolutions" << endl;
  // some combinations are not meaningful, but for a simple implementation the brute force check is sufficient.
  for(int i=0; i<32; i++)
  {
    for(int j=0; j<32; j++)
    {
      fmt.fmt.pix.width       = res[i];
      fmt.fmt.pix.height      = res[j];
      ret = ioctl(_nDeviceHandle, VIDIOC_S_FMT, &fmt);
      if(ret==0)
      {
        if ((fmt.fmt.pix.width == res[i]) && (fmt.fmt.pix.height == res[j]))
        {
          cout << res[i] << "x" << res[j] << endl;
        }
      }
      else
      {
        cout << "unable to query format" << endl;
      }
    }
  }

  disconnect();
  return CAMSUCCESS;
}

EnumCameraError UvcCam::mapMemory()
{
  CLEAR(_rb);
  _rb.count  = _nb_buffers;
  _rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  _rb.memory = V4L2_MEMORY_MMAP;
  if (ioctl(_nDeviceHandle, VIDIOC_REQBUFS, &_rb) < 0)
  {
    cout << "UvcCam::mapMemory(): Unable to allocate buffers ." << endl;
    return CAMERRORINIT;
  }
  _mem = (buffer *) calloc(_nb_buffers, sizeof(*_mem));

  // map the buffers
  for (int i = 0; i < _nb_buffers; i++)
  {
    CLEAR(_buf);
    _buf.index = i;
    _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    _buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(_nDeviceHandle, VIDIOC_QUERYBUF, &_buf) < 0)
    {
      cout << "UvcCam::mapMemory(): Unable to query buffer" << endl;
      return CAMERRORINIT;
    }
    _mem[i].length = _buf.length;
    _mem[i].start = mmap(0 , _buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _nDeviceHandle, _buf.m.offset);
    if (_mem[i].start == MAP_FAILED)
    {
      cout << "UvcCam::mapMemory(): Unable to map buffer" << endl;
      return CAMERRORINIT;
    }
  }
  return CAMSUCCESS;
}

void UvcCam::unmapMemory()
{
  for(int i = 0; i < _rb.count; i++)
    munmap(_mem[i].start, _mem[i].length);
}

// aus lucview color.c util.c
unsigned int UvcCam::Pyuv422torgb24(unsigned char * input_ptr, unsigned char * output_ptr, unsigned int image_width, unsigned int image_height)
{
  unsigned int i, size;
  unsigned char Y, Y1, U, V;
  unsigned char *buff = input_ptr;
  unsigned char *output_pt = output_ptr;
  size = image_width * image_height / 2;
  for (i = size; i > 0; i--)
  {
    /* bgr instead rgb ?? */
    Y  = buff[0];
    U  = buff[1];
    Y1 = buff[2];
    V  = buff[3];
    buff += 4;
    *output_pt++ = R_FROMYV(Y,V);
    *output_pt++ = G_FROMYUV(Y,U,V); //b
    *output_pt++ = B_FROMYU(Y,U); //v

    *output_pt++ = R_FROMYV(Y1,V);
    *output_pt++ = G_FROMYUV(Y1,U,V); //b
    *output_pt++ = B_FROMYU(Y1,U); //v
  }

  return 1;
}

unsigned int UvcCam::Pyuv422togray(unsigned char* input_ptr, unsigned char* output_ptr, unsigned int image_width, unsigned int image_height)
{
  unsigned int i, size;
  unsigned char Y, Y1;
  unsigned char *buff = input_ptr;
  unsigned char *output_pt = output_ptr;
  size = image_width * image_height / 2;
  for (i = size; i > 0; i--)
  {
    Y  = buff[0];
    Y1 = buff[2];
    buff += 4;
    *output_pt++ = Y;
    *output_pt++ = Y1;
  }

  return 1;
}

/* return >= 0 ok otherwhise -1 */
int UvcCam::isv4l2Control(int nHandle, int control, struct v4l2_queryctrl *queryctrl)
{
  int err = 0;
  queryctrl->id = control;
  if ((err = ioctl(nHandle, VIDIOC_QUERYCTRL, queryctrl)) < 0)
  {
    cout << "ioctl querycontrol error: " << errno << endl;
  }
  else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED)
  {
    cout << "control " << (char *) queryctrl->name << " disabled" << endl;
  }
  else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN)
  {
    return 1;
  }
  else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER)
  {
    return 0;
  }
  else
  {
    cout << "contol " << (char *) queryctrl->name << " unsupported" << endl;
  }
  return -1;
}

int UvcCam::v4l2GetControl(int nHandle, int control)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_control control_s;
  int err;
  if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
  control_s.id = control;
  if ((err = ioctl(nHandle, VIDIOC_G_CTRL, &control_s)) < 0)
  {
    cout << "ioctl get control error" << endl;
    return -1;
  }
  return control_s.value;
}

int UvcCam::v4l2SetControl(int nHandle, int control, int value)
{
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int min, max, step, val_def;
  int err;
  if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
  min     = queryctrl.minimum;
  max     = queryctrl.maximum;
  step    = queryctrl.step;
  val_def = queryctrl.default_value;
  if ((value >= min) && (value <= max))
  {
    control_s.id = control;
    control_s.value = value;
    if ((err = ioctl(nHandle, VIDIOC_S_CTRL, &control_s)) < 0)
    {
      cout << "ioctl set control error" << endl;
      return -1;
    }
  }
  return 0;
}

int UvcCam::v4l2ResetControl(int nHandle, int control)
{
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int val_def;
  int err;
  if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
  val_def         = queryctrl.default_value;
  control_s.id    = control;
  control_s.value = val_def;
  if ((err = ioctl(nHandle, VIDIOC_S_CTRL, &control_s)) < 0)
  {
    cout << "ioctl reset control error" << endl;
    return -1;
  }

  return 0;
}

}

