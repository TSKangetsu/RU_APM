#include "Drive_V4L2Reader.hpp"

V4L2Tools::V4L2Drive::V4L2Drive(std::string Device, V4l2Info Info)
{
    v4l2d = Info;
    _flag_TargetDevice = Device;
    _flag_CameraFD = open(_flag_TargetDevice.c_str(), O_RDWR | O_NONBLOCK, 0);
    memset(&v4l2.CameraInfo, 0, sizeof(v4l2.CameraInfo));
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QUERYCAP, &v4l2.CameraInfo), _v4l2_query_error);
#ifdef DEBUG
    std::cout << "DriveInfo: \n"
              << "  Drive      :" << v4l2.CameraInfo.driver << "\n"
              << "  Card       :" << v4l2.CameraInfo.card << "\n"
              << "  Bus        :" << v4l2.CameraInfo.bus_info << "\n"
              << "  Version    :" << v4l2.CameraInfo.version << "\n"
              << "  Capbilities:" << v4l2.CameraInfo.capabilities << "\n";
#endif

    if (v4l2d.PixFormat == V4L2_PIX_FMT_H264)
    {
        V4L2Control(V4L2_CID_MPEG_VIDEO_BITRATE_MODE, V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
        V4L2Control(V4L2_CID_MPEG_VIDEO_H264_PROFILE, v4l2d.H264_Profile);
        V4L2Control(V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, v4l2d.H264_PSize);
        V4L2Control(V4L2_CID_MPEG_VIDEO_BITRATE, v4l2d.H264_Bitrate);
        V4L2Control(V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER, v4l2d.H264_EnablePPS);
        V4L2Control(V4L2_CID_MPEG_VIDEO_H264_LEVEL, V4L2_MPEG_VIDEO_H264_LEVEL_5_0);
        V4L2Control(V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 16);
        V4L2Control(V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 32);
    }

    if (v4l2d.Is_AutoSize)
    {
        int MaxWidth = 0;
        int MaxHeight = 0;
        memset(&v4l2.CameraFMTInfo, 0, sizeof(v4l2.CameraFMTInfo));
        v4l2.CameraFMTInfo.index = 0;
        v4l2.CameraFMTInfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (ioctl(_flag_CameraFD, VIDIOC_ENUM_FMT, &v4l2.CameraFMTInfo) >= 0)
        {
            v4l2.CameraFMTInfo.index++;
            v4l2.CameraFMTInfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (v4l2.CameraFMTInfo.pixelformat == v4l2d.PixFormat)
            {
                v4l2_frmsizeenum _FrameSize;
                memset(&_FrameSize, 0, sizeof(_FrameSize));
                _FrameSize.index = 0;
                _FrameSize.pixel_format = v4l2.CameraFMTInfo.pixelformat;
                while (ioctl(_flag_CameraFD, VIDIOC_ENUM_FRAMESIZES, &_FrameSize) >= 0)
                {
                    _FrameSize.index++;
                    MaxHeight = MaxHeight < _FrameSize.discrete.height ? _FrameSize.discrete.height : MaxHeight;
                    MaxWidth = MaxWidth < _FrameSize.discrete.width ? _FrameSize.discrete.width : MaxWidth;
                }
            }
        };
        v4l2d.ImgWidth = MaxWidth;
        v4l2d.ImgHeight = MaxHeight;
#ifdef DEBUG
        std::cout << "AutoSet is Comfirm:" << MaxWidth << "," << MaxHeight << "\n";
#endif
    }
    memset(&v4l2.CameraFormat, 0, sizeof(v4l2.CameraFormat));
    v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormat), _v4l2_vidioc_g_error);
    //
    v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.CameraFormat.fmt.pix.width = v4l2d.ImgWidth;
    v4l2.CameraFormat.fmt.pix.height = v4l2d.ImgHeight;
    v4l2.CameraFormat.fmt.pix.pixelformat = v4l2d.PixFormat;
    v4l2.CameraFormat.fmt.pix.field = V4L2_FIELD_ANY;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_S_FMT, &v4l2.CameraFormat), _v4l2_vidioc_s_error);
    v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormat), _v4l2_vidioc_g_error);
#ifdef DEBUG
    std::cout << "DeviceStatus: \n"
              << "  Width:  " << v4l2.CameraFormat.fmt.pix.width << "\n"
              << "  Height: " << v4l2.CameraFormat.fmt.pix.height << "\n"
              << "  PixFmt: " << v4l2.CameraFormat.fmt.pix.pixelformat << "\n";
#endif
    v4l2d.ImgWidth = v4l2.CameraFormat.fmt.pix.width;
    v4l2d.ImgHeight = v4l2.CameraFormat.fmt.pix.height;
    v4l2d.PixFormat = v4l2.CameraFormat.fmt.pix.pixelformat;

    // don't check parm, not support driver stil fine
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(_flag_CameraFD, VIDIOC_G_PARM, &parm);
    parm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = v4l2d.FrameRate;
    ioctl(_flag_CameraFD, VIDIOC_S_PARM, &parm);

    memset(&v4l2.CameraReqBuffer, 0, sizeof(v4l2.CameraReqBuffer));
    v4l2.CameraReqBuffer.count = v4l2d.FrameBuffer;
    v4l2.CameraReqBuffer.memory = V4L2_MEMORY_MMAP;
    v4l2.CameraReqBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_REQBUFS, &v4l2.CameraReqBuffer), _v4l2_reqbuff_error);
    v4l2Buffers = (void **)calloc(v4l2.CameraReqBuffer.count, sizeof(*v4l2Buffers));
    for (int Index = 0; Index < v4l2.CameraReqBuffer.count; ++Index)
    {
        memset(&v4l2.CameraQBuffer, 0, sizeof(v4l2.CameraQBuffer));
        v4l2.CameraQBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2.CameraQBuffer.memory = V4L2_MEMORY_MMAP;
        v4l2.CameraQBuffer.index = Index;
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QUERYBUF, &v4l2.CameraQBuffer), _v4l2_querybuff_error);
        v4l2Buffers[Index] = mmap(NULL, v4l2.CameraQBuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, _flag_CameraFD, v4l2.CameraQBuffer.m.offset);
        if (MAP_FAILED == v4l2Buffers[Index])
        {
#ifdef DEBUG
            std::cout << "  Mapping ERROR"
                      << "\n";
#endif
        }
    }

    for (int i = 0; i < v4l2.CameraReqBuffer.count; ++i)
    {
        memset(&v4l2.CameraBuffer, 0, sizeof(v4l2.CameraBuffer));
        v4l2.CameraBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2.CameraBuffer.memory = V4L2_MEMORY_MMAP;
        v4l2.CameraBuffer.index = i;
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraBuffer), _v4l2_qbuf_error);
    }

    memset(&v4l2.CameraBuffer, 0, sizeof(v4l2.CameraBuffer));
    v4l2.CameraBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.CameraBuffer.memory = V4L2_MEMORY_MMAP;
    v4l2.CameraBuffer.index = 0;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_STREAMON, &v4l2.CameraBuffer.type), _v4l2_vidioc_streamon_error);
}

bool V4L2Tools::V4L2Drive::V4L2Control(unsigned int id, int value)
{
    v4l2.CameraContorl.id = id;
    v4l2.CameraContorl.value = value;
    if (ioctl(_flag_CameraFD, VIDIOC_S_CTRL, &v4l2.CameraContorl) == -1)
    {
        return false;
    }
    return true;
}

void V4L2Tools::V4L2Drive::V4L2Read(V4L2Tools::V4l2Data &Vdata)
{
    unsigned int size = 0;
    if (Vdata.size <= 0)
        Vdata = V4L2Tools::V4l2Data(
            v4l2d.ImgWidth,
            v4l2d.ImgHeight,
            v4l2.CameraQBuffer.length,
            v4l2.CameraQBuffer.length,
            v4l2d.PixFormat,
            v4l2.CameraFormat.fmt.pix.bytesperline);

    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(_flag_CameraFD, &fds);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r = select(_flag_CameraFD + 1, &fds, NULL, NULL, &tv);
    if (ioctl(_flag_CameraFD, VIDIOC_DQBUF, &v4l2.CameraBuffer) != -1)
    {
        Vdata.bytesperline = v4l2.CameraFormat.fmt.pix.bytesperline;
        Vdata.size = v4l2.CameraBuffer.bytesused;
        std::copy((unsigned char *)v4l2Buffers[v4l2.CameraBuffer.index],
                  (unsigned char *)v4l2Buffers[v4l2.CameraBuffer.index] + Vdata.size,
                  Vdata.data.get());
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraBuffer), _v4l2_camread_error);
    }
    else
    {
    }
}

V4L2Tools::V4L2Drive::~V4L2Drive()
{
    memset(&v4l2.CameraBuffer, 0, sizeof(v4l2.CameraBuffer));
    v4l2.CameraBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(_flag_CameraFD, VIDIOC_STREAMOFF, &v4l2.CameraBuffer.type);
    for (int i = 0; i < v4l2d.FrameBuffer; i++)
    {
        munmap(v4l2Buffers[i], v4l2.CameraQBuffer.length);
    }
    free(v4l2Buffers);
    close(_flag_CameraFD);
}

void V4L2Tools::V4L2Drive::V4L2Log(int signal, int error)
{
    if (signal == -1)
    {
#ifdef DEBUG
        std::cout << "\033[31m[V4L2Expection]V4L2 Error with: \033[0m" << error << "\n";
        std::cout << "\033[31m[V4L2Expection]Error Codes    : \033[0m" << errno << "\n";
#endif
        throw error;
    }
}