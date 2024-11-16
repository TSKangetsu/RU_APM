#include "Drive_V4L2Reader.hpp"

V4L2Tools::V4L2Encoder::V4L2Encoder(std::string Device, V4l2Info Info, bool isgeter)
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
    if (v4l2d.PixFormatOut == V4L2_PIX_FMT_H264)
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

    isMPlaneSupported = true;
    if (!isgeter)
    {
        memset(&v4l2.CameraFormat, 0, sizeof(v4l2.CameraFormat));
        memset(&v4l2.CameraFormatOut, 0, sizeof(v4l2.CameraFormatOut));
        v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2.CameraFormatOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormat) < 0 ||
            ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormatOut) < 0)
        {
            v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            v4l2.CameraFormatOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormat) == 0 &&
                ioctl(_flag_CameraFD, VIDIOC_G_FMT, &v4l2.CameraFormatOut) == 0)
            {
                isMPlaneSupported = false;
            }
            else
                throw std::invalid_argument("not a 2in-out m2m device");
        }

        // try MPLANE
        if (isMPlaneSupported)
        {
            v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            v4l2.CameraFormat.fmt.pix_mp.width = v4l2d.ImgWidth;
            v4l2.CameraFormat.fmt.pix_mp.height = v4l2d.ImgHeight;
            v4l2.CameraFormat.fmt.pix_mp.pixelformat = v4l2d.PixFormat;
            v4l2.CameraFormat.fmt.pix_mp.field = V4L2_FIELD_ANY;
            v4l2.CameraFormat.fmt.pix_mp.num_planes = 1;

            v4l2.CameraFormatOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            v4l2.CameraFormatOut.fmt.pix_mp.width = v4l2d.ImgWidth;
            v4l2.CameraFormatOut.fmt.pix_mp.height = v4l2d.ImgHeight;
            v4l2.CameraFormatOut.fmt.pix_mp.pixelformat = v4l2d.PixFormatOut;
            v4l2.CameraFormatOut.fmt.pix_mp.field = V4L2_FIELD_ANY;
            v4l2.CameraFormatOut.fmt.pix_mp.num_planes = 1;
        }
        else
        {
            v4l2.CameraFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            v4l2.CameraFormat.fmt.pix.width = v4l2d.ImgWidth;
            v4l2.CameraFormat.fmt.pix.height = v4l2d.ImgHeight;
            v4l2.CameraFormat.fmt.pix.pixelformat = v4l2d.PixFormat;
            v4l2.CameraFormat.fmt.pix.field = V4L2_FIELD_ANY;

            v4l2.CameraFormatOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            v4l2.CameraFormatOut.fmt.pix.width = v4l2d.ImgWidth;
            v4l2.CameraFormatOut.fmt.pix.height = v4l2d.ImgHeight;
            v4l2.CameraFormatOut.fmt.pix.pixelformat = v4l2d.PixFormatOut;
            v4l2.CameraFormatOut.fmt.pix.field = V4L2_FIELD_ANY;
        }

        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_S_FMT, &v4l2.CameraFormat), _v4l2_vidioc_s_error);
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_S_FMT, &v4l2.CameraFormatOut), 10 + _v4l2_vidioc_s_error);
    }

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = isMPlaneSupported ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
    parm.parm.output.timeperframe.numerator = 1;
    parm.parm.output.timeperframe.denominator = v4l2d.FrameRate;
    ioctl(_flag_CameraFD, VIDIOC_S_PARM, &parm); // if not work just skip

    //=========================================================================================//
    memset(&v4l2.CameraReqBuffer, 0, sizeof(v4l2.CameraReqBuffer));
    v4l2.CameraReqBuffer.count = v4l2d.FrameBuffer;
    v4l2.CameraReqBuffer.memory = V4L2_MEMORY_MMAP;
    v4l2.CameraReqBuffer.type = isMPlaneSupported ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_REQBUFS, &v4l2.CameraReqBuffer), _v4l2_reqbuff_error);
    v4l2Buffers = (void **)calloc(v4l2.CameraReqBuffer.count, sizeof(*v4l2Buffers));

    memset(&v4l2.CameraReqBufferOut, 0, sizeof(v4l2.CameraReqBufferOut));
    v4l2.CameraReqBufferOut.count = v4l2d.FrameBuffer;
    v4l2.CameraReqBufferOut.memory = V4L2_MEMORY_MMAP;
    v4l2.CameraReqBufferOut.type = isMPlaneSupported ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L2Log(ioctl(_flag_CameraFD, VIDIOC_REQBUFS, &v4l2.CameraReqBufferOut), 10 + _v4l2_reqbuff_error);
    v4l2BuffersOut = (void **)calloc(v4l2.CameraReqBufferOut.count, sizeof(*v4l2BuffersOut));

    if (isMPlaneSupported)
    {
        for (int Index = 0; Index < v4l2.CameraReqBuffer.count; ++Index)
        {
            v4l2_plane plane;
            memset(&plane, 0, sizeof(plane));
            memset(&v4l2.CameraQBuffer, 0, sizeof(v4l2.CameraQBuffer));
            v4l2.CameraQBuffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            v4l2.CameraQBuffer.memory = V4L2_MEMORY_MMAP;
            v4l2.CameraQBuffer.index = Index;
            v4l2.CameraQBuffer.length = 1;
            v4l2.CameraQBuffer.m.planes = &plane;
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QUERYBUF, &v4l2.CameraQBuffer), _v4l2_querybuff_error);
            v4l2Buffers[Index] = mmap(NULL, plane.length,
                                      PROT_READ | PROT_WRITE, MAP_SHARED, _flag_CameraFD, plane.m.mem_offset);
            if (MAP_FAILED == v4l2Buffers[Index])
            {
#ifdef DEBUG
                std::cout << "  Mapping ERROR"
                          << "\n";
#endif
            }
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraQBuffer), _v4l2_qbuf_error);
        }

        for (int Index = 0; Index < v4l2.CameraReqBufferOut.count; ++Index)
        {
            v4l2_plane plane;
            memset(&plane, 0, sizeof(plane));
            memset(&v4l2.CameraQBufferOut, 0, sizeof(v4l2.CameraQBufferOut));
            v4l2.CameraQBufferOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            v4l2.CameraQBufferOut.memory = V4L2_MEMORY_MMAP;
            v4l2.CameraQBufferOut.index = Index;
            v4l2.CameraQBufferOut.length = 1;
            v4l2.CameraQBufferOut.m.planes = &plane;
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QUERYBUF, &v4l2.CameraQBufferOut), 10 + _v4l2_querybuff_error);
            v4l2BuffersOut[Index] = mmap(NULL, plane.length,
                                         PROT_READ | PROT_WRITE, MAP_SHARED, _flag_CameraFD, plane.m.mem_offset);
            if (MAP_FAILED == v4l2BuffersOut[Index])
            {
#ifdef DEBUG
                std::cout << "  Mapping ERROR"
                          << "\n";
#endif
            }
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraQBufferOut), 10 + _v4l2_qbuf_error);
        }
    }
    else
    {
        for (int Index = 0; Index < v4l2.CameraReqBuffer.count; ++Index)
        {
            memset(&v4l2.CameraQBuffer, 0, sizeof(v4l2.CameraQBuffer));
            v4l2.CameraQBuffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
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
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraQBuffer), _v4l2_qbuf_error);
        }

        for (int Index = 0; Index < v4l2.CameraReqBufferOut.count; ++Index)
        {
            memset(&v4l2.CameraQBufferOut, 0, sizeof(v4l2.CameraQBufferOut));
            v4l2.CameraQBufferOut.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            v4l2.CameraQBufferOut.memory = V4L2_MEMORY_MMAP;
            v4l2.CameraQBufferOut.index = Index;
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QUERYBUF, &v4l2.CameraQBufferOut), _v4l2_querybuff_error);
            v4l2BuffersOut[Index] = mmap(NULL, v4l2.CameraQBufferOut.length, PROT_READ | PROT_WRITE, MAP_SHARED, _flag_CameraFD, v4l2.CameraQBufferOut.m.offset);
            if (MAP_FAILED == v4l2BuffersOut[Index])
            {
#ifdef DEBUG
                std::cout << "  Mapping ERROR"
                          << "\n";
#endif
            }
            V4L2Log(ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraQBufferOut), 10 + _v4l2_qbuf_error);
        }
    }
    //=========================================================================================//

    {
        v4l2_plane plane;
        memset(&plane, 0, sizeof(plane));
        memset(&v4l2.CameraBuffer, 0, sizeof(v4l2.CameraBuffer));
        v4l2.CameraBuffer.type = isMPlaneSupported ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
        v4l2.CameraBuffer.memory = V4L2_MEMORY_MMAP;
        v4l2.CameraBuffer.field = V4L2_FIELD_NONE;
        v4l2.CameraBuffer.index = 0;
        v4l2.CameraBuffer.length = isMPlaneSupported ? 1 : v4l2.CameraQBuffer.length;
        v4l2.CameraBuffer.m.planes = &plane;
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_STREAMON, &v4l2.CameraBuffer.type), _v4l2_vidioc_streamon_error);
    }

    {
        v4l2_plane plane;
        memset(&plane, 0, sizeof(plane));
        memset(&v4l2.CameraBufferOut, 0, sizeof(v4l2.CameraBufferOut));
        v4l2.CameraBufferOut.type = isMPlaneSupported ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2.CameraBufferOut.memory = V4L2_MEMORY_MMAP;
        v4l2.CameraBufferOut.field = V4L2_FIELD_NONE;
        v4l2.CameraBufferOut.index = 0;
        v4l2.CameraBufferOut.length = isMPlaneSupported ? 1 : v4l2.CameraQBufferOut.length;
        v4l2.CameraBufferOut.m.planes = &plane;
        V4L2Log(ioctl(_flag_CameraFD, VIDIOC_STREAMON, &v4l2.CameraBufferOut.type), _v4l2_vidioc_streamon_error);
    }
}

void V4L2Tools::V4L2Encoder::V4L2EncodeSet(V4L2Tools::V4l2Data &VdataIn, V4L2Tools::V4l2Data &VdataOut)
{
    if (VdataIn.size <= 0)
        VdataIn = V4L2Tools::V4l2Data(
            v4l2d.ImgWidth,
            v4l2d.ImgHeight,
            isMPlaneSupported ? v4l2.CameraQBuffer.m.planes->length : v4l2.CameraQBuffer.length,
            isMPlaneSupported ? v4l2.CameraQBuffer.m.planes->length : v4l2.CameraQBuffer.length,
            v4l2d.PixFormat,
            v4l2.CameraFormat.fmt.pix.bytesperline);

    ioctl(_flag_CameraFD, VIDIOC_DQBUF, &v4l2.CameraBuffer);
    VdataIn.data = (unsigned char *)v4l2Buffers[v4l2.CameraBuffer.index];

    if (isMPlaneSupported)
        v4l2.CameraBuffer.m.planes->length = VdataIn.maxsize;
    else
    {
        v4l2.CameraBuffer.length = VdataIn.maxsize;
        v4l2.CameraBuffer.bytesused = VdataIn.size;
    }
    //
    ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraBuffer);
    //=========================================================================================//
    if (VdataOut.size <= 0)
        VdataOut = V4L2Tools::V4l2Data(
            v4l2d.ImgWidth,
            v4l2d.ImgHeight,
            isMPlaneSupported ? v4l2.CameraQBufferOut.m.planes->length : v4l2.CameraQBufferOut.length,
            isMPlaneSupported ? v4l2.CameraQBufferOut.m.planes->length : v4l2.CameraQBufferOut.length,
            v4l2d.PixFormatOut,
            v4l2.CameraFormatOut.fmt.pix.bytesperline);

    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(_flag_CameraFD, &fds);
    tv.tv_sec = 1; // TODO: fix loop logic
    tv.tv_usec = 0;
    r = select(_flag_CameraFD + 1, &fds, NULL, NULL, &tv);

    ioctl(_flag_CameraFD, VIDIOC_DQBUF, &v4l2.CameraBufferOut);

    VdataOut.bytesperline = v4l2.CameraFormatOut.fmt.pix.bytesperline;
    if (isMPlaneSupported)
        VdataOut.size = v4l2.CameraBufferOut.m.planes->bytesused;
    else
        VdataOut.size = v4l2.CameraBufferOut.bytesused;
    VdataOut.data = (unsigned char *)v4l2BuffersOut[v4l2.CameraBufferOut.index];
    ioctl(_flag_CameraFD, VIDIOC_QBUF, &v4l2.CameraBufferOut);
}