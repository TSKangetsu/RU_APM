#pragma once
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <sys/mman.h>
#include <errno.h>
#include <queue>
#include <mutex>
#include <memory>
#ifdef DEBUG
#include <iostream>
#endif

namespace V4L2Tools
{
    struct V4l2Info
    {
        int ImgWidth = 640;
        int ImgHeight = 380;
        int FrameRate = 30;
        int FrameBuffer = 4;
        bool Is_AutoSize = false;
        unsigned int PixFormat = V4L2_PIX_FMT_BGR24;
        unsigned int PixFormatOut = V4L2_PIX_FMT_H264;
        // H264 camera codec control
        int H264_PSize = 60;
        int H264_Profile = 0;
        int H264_Bitrate = 1500000;
        bool H264_EnablePPS = true;
    };

    struct V4l2Data
    {
        int width;
        int height;
        int maxsize;
        unsigned int size;
        unsigned int pixfmt;
        unsigned char *data;
        unsigned int bytesperline;
        //
        V4l2Data() : width(0), height(0), maxsize(0), size(0), pixfmt(0), data(nullptr), bytesperline(0) {};
        V4l2Data(int width,
                 int height,
                 int maxsize,
                 unsigned int size,
                 unsigned int pixfmt,
                 unsigned int bytesperline)
        {
            this->width = width;
            this->height = height;
            this->size = size;
            this->maxsize = maxsize;
            this->pixfmt = pixfmt;
            this->data = new unsigned char[this->size];
            this->bytesperline = bytesperline;
        };
        V4l2Data &operator=(const V4l2Data &DataCpy)
        {
            width = DataCpy.width;
            height = DataCpy.height;
            size = DataCpy.size;
            maxsize = DataCpy.maxsize;
            pixfmt = DataCpy.pixfmt;
            data = new unsigned char[size];
            std::copy(DataCpy.data, DataCpy.data + size, this->data);
            bytesperline = DataCpy.bytesperline;
            return *this;
        };
        V4l2Data(const V4l2Data &DataCpy)
        {
            width = DataCpy.width;
            height = DataCpy.height;
            size = DataCpy.size;
            maxsize = DataCpy.maxsize;
            pixfmt = DataCpy.pixfmt;
            data = new unsigned char[size];
            std::copy(DataCpy.data, DataCpy.data + size, this->data);
            bytesperline = DataCpy.bytesperline;
        };
        ~V4l2Data()
        {
            if (data != nullptr)
                delete[] data;
        };
    };

    enum V4L2Error
    {
        _v4l2_query_error,
        _v4l2_vidioc_s_error,
        _v4l2_vidioc_g_error,
        _v4l2_vipram_s_error,
        _v4l2_vipram_g_error,
        _v4l2_reqbuff_error,
        _v4l2_querybuff_error,
        _v4l2_qbuf_error,
        _v4l2_vidioc_streamon_error,
        _v4l2_camread_error,
        _v4l2_autoset_error,
        _v4l2_encin_error
    };

    class V4L2Drive
    {
    public:
        V4L2Drive(std::string Device, V4l2Info Info);
        void V4L2Read(V4L2Tools::V4l2Data &Vdata);

        int V4L2FDGetter() { return _flag_CameraFD; };
        bool V4L2Control(unsigned int id, int value);

        ~V4L2Drive();

    protected:
        V4L2Drive() {}; // empty for encoder

        void V4L2Log(int signal, int error);

        void **v4l2Buffers;
        void **v4l2BuffersOut;
        int _flag_Error;
        int _flag_CameraFD;
        std::string _flag_TargetDevice;
        V4l2Info v4l2d;

        struct V4l2Dep
        {
            v4l2_control CameraContorl;
            v4l2_capability CameraInfo;
            v4l2_format CameraFormat;
            v4l2_format CameraFormatOut;
            v4l2_requestbuffers CameraReqBuffer;
            v4l2_requestbuffers CameraReqBufferOut;
            v4l2_buf_type CameraType;
            v4l2_buffer CameraQBuffer;
            v4l2_buffer CameraQBufferOut;
            v4l2_buffer CameraBuffer;
            v4l2_buffer CameraBufferOut;
            v4l2_fmtdesc CameraFMTInfo;
        } v4l2;
    };

    class V4L2Encoder : private V4L2Drive
    {
    public:
        V4L2Encoder(std::string Device, V4l2Info Info);
        void V4L2EncodeSet(V4L2Tools::V4l2Data &VdataIn, V4L2Tools::V4l2Data &VdataOut);
    };
} // namespace V4L2Tools
