#pragma once
#include <string.h>

#include "../_Excutable/crc32.h"
#include "../_WIFIBroadcast/WIFICastDriver.hpp"

#include "../RPiSingleAPM/src/_thirdparty/FlowController.hpp"

#ifdef MODULE_FFMPEG
#include "../_Thirdparty/FFMPEG/FFMPEGCodec.hpp"
#endif

#include "../_Excutable/CameraDrive/Drive_V4L2Reader.hpp"

#ifdef MODULE_FECLIB
#endif

using namespace WIFIBroadCast;
using SYSU = RuAPSSys::UORBMessage;
using SYSC = RuAPSSys::ConfigCLA;

#ifdef MODULE_FFMPEG
inline static const std::map<std::string, int> CodecFormats =
    {
        {"BGR3", AV_PIX_FMT_BGR24},
        {"YUYV", AV_PIX_FMT_YUYV422},
        {"YUV420", AV_PIX_FMT_YUV420P},
};
#endif

uint64_t GetTimeStamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

class COMController_t
{
public:
    COMController_t();
    ~COMController_t();

private:
    void VideoDataInject(uint8_t *data, int size);
    void COMBoradCastDataInject();
    //
    uint16_t FrameFECSyncID = 0;
    //
    bool IsTimedetectUpdated = false;
    int Timedetectedstart = 0;
    int Timedetectedstop = 0;
    int Timedetected = 0;
    int BroadCastDataCount = 0;
    //
    std::unique_ptr<WIFICastDriver> Injector;
    std::unique_ptr<FlowThread> NormalThread;
    std::unique_ptr<FlowThread> BroadcastThread;
    std::unique_ptr<FlowThread> RecvcastThread;
#ifdef MODULE_FFMPEG
    std::queue<FFMPEGTools::AVData> EncoderQueue;
    std::unique_ptr<FFMPEGTools::FFMPEGCodec> Encoder;
#endif
    //
    std::unique_ptr<V4L2Tools::V4L2Encoder> V4L2Enc;
};

COMController_t::COMController_t()
{
    // Step 1:
    if (SYSC::CommonConfig.COM_BroadCastEnable)
    {
        if (SYSU::StreamStatus.VideoIFlowRaw.size() > 0)
        {
            if (!(std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceIFormat == "H264" ||
                  std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceIFormat == "H265"))
            {
#ifdef MODULE_FFMPEG
                AVPixelFormat targetOption = (AVPixelFormat)CodecFormats.at((std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceIFormat));
                Encoder.reset(new FFMPEGTools::FFMPEGCodec({
                    .IOWidth = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceWidth,
                    .IOHeight = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceHeight,
                    .OBuffer = SYSC::CommonConfig.COM_BroadCastPFrameSize,
                    .OFrameRate = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceFPS,
                    .OBitRate = SYSC::CommonConfig.COM_BroadCastBitRate,
                    .CodecProfile = "baseline",
                    .OutputFormat = AV_CODEC_ID_H264,
                    .TargetFormat = targetOption,
                }));
#endif

                V4L2Enc.reset(new V4L2Tools::V4L2Encoder(
                    "/dev/video11",
                    {
                        .ImgWidth = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceWidth,
                        .ImgHeight = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceHeight,
                        .FrameRate = SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceFPS,
                        .FrameBuffer = MAXV4LBUF,
                        .Is_AutoSize = (SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceWidth < 0),
                        .PixFormat = V4L2Format_s.at(SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceIFormat),
                        .H264_PSize = SYSC::CommonConfig.COM_BroadCastPFrameSize,
                        .H264_Profile = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
                        .H264_Bitrate = SYSC::CommonConfig.COM_BroadCastBitRate,
                        .H264_EnablePPS = true,
                    }));
            }
            else
            {
            }

            // TODO: better way network control
            system("iw dev wlan1 set type monitor");
            system("iw dev wlan1 set monitor fcsfail");
            system("iw dev wlan1 set freq 5600");
            system("iw dev wlan1 set txpower fixed 3000");

            Injector.reset(new WIFICastDriver(SYSC::CommonConfig.BroadcastInterfaces));

            // Injector->WIFIRecvSinff();

            BroadcastThread.reset(new FlowThread(
                [&]()
                {
                    // Step 0. Target Video data
                    V4L2Tools::V4l2Data data;
                    V4L2Tools::V4l2Data dataOut;
                    size_t InjectVSize = 0;
                    std::shared_ptr<uint8_t> InjectVTarget;
                    // Step 1. Read From uorb
                    if (std::get<FrameBuffer<V4L2Tools::V4l2Data>>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).frameCount > 0)
                        data = std::get<FrameBuffer<V4L2Tools::V4l2Data>>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).peekFrame();
                    // Step 2. Transcodec or not, deal with VID data
                    if (data.size > 0)
                    {
                        // TODO: consider add a timestamp binding EFC and data
                        if (std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceIFormat == "H264" ||
                            std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceIFormat == "H265")
                        {
                            VideoDataInject(data.data, data.size);
                        }
                        else
                        {
#ifdef MODULE_FFMPEG
                            Encoder->pushFrame(data.data, data.size, data.bytesperline);
                            Encoder->getFrame(EncoderQueue);
                            //
                            for (; !EncoderQueue.empty(); EncoderQueue.pop())
                            {
                                VideoDataInject(EncoderQueue.front().data, EncoderQueue.front().size);
                            }
#else
                            // TODO: V4L2ENC support
                            V4L2Enc->V4L2EncodeSet(data, dataOut);

                            FrameFECSyncID++;
                            FrameFECSyncID = FrameFECSyncID == 0xff ? 0 : FrameFECSyncID;

                            InjectVSize = dataOut.size + 1 + 4;
                            InjectVTarget.reset(new uint8_t[InjectVSize]);
                            uint8_t *dats = new uint8_t[InjectVSize * 10];

                            InjectVTarget.get()[0] = FrameFECSyncID;
                            std::copy(dataOut.data, dataOut.data + dataOut.size, InjectVTarget.get() + 1);
                            // TODO: add CRC check
                            uint32_t table[256];
                            crc32::generate_table(table);
                            uint32_t crc = crc32::update(table, 0, (const void *)InjectVTarget.get(), InjectVSize - 4);
                            // std::cout<< std::hex << crc << "\n";
                            InjectVTarget.get()[InjectVSize - 4] = (uint8_t)(crc);
                            InjectVTarget.get()[InjectVSize - 3] = (uint8_t)(crc >> 8);
                            InjectVTarget.get()[InjectVSize - 2] = (uint8_t)(crc >> 16);
                            InjectVTarget.get()[InjectVSize - 1] = (uint8_t)(crc >> 24);
                            // TODO: add EFC data frame on COM_CastFrameIndex + 1
                            Injector->WIFICastInject(InjectVTarget.get(), InjectVSize, 0, BroadCastType::VideoStream, 0, SYSC::CommonConfig.COM_CastFrameIndex * 2);
#endif
                        }

                        // Step N + 1. Inject img info.
                        BroadCastDataCount++;
                        if (BroadCastDataCount >= (float)SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceFPS)
                        {
                            BroadCastDataCount = 0;
                            uint8_t ImgInfo[] = {
                                (uint8_t)(SYSC::CommonConfig.COM_CastFrameIndex),
                                (uint8_t)(data.maxsize),
                                (uint8_t)(data.maxsize >> 8),
                                (uint8_t)(data.maxsize >> 16),
                                (uint8_t)(data.maxsize >> 24),
                                (uint8_t)(std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceWidth),
                                (uint8_t)(std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceWidth >> 8),
                                (uint8_t)(std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceHeight),
                                (uint8_t)(std::get<SYSC::VideoSettings>(SYSU::StreamStatus.VideoIFlowRaw[SYSC::CommonConfig.COM_CastFrameIndex]).DeviceHeight >> 8),
                            };

                            Injector->WIFICastInject(ImgInfo, sizeof(ImgInfo), 0, BroadCastType::DataStream, 0, 0xf);
                            if (IsTimedetectUpdated)
                            {
                                Timedetectedstart = GetTimeStamp();
                                IsTimedetectUpdated = false;
                            };
                        }
                    }
                    COMBoradCastDataInject();
                },
                (float)SYSC::VideoConfig[SYSC::CommonConfig.COM_CastFrameIndex].DeviceFPS));

            RecvcastThread.reset(new FlowThread(
                [&]
                {
                    if (Injector->DataEBuffer.size() > 0)
                    {
                        std::string DataInput = Injector->DataEBuffer.front();
                        if (DataInput.c_str()[0] == FeedBackTrans)
                        {
                            Timedetectedstop = GetTimeStamp();
                            Timedetected = Timedetectedstop - Timedetectedstart;
                            IsTimedetectUpdated = true;
                        }
                        Injector->DataEBuffer.pop();
                    }
                },
                500.f));
        }
    }
}

void COMController_t::VideoDataInject(uint8_t *data, int size)
{
    int InjectVSize;
    std::shared_ptr<uint8_t> InjectVTarget;

    FrameFECSyncID++;
    FrameFECSyncID = FrameFECSyncID == 0xff ? 0 : FrameFECSyncID;

    InjectVSize = size + 1 + 4;
    InjectVTarget.reset(new uint8_t[InjectVSize]);

    InjectVTarget.get()[0] = FrameFECSyncID;
    std::copy(data, data + size, InjectVTarget.get() + 1);
    // TODO: add CRC check
    uint32_t table[256];
    crc32::generate_table(table);
    uint32_t crc = crc32::update(table, 0, (const void *)InjectVTarget.get(), InjectVSize - 4);
    // std::cout<< std::hex << crc << "\n";
    InjectVTarget.get()[InjectVSize - 4] = (uint8_t)(crc);
    InjectVTarget.get()[InjectVSize - 3] = (uint8_t)(crc >> 8);
    InjectVTarget.get()[InjectVSize - 2] = (uint8_t)(crc >> 16);
    InjectVTarget.get()[InjectVSize - 1] = (uint8_t)(crc >> 24);
    // TODO: add EFC data frame on COM_CastFrameIndex + 1
    Injector->WIFICastInject(InjectVTarget.get(), InjectVSize, 0, BroadCastType::VideoStream, 0, SYSC::CommonConfig.COM_CastFrameIndex * 2);
}

void COMController_t::COMBoradCastDataInject()
{
}

COMController_t::~COMController_t()
{
    if (NormalThread != nullptr)
        NormalThread->FlowStopAndWait();
    if (BroadcastThread != nullptr)
    {
        BroadcastThread->FlowStopAndWait();
#ifdef MODULE_FFMPEG
        if (Encoder != nullptr)
        {
            Encoder.reset();
        }
#endif
    }
    if (RecvcastThread != nullptr)
        RecvcastThread->FlowStopAndWait();
    if (Injector != nullptr)
        Injector.reset();
}