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
#include "../_Excutable/FEC/fec.hpp"
#define FEC_PACKET_MAX 64
#define FEC_DATA_MAX (FEC_PACKET_MAX * PacketPrePacks)
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
    V4L2Tools::V4l2Data comInVdata;
    V4L2Tools::V4l2Data comInVdataOut;
    std::shared_ptr<uint8_t> InjectVTarget;
    std::shared_ptr<uint8_t> InjectFTarget;

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
#ifdef MODULE_FECLIB
    FecPacket<FEC_DATA_MAX, FEC_PACKET_MAX, PacketPrePacks> fecPool;
    FecPacket<FEC_DATA_MAX, FEC_PACKET_MAX, PacketPrePacks> dataPool;
#endif
    //
    std::unique_ptr<V4L2Tools::V4L2Encoder> V4L2Enc;
};

COMController_t::COMController_t()
{
#ifdef MODULE_FECLIB
    fec_init();
#endif
    InjectVTarget.reset(new uint8_t[1920 * 1080 * 3]);
    InjectFTarget.reset(new uint8_t[1920 * 1080 * 3]);

    // Step 1:
    if (SYSC::CommonConfig.COM_BroadCastEnable)
    {
        if (SYSU::StreamStatus.VideoIFlowRaw.size() > 0)
        {
            if (!(std::get<SYSC::CameraSettings>(
                      SYSU::StreamStatus.VideoIFlowRaw
                          [SYSC::CommonConfig.COM_CastFrameIndex])
                          .DeviceIFormat == "H264" ||
                  std::get<SYSC::CameraSettings>(
                      SYSU::StreamStatus.VideoIFlowRaw
                          [SYSC::CommonConfig.COM_CastFrameIndex])
                          .DeviceIFormat == "H265"))
            {
#ifdef MODULE_FFMPEG
                AVPixelFormat targetOption = (AVPixelFormat)CodecFormats.at(
                    (std::get<SYSC::CameraSettings>(
                         SYSU::StreamStatus.VideoIFlowRaw
                             [SYSC::CommonConfig.COM_CastFrameIndex])
                         .DeviceIFormat));

                Encoder.reset(new FFMPEGTools::FFMPEGCodec({
                    .IOWidth = SYSC::CameraConfig[SYSC::CommonConfig
                                                      .COM_CastFrameIndex]
                                   .DeviceWidth,
                    .IOHeight = SYSC::CameraConfig[SYSC::CommonConfig
                                                       .COM_CastFrameIndex]
                                    .DeviceHeight,
                    .OBuffer = SYSC::CommonConfig.COM_BroadCastPFrameSize,
                    .OFrameRate = SYSC::CameraConfig[SYSC::CommonConfig
                                                         .COM_CastFrameIndex]
                                      .DeviceFPS,
                    .OBitRate = SYSC::CommonConfig.COM_BroadCastBitRate,
                    .CodecProfile = "baseline",
                    .OutputFormat = AV_CODEC_ID_H264,
                    .TargetFormat = targetOption,
                }));
#endif

                V4L2Enc.reset(new V4L2Tools::V4L2Encoder(
                    SYSC::VideoConfig.V4L2Encoder,
                    {
                        .ImgWidth = SYSC::CameraConfig
                                        [SYSC::CommonConfig.COM_CastFrameIndex]
                                            .DeviceWidth,
                        .ImgHeight = SYSC::CameraConfig
                                         [SYSC::CommonConfig.COM_CastFrameIndex]
                                             .DeviceHeight,
                        .FrameRate = SYSC::CameraConfig
                                         [SYSC::CommonConfig.COM_CastFrameIndex]
                                             .DeviceFPS,
                        .FrameBuffer = MAXV4LBUF,
                        .Is_AutoSize = (SYSC::CameraConfig
                                            [SYSC::CommonConfig.COM_CastFrameIndex]
                                                .DeviceWidth < 0),
                        .PixFormat = V4L2Format_s.at(SYSC::CameraConfig
                                                         [SYSC::CommonConfig
                                                              .COM_CastFrameIndex]
                                                             .DeviceIFormat),
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
            char cmd[64];
            sprintf(cmd, "ifconfig %s up",
                    SYSC::CommonConfig.BroadcastInterfaces[0].c_str());
            system(cmd);
            sprintf(cmd, "iw dev %s set type monitor",
                    SYSC::CommonConfig.BroadcastInterfaces[0].c_str());
            system(cmd);
            sprintf(cmd, "iw dev %s set monitor fcsfail otherbss",
                    SYSC::CommonConfig.BroadcastInterfaces[0].c_str());
            system(cmd);
            sprintf(cmd, "iw dev %s set freq 5600 NOHT",
                    SYSC::CommonConfig.BroadcastInterfaces[0].c_str());
            system(cmd);
            sprintf(cmd, "iw dev %s set txpower fixed 3000",
                    SYSC::CommonConfig.BroadcastInterfaces[0].c_str());
            system(cmd);

            Injector.reset(new WIFICastDriver(
                SYSC::CommonConfig.BroadcastInterfaces));

            // Injector->WIFIRecvSinff();

            BroadcastThread.reset(new FlowThread(
                [&]()
                {
                    // Step 0. Target Video data
                    size_t InjectVSize = 0;
                    // Step 1. Read From uorb
                    while (std::get<FrameBuffer<V4L2Tools::V4l2Data>>(
                               SYSU::StreamStatus.VideoIFlowRaw
                                   [SYSC::CommonConfig.COM_CastFrameIndex])
                               .frameCount >= MAXBUFFER)
                        comInVdata = std::get<FrameBuffer<V4L2Tools::V4l2Data>>(
                                         SYSU::StreamStatus.VideoIFlowRaw
                                             [SYSC::CommonConfig.COM_CastFrameIndex])
                                         .getFrame();

                    // std::cout << std::get<FrameBuffer<V4L2Tools::V4l2Data>>(
                    //                  SYSU::StreamStatus.VideoIFlowRaw
                    //                      [SYSC::CommonConfig.COM_CastFrameIndex])
                    //                  .frameCount
                    //           << " " << comInVdata.id << '\n';
                    // Step 2. Transcodec or not, deal with VID data
                    if (comInVdata.size > 0)
                    {
                        // TODO: consider add a timestamp binding EFC and data
                        if (std::get<SYSC::CameraSettings>(
                                SYSU::StreamStatus.VideoIFlowRaw
                                    [SYSC::CommonConfig.COM_CastFrameIndex])
                                    .DeviceIFormat == "H264" ||
                            std::get<SYSC::CameraSettings>(
                                SYSU::StreamStatus.VideoIFlowRaw
                                    [SYSC::CommonConfig.COM_CastFrameIndex])
                                    .DeviceIFormat == "H265")
                        {
                            VideoDataInject(comInVdata.data, comInVdata.size);
                        }
                        else
                        {
#ifdef MODULE_FFMPEG
                            Encoder->pushFrame(comInVdata.data.get(), comInVdata.size, comInVdata.bytesperline);
                            Encoder->getFrame(EncoderQueue);
                            //
                            for (; !EncoderQueue.empty(); EncoderQueue.pop())
                            {
                                VideoDataInject(EncoderQueue.front().comInVdata.get(),
                                                EncoderQueue.front().size);
                            }
#else
                            // TODO: V4L2ENC support
                            comInVdata.ismapping = false;
                            comInVdataOut = V4L2Enc->V4l2DataGetOut();
                            V4L2Enc->V4L2EncodeSet(comInVdata, comInVdataOut);
                            if (comInVdataOut.size != 0)
                                VideoDataInject(comInVdataOut.data, comInVdataOut.size);
#endif
                        }

                        // Step N + 1. Inject img info.
                        BroadCastDataCount++;
                        if (BroadCastDataCount >=
                            (float)SYSC::CameraConfig
                                [SYSC::CommonConfig.COM_CastFrameIndex]
                                    .DeviceFPS)
                        {
                            BroadCastDataCount = 0;
                            uint8_t ImgInfo[] = {
                                (uint8_t)(SYSC::CommonConfig.COM_CastFrameIndex),
                                (uint8_t)(comInVdata.maxsize),
                                (uint8_t)(comInVdata.maxsize >> 8),
                                (uint8_t)(comInVdata.maxsize >> 16),
                                (uint8_t)(comInVdata.maxsize >> 24),
                                (uint8_t)(std::get<SYSC::CameraSettings>(
                                              SYSU::StreamStatus.VideoIFlowRaw
                                                  [SYSC::CommonConfig.COM_CastFrameIndex])
                                              .DeviceWidth),
                                (uint8_t)(std::get<SYSC::CameraSettings>(
                                              SYSU::StreamStatus.VideoIFlowRaw
                                                  [SYSC::CommonConfig.COM_CastFrameIndex])
                                              .DeviceWidth >>
                                          8),
                                (uint8_t)(std::get<SYSC::CameraSettings>(
                                              SYSU::StreamStatus.VideoIFlowRaw
                                                  [SYSC::CommonConfig.COM_CastFrameIndex])
                                              .DeviceHeight),
                                (uint8_t)(std::get<SYSC::CameraSettings>(
                                              SYSU::StreamStatus.VideoIFlowRaw
                                                  [SYSC::CommonConfig.COM_CastFrameIndex])
                                              .DeviceHeight >>
                                          8),
                            };

                            Injector->WIFICastInject(ImgInfo,
                                                     sizeof(ImgInfo), 0,
                                                     BroadCastType::DataStream,
                                                     0, 0xf, 0xff);
#ifdef MODULE_FECLIB
                            // FEC data using next channel
                            ImgInfo[0] = (uint8_t)(SYSC::CommonConfig.COM_CastFrameIndex + 1);
                            Injector->WIFICastInject(ImgInfo,
                                                     sizeof(ImgInfo), 0,
                                                     BroadCastType::DataStream,
                                                     0, 0xf, 0xff);
#endif
                            if (IsTimedetectUpdated)
                            {
                                Timedetectedstart = GetTimeStamp();
                                IsTimedetectUpdated = false;
                            };
                        }
                    }
                    COMBoradCastDataInject();
                },
                (float)SYSC::CameraConfig
                    [SYSC::CommonConfig.COM_CastFrameIndex]
                        .DeviceFPS));

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
    int InjectFSize;

    FrameFECSyncID++;
    FrameFECSyncID = FrameFECSyncID > 7 ? 0 : FrameFECSyncID;

    InjectVSize = size + 4;

    std::copy(data, data + size, InjectVTarget.get());
    // TODO: add CRC check
    {
        uint32_t table[256];
        crc32::generate_table(table);
        uint32_t crc = crc32::update(table,
                                     0,
                                     (const void *)InjectVTarget.get(),
                                     InjectVSize - 4);
        // std::cout << std::hex << crc << "\n";
        InjectVTarget.get()[InjectVSize - 4] = (uint8_t)(crc);
        InjectVTarget.get()[InjectVSize - 3] = (uint8_t)(crc >> 8);
        InjectVTarget.get()[InjectVSize - 2] = (uint8_t)(crc >> 16);
        InjectVTarget.get()[InjectVSize - 1] = (uint8_t)(crc >> 24);
    }
    // TODO: add EFC data frame on COM_CastFrameIndex + 1
    int packetSize = Injector->WIFICastInject(InjectVTarget.get(),
                                              InjectVSize, 0,
                                              BroadCastType::VideoStream,
                                              0, SYSC::CommonConfig.COM_CastFrameIndex * 2,
                                              FrameFECSyncID);

#ifdef MODULE_FECLIB
    InjectFSize = size + 4; // NO CRC32, why using CRC for FEC? Update: yes, just fec the crc data, after fix can use it
    if (InjectFSize < FEC_DATA_MAX)
    {
        //
        std::memset(fecPool.FecDataType_t.data1d, 0x00, FEC_DATA_MAX);
        std::memset(dataPool.FecDataType_t.data1d, 0x00, FEC_DATA_MAX);
        //
        std::copy(InjectVTarget.get(),
                  InjectVTarget.get() + InjectFSize,
                  dataPool.FecDataType_t.data1d);
        fec_encode(PacketPrePacks,
                   dataPool.dataout, packetSize, fecPool.dataout,
                   packetSize);

        Injector->WIFICastInject(fecPool.FecDataType_t.data1d,
                                 InjectFSize, // FIXME: FEC frame same as data frame? now test with full fec out
                                 0, BroadCastType::VideoStream, 0,
                                 SYSC::CommonConfig.COM_CastFrameIndex * 2 + 1,
                                 FrameFECSyncID);
    }
#endif
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