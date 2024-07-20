#pragma once
#include <map>
#include <tuple>
#include <queue>
#include <vector>
#include <string>
#include <fstream>
#include "../_Excutable/Drive_Json.hpp"
#include "../_Excutable/ThreadBuffer.hpp"
#ifdef MODULE_APM
#include "../RPiSingleAPM/json/APMJsonConfig.hpp"
#include "../RPiSingleAPM/src/SingleAPM.hpp"
#endif
#include "../_Excutable/CameraDrive/Drive_V4L2Reader.hpp"

using json = nlohmann::json;
#define EMAP(Variable) (#Variable)

namespace RuAPSSys
{
	class ConfigCLA
	{
	public:
#ifdef MODULE_APM
		inline static struct SingleAPMAPI::APMSettinngs APMConfig;
#endif

		struct VideoSettings
		{
			std::string DevicePATH;
			std::string DeviceDriver;
			std::string DeviceIFormat;

			int DeviceWidth;
			int DeviceHeight;
			int DeviceFPS;
			bool enable;
		};
		inline static std::vector<VideoSettings> VideoConfig;

		inline static struct CommonSettings
		{
			int COM_CastFrameIndex;
			bool COM_BroadCastEnable;
			bool COM_NormalCastEnable;

			int COM_BroadCastBitRate;
			int COM_BroadCastPFrameSize;
			std::vector<std::string> BroadcastInterfaces;
		} CommonConfig;

		inline static struct PluginSettings
		{

		} PluginConfig;
	};

	class UORBMessage
	{
	public:
		inline static struct ControllerStatus_t
		{
			int *_Accel_ClippedTimes;
			float *_Accel_Accelration[3];
			float *_Accel_VIBE[3];
			float *_Accel_RawG[3];
			float *_Accel_GForce;

			float *_ATT_Quaterion[4];
			float *_ATT_EulerAngle[3];
			float *_Gyro_AngleRate[3];

			float *_Baro_Temp;
			float *_Baro_PressureHPA;
			float *_Baro_AGLAltitudeCM;
			float *_RangeFinder_AGLAltCM;

			int *_SYS_TimeInfo[10];
			uint16_t *_SYS_PreARMFlag;
			uint16_t *_SYS_FailSafeFlag;
			int *_SYS_APMStatus;

			double *_NAV_Speed[3];
			double *_NAV_Global_Speed[2];
			float *_NAV_Relative_Head;
			int *_NAV_Global_Pos[3];
			int *_NAV_Global_HOME[2];
			double *_NAV_Relative_Pos[3];
			int *_NAV_Global_SATCount;
			float *_NAV_Global_Head;
		} ControllerStatus;

		inline static struct StreamStatus_t
		{
			std::vector<std::tuple<FrameBuffer<V4L2Tools::V4l2Data>, ConfigCLA::VideoSettings>> VideoIFlowRaw;
		} StreamStatus;

		inline static struct SystemStatus_t
		{
			std::queue<std::string> SystemMessage;
		} SystemStatus;

		inline static struct PluginStatus_t
		{

		} PluginStatus;
	};
};

namespace RuAPSSys
{
	void to_json(json &j, const ConfigCLA::VideoSettings &p)
	{
		j = json{
			{"DevicePATH", p.DevicePATH},
			{"DeviceDriver", p.DeviceDriver},
			{"DeviceIFormat", p.DeviceIFormat},
			{"DeviceWidth", p.DeviceWidth},
			{"DeviceHeight", p.DeviceHeight},
			{"DeviceFPS", p.DeviceFPS},
			{"enable", p.enable},
		};
	}
	void from_json(const json &j, ConfigCLA::VideoSettings &p)
	{
		j.at("DevicePATH").get_to(p.DevicePATH);
		j.at("DeviceDriver").get_to(p.DeviceDriver);
		j.at("DeviceIFormat").get_to(p.DeviceIFormat);
		j.at("DeviceWidth").get_to(p.DeviceWidth);
		j.at("DeviceHeight").get_to(p.DeviceHeight);
		j.at("DeviceFPS").get_to(p.DeviceFPS);
		j.at("enable").get_to(p.enable);
	}
	//===========================================================//
	void to_json(json &j, const RuAPSSys::ConfigCLA::CommonSettings &p)
	{
		j = json{
			{"COM_CastFrameIndex", p.COM_CastFrameIndex},
			{"COM_BroadCastEnable", p.COM_BroadCastEnable},
			{"COM_NormalCastEnable", p.COM_NormalCastEnable},
			{"COM_BroadCastBitRate", p.COM_BroadCastBitRate},
			{"COM_BroadCastPFrameSize", p.COM_BroadCastPFrameSize},
			{"BroadcastInterfaces", p.BroadcastInterfaces},
		};
	}
	void from_json(const json &j, RuAPSSys::ConfigCLA::CommonSettings &p)
	{
		j.at("COM_CastFrameIndex").get_to(p.COM_CastFrameIndex);
		j.at("COM_BroadCastEnable").get_to(p.COM_BroadCastEnable);
		j.at("COM_NormalCastEnable").get_to(p.COM_NormalCastEnable);
		j.at("COM_BroadCastBitRate").get_to(p.COM_BroadCastBitRate);
		j.at("COM_BroadCastPFrameSize").get_to(p.COM_BroadCastPFrameSize);
		j.at("BroadcastInterfaces").get_to(p.BroadcastInterfaces);
	}
	//===========================================================//
	void to_json(json &j, const ConfigCLA::PluginSettings &p)
	{
	}
	void from_json(const json &j, ConfigCLA::PluginSettings &p)
	{
	}
	//===========================================================//
	//===========================================================//
	void to_json(json &j, const UORBMessage::SystemStatus_t &p)
	{
	}
	void from_json(const json &j, UORBMessage::SystemStatus_t &p)
	{
	}
	//===========================================================//
	void to_json(json &j, const UORBMessage::ControllerStatus_t &p)
	{
	}
	void from_json(const json &j, UORBMessage::ControllerStatus_t &p)
	{
	}
	//===========================================================//
	void to_json(json &j, const UORBMessage::PluginStatus_t &p)
	{
	}
	void from_json(const json &j, UORBMessage::PluginStatus_t &p)
	{
	}

};

// Operator
namespace RuAPSSys
{
	inline void ConfigFileSync(std::string path)
	{
		std::ifstream config(path);
		std::string content((std::istreambuf_iterator<char>(config)),
							(std::istreambuf_iterator<char>()));

		json Container = json::parse(content);
#ifdef MODULE_APM
		ConfigCLA::APMConfig = SingleAPMAPI::readConfigFromJson(Container["APMConfig"]);
#endif
		ConfigCLA::VideoConfig = Container["VideoConfig"]["Device"].get<std::vector<ConfigCLA::VideoSettings>>();
		ConfigCLA::CommonConfig = Container["CommonConfig"].get<ConfigCLA::CommonSettings>();
		return; // just for breakpoint
	};
}