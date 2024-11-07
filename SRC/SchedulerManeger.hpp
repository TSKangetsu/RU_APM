#pragma once
#include <queue>
#include <string>
#include <csignal>
#include <sstream>
#include <iostream>
#include "_Mainfunction/UORBMessage.hpp"
#ifdef MODULE_APM
#include "_Mainfunction/APMController.hpp"
#endif
#include "_Mainfunction/PLGController.hpp"
#include "_Mainfunction/VIDController.hpp"
#include "_Mainfunction/COMController.hpp"
#include "_Excutable/LogPublicator.hpp"

#include "./RPiSingleAPM/src/_thirdparty/FlowController.hpp"

#define FileConfigTarget "/etc/APSconfig.json"
#define IsServerSiteEnable true
#define IsClientSiteEnable false
#define FlowExchangeHZ 250.f
#define FlowSytemMonHZ 50.f

using UORB = RuAPSSys::UORBMessage;
namespace LOG = LOGPublicator;

namespace RuAPSSys
{
	inline static int APSSystemSignal;
	inline std::queue<std::string> SystemMessage;
	//
	class SchedulerController
	{
	public:
		SchedulerController()
		{
			// Step 1. Read Config from /boot/APSconfig.json.
			RuAPSSys::ConfigFileSync(FileConfigTarget);
#ifdef MODULE_APM
			// Step 2. Load up SingleController.
			APMController.reset(new APMController_t());
#endif
			// Step 3. Load up Camera System.
			VIDController.reset(new VIDController_t());
			
			// Step 4. Load up Message and VideoStream BoradCast;
			COMController.reset(new COMController_t());
			// ...
			// Step N. Start up all process complete.
			LOG::LogPrintSTDIO(_SYS << STARTUPCOMPLETE);
		};

		void SystemReboot();

		SchedulerController &&SystemMonitorReg(); // SystemMonitor will check system Status 50HZ and Report Log to file

	private:
		std::unique_ptr<FlowThread> SystemMonitoThread;
#ifdef MODULE_APM
		std::unique_ptr<APMController_t> APMController;
#endif
		std::unique_ptr<PLGController_t> PLGController;
		std::unique_ptr<VIDController_t> VIDController;
		std::unique_ptr<COMController_t> COMController;
	};
}

RuAPSSys::SchedulerController &&RuAPSSys::SchedulerController::SystemMonitorReg()
{
	SystemMonitoThread.reset(
		new FlowThread(
			[&]()
			{
				// Check message queue from other thread and print
				if (UORB::SystemStatus.SystemMessage.size() > 0)
				{
					LOG::LogPrintSTDIO(UORB::SystemStatus.SystemMessage.front());
					UORB::SystemStatus.SystemMessage.pop();
				}

				// Stop Signal Handle Check.
				if (APSSystemSignal == SIGTERM || APSSystemSignal == SIGINT)
				{
					// Call APMControll Stop all threads.
					LOG::LogPrintSTDIO(_SYS << EXITPROCESSACTD);
					// Shut off all Controller
					COMController.reset();
					LOG::LogPrintSTDIO(_VID << COMEXITPROCESSD);
					VIDController.reset();
					LOG::LogPrintSTDIO(_VID << VIDEXITPROCESSD);
#ifdef MODULE_APM
					APMController.reset(); // This Will Block untill APM complete Stop.
					LOG::LogPrintSTDIO(_APM << APMEXITPROCESSD);
#endif
					// Exiting whole progress
					LOG::LogPrintSTDIO(_SYS << SYSTEMEXITEDCAL);
					exit(0);
				}
			},
			FlowSytemMonHZ));
	// Call Signal Handler
	std::signal(SIGINT,
				[](int Signal) -> void
				{
					RuAPSSys::APSSystemSignal = Signal;
					LOG::LogPrintSTDIO(_SYS << SIGNALRECVINPUT << Signal << "\n");
				});
	std::signal(SIGTERM,
				[](int Signal) -> void
				{
					RuAPSSys::APSSystemSignal = Signal;
					LOG::LogPrintSTDIO(_SYS << SIGNALRECVINPUT << Signal << "\n");
				});
	std::signal(SIGPIPE, [](int Signal) -> void
				{
					//
					//
				});
	// Block Runner life
	SystemMonitoThread->FlowWait();
	return std::move(*this);
};

void RuAPSSys::SchedulerController::SystemReboot() {

};