#pragma once

#include "cmsis_os.h"
#include "orion_protocol/orion_cobs_framer.h"
#include "OrionChibiosHal.h"
#include "orion_protocol/orion_minor.h"
#include "orion_protocol/orion_frame_transport.h"

class OrionMain  {
	  osThreadId id;

	  orion::COBSFramer cobs_framer;
	  OrionChibiosHal orion_chibios_hal;
	  orion::FrameTransport*  frame_transport;
	  orion::Minor* orion_minor;
public:
	OrionMain() {};
	void init();
	void Thread();
};
