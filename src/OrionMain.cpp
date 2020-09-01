#include "OrionMain.h"

static void Thread (void const *arg) {
	OrionMain* pthis = (OrionMain*)arg;
	pthis->Thread();
};                           // function prototype for Thread_1
osThreadDef (Thread, osPriorityNormal, 1, 512);            // define Thread_1

void OrionMain::init()
{
	frame_transport = new orion::FrameTransport(&orion_chibios_hal, &cobs_framer);
	orion_minor = new orion::Minor(frame_transport);

	  id = osThreadCreate (osThread (Thread), this);         // create the thread
	  if (id == NULL) {                                        // handle thread creation
	    // Failed to create a thread
	  }
}  

void OrionMain::Thread()
{
	while(true) {
		orion_minor->receiveCommand();
	}
}
