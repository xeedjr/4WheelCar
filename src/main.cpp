
#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"

void Timer1_Callback  (void const *arg) {
	
};
osTimerDef (Timer1, Timer1_Callback);

int main () {
	/*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
	halInit();
	osKernelInitialize();
	
	auto id2 = osTimerCreate (osTimer(Timer1), osTimerPeriodic, nullptr);
	if (id2 != nullptr)  {
		// Periodic timer created
	}
	  
	while(1) {
		
	}
}