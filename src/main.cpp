
#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"

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
	
	while(1) {
		
	}
}