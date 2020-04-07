
#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"
#include "TB6612FNG.h"
#include "VelocityEncoder.h"

extern const SerialConfig sd1_config;
extern PWMConfig pwm3cfg;

void Timer1_Callback  (void const *arg) {
	palToggleLine(LINE_LED1);
};
osTimerDef (Timer1, Timer1_Callback);

TB6612FNG driver;
VelocityEncoder velA;
VelocityEncoder velB;

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
	
	sdStart(&DEBUG_UART_DRIVE, NULL);
	sdStart(&RASPBERY_UART_DRIVE, NULL);
	pwmStart(PWM_TB66_A, &pwm3cfg);

	palClearLine(LINE_LED1);
  
	auto id2 = osTimerCreate (osTimer(Timer1), osTimerPeriodic, nullptr);
	if (id2 == nullptr)  {
		// Periodic timer created
		chSysHalt(__FUNCTION__);
	}
	osTimerStart(id2, 1000);
	
  driver.init(LINE_TB66_PWMA, LINE_TB66_AIN2, LINE_TB66_AIN1,
              LINE_TB66_PWMB, LINE_TB66_BIN2, LINE_TB66_BIN1,
              LINE_TB66_STBY,
			  PWM_TB66_A, PWM_CHAN_TB66_A,
			  PWM_TB66_B, PWM_CHAN_TB66_B);

  velA.init(VELOCITY_A_LINE);
  velB.init(VELOCITY_B_LINE);
  
  driver.drive(TB6612FNG::kA, TB6612FNG::kCW, 20);
  driver.drive(TB6612FNG::kB, TB6612FNG::kCW, 20);



  while(1) {
//		driver.drive(TB6612FNG::kA, TB6612FNG::kCW, 20);
//    driver.drive(TB6612FNG::kB, TB6612FNG::kCCW, 20);
//    osDelay(2000);
//		driver.drive(TB6612FNG::kA, TB6612FNG::kCCW, 20);
//    driver.drive(TB6612FNG::kB, TB6612FNG::kCW, 20);
    osDelay(2000);
    static auto speed = velA.getSpeed();

		sdWrite(&DEBUG_UART_DRIVE, (uint8_t*)"Test\n\r", sizeof("Test\n\r") - 1);
	}
}
