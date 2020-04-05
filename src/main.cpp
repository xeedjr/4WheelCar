
#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"
#include "TB6612FNG.h"

extern const SerialConfig sd1_config;

//64kHz
#define PERIOD  250U

static PWMConfig pwm3cfg = {
  F_CPU,                            /* PWM frequency.         */
  PERIOD,                           /* PWM period.            */
  NULL,                             /* TODO: comment.         */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 1 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 2 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 3 actived. */
  },
};

void Timer1_Callback  (void const *arg) {
	palTogglePad(IOPORT2, PORTB_LED1);
};
osTimerDef (Timer1, Timer1_Callback);

TB6612FNG driver;

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
	
	sdStart(&SD1, &sd1_config);
  pwmStart(&PWMD3, &pwm3cfg);

	palClearPad(IOPORT2, PORTB_LED1);
  
	auto id2 = osTimerCreate (osTimer(Timer1), osTimerPeriodic, nullptr);
	if (id2 != nullptr)  {
		// Periodic timer created
	}
	osTimerStart(id2, 1000);
	
  driver.init(LINE_TB66_PWMA, LINE_TB66_AIN2, LINE_TB66_AIN1,
              LINE_TB66_PWMB, LINE_TB66_BIN2, LINE_TB66_BIN1,
              LINE_TB66_STBY,
              &PWMD3, PWM_CHAN_TB66_A,
              &PWMD3, PWM_CHAN_TB66_B);
  
  while(1) {
		driver.drive(TB6612FNG::kA, TB6612FNG::kCW, 20);
    driver.drive(TB6612FNG::kB, TB6612FNG::kCCW, 20);
    osDelay(2000);
		driver.drive(TB6612FNG::kA, TB6612FNG::kCCW, 20);
    driver.drive(TB6612FNG::kB, TB6612FNG::kCW, 20);
    osDelay(2000);



		sdWrite(&SD1, (uint8_t*)"Test\n\r", sizeof("Test\n\r") - 1);
	}
}