
#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"
#include "TB6612FNG.h"
#include "VelocityEncoder.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "imu.h"

extern const SerialConfig sd1_config;
extern PWMConfig pwm3cfg;

void Timer1_Callback  (void const *arg) {
	palToggleLine(LINE_LED1);
};
osTimerDef (Timer1, Timer1_Callback);

TB6612FNG driver;
VelocityEncoder velA;
VelocityEncoder velB;
BMP280  		bmp280;

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
  
  bmp280.init(BMP280_I2C_DRIVER);
  imu_setup();

  driver.drive(TB6612FNG::kA, TB6612FNG::kCW, 20);
  driver.drive(TB6612FNG::kB, TB6612FNG::kCW, 20);

  auto pres = bmp280.checkPresence();
  bmp280.writeCntrlMeasRegister(0b001, 0b001, 0b00);
  bmp280.writeCntrlMeasRegister(0b001, 0b001, 0b01);
  bmp280.readCompensationRegister();
  auto T = bmp280.readTemperatureTimeout();
  auto P = bmp280.readPresserTimeout();

  while(1) {
	  imu_loop();

//		driver.drive(TB6612FNG::kA, TB6612FNG::kCW, 20);
//    driver.drive(TB6612FNG::kB, TB6612FNG::kCCW, 20);
//    osDelay(2000);
//		driver.drive(TB6612FNG::kA, TB6612FNG::kCCW, 20);
//    driver.drive(TB6612FNG::kB, TB6612FNG::kCW, 20);
//    osDelay(2000);
//    static auto speed = velA.getSpeed();
//
//		sdWrite(&DEBUG_UART_DRIVE, (uint8_t*)"Test\n\r", sizeof("Test\n\r") - 1);
	}
}
