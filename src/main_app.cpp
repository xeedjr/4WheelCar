/*
 * main_app.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Bogdan
 */
#include <Motor.h>
#include "qpcpp.hpp"
#include <new>

#include "TB6612FNG.h"
#include "RPMEncoderOptical.h"
#include "tim.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "MPU9250.h"
#include "MPU9250HALSTM32HALI2C.h"
#include "IMU.h"
#include "Communication.h"
#include <ros.h>
#include "Rosserial.h"

using namespace std;
using namespace QP;

TB6612FNG *driver;
RPMEncoderOptical *enc1;
RPMEncoderOptical *enc2;
MPU9250FIFO *mpu;
MPU9250HALSTM32HALI2C *mpuHal;
IMU *imu;
motor::Motor *motorp;
Communication *communication;
ros_serial::Rosserial *rosserialp;

volatile static float time = 0;

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
	case (GPIO_PIN_5) : {
		if (imu != nullptr)
			imu->data_ready();
		break;
	}
	}
}


extern "C" int __io_putchar(int ch) {
	uint8_t c = (char)ch;
	HAL_UART_Transmit(&huart1, &c, 1, 1000);
}

extern "C" int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);

	return len;
}

extern "C" Q_NORETURN Q_onAssert(char const * const module, int_t const loc) {
    exit(-1);
}
void QF::onStartup(void) {}
void QF::onCleanup(void) {}

uint8_t poolStor2[1024];
uint8_t poolStor3[1024];

uint8_t mmm[sizeof(MPU9250FIFO)] = {0};

void main_cpp(void) {
    QF::init(); // initialize the framework

	QF::poolInit(poolStor2,
		sizeof(poolStor2),
		50);
	QF::poolInit(poolStor3,
		sizeof(poolStor3),
		100);


//	auto clock = HAL_RCC_GetPCLK1Freq()/htim2.Init.Prescaler;
//	auto pulses = 0.04/(1.0/(float)clock);
//	enc1 = new RPMEncoderOptical(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, pulses);

	driver = new TB6612FNG();
	driver->init(GPIOA,
		  LINE_TB66_AIN2_Pin, LINE_TB66_AIN1_Pin,
		  LINE_TB66_BIN2_Pin, LINE_TB66_BIN1_Pin,
		  LINE_TB66_STBY_Pin,
		  &htim3, TIM_CHANNEL_2,
		  &htim3, TIM_CHANNEL_1);

	motorp = new motor::Motor(driver, enc1, nullptr);

    //communication = new Communication(&huart6, motorp);
	rosserialp = new ros_serial::Rosserial();
/*
	mpuHal = new MPU9250HALSTM32HALI2C(&hi2c1, 0x68);
	mpu = new (mmm) MPU9250FIFO(mpuHal);
	imu = new IMU(mpu, communication);
*/

	/// Start QP

	auto t = xTimerCreate("QPRoootTimer",
	                     (10 / portTICK_PERIOD_MS), ///< 10ms tick
	                     pdTRUE,
	                     ( void * ) 0,
	                     [](TimerHandle_t xTimer){
								configASSERT( xTimer );
								QF::TICK_X(0U, nullptr);  // perform the QF clock tick processing
						}
	                   );
	xTimerStart( t, 0 );

	motorp->startAO();
	rosserialp->startAO();
	//communication->startAO();
	imu->startAO();

	QF::run();
}


void DefaultTask(void const * argument)
{
	  for(;;)
	  {
		  osDelay(1000);
	  }
}
