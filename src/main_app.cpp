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
#include "cmsis_os.h"
#include "MPU9250.h"
#include "MPU9250HALSTM32HALI2C.h"
#include "IMU.h"
#include "tim.h"
#include <ros.h>
#include "Rosserial.h"
#include "HallEncoder.h"
#include "WheelMotorEncoder.h"
#include "USTrigger.h"
#include "USSensor.h"

#ifdef USE_HAL_UART_REGISTER_CALLBACKS
#include <new>
#endif

using namespace std;
using namespace QP;

TB6612FNG *driver;
RPMEncoderOptical *enc1;
RPMEncoderOptical *enc2;
MPU9250FIFO *mpu;
MPU9250HALSTM32HALI2C *mpuHal;
IMU *imu;
motor::Motor *motorp;
ros_serial::Rosserial *rosserialp;
HallEncoder *encoder1;
WheelMotorEncoder *wheel_encode;
WheelMotorEncoder *wheel_encode2;
USTrigger *us_trigger;
USSensor *us_sensor;

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
	HAL_UART_Transmit(&huart3, &c, 1, 1000);
}

extern "C" int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 1000);

	return len;
}

extern "C" Q_NORETURN Q_onAssert(char const * const module, int_t const loc) {
    exit(-1);
}
void QF::onStartup(void) {}
void QF::onCleanup(void) {}

uint8_t poolStor2[40*1024];
uint8_t poolStor3[40*1024];

uint8_t mmm[sizeof(MPU9250FIFO)] = {0};

void main_cpp(void) {
    QF::init(); // initialize the framework

	QF::poolInit(poolStor2,
		sizeof(poolStor2),
		50);
	QF::poolInit(poolStor3,
		sizeof(poolStor3),
		100);


	us_trigger = new USTrigger(&htim5);
	us_sensor = new USSensor(&htim2);

//	auto clock = HAL_RCC_GetPCLK1Freq()/htim2.Init.Prescaler;
//	auto pulses = 0.04/(1.0/(float)clock);
//	enc1 = new RPMEncoderOptical(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, pulses);

	//encoder1 = new HallEncoder(&htim4);
	wheel_encode = new WheelMotorEncoder(&htim4, 34);
	wheel_encode2 = new WheelMotorEncoder(&htim3, 34);

	driver = new TB6612FNG();
	driver->init({LINE_TB66_AIN2_GPIO_Port, LINE_TB66_AIN2_Pin},
	             {LINE_TB66_AIN1_GPIO_Port, LINE_TB66_AIN1_Pin},
	             {LINE_TB66_BIN2_GPIO_Port, LINE_TB66_BIN2_Pin},
	             {LINE_TB66_BIN1_GPIO_Port, LINE_TB66_BIN1_Pin},
	             {LINE_TB66_STBY_GPIO_Port, LINE_TB66_STBY_Pin},
		  &htim15, TIM_CHANNEL_1,
		  &htim15, TIM_CHANNEL_2);

	motorp = new motor::Motor(driver, wheel_encode, wheel_encode2,
	                            [](double* vals, uint8_t n){ /// Wheel position
	                                rosserialp->motor_pub(vals[0], vals[1]);
	                            });

    //communication = new Communication(&huart6, motorp);
	rosserialp = new ros_serial::Rosserial(&htim16, motorp);

	mpuHal = new MPU9250HALSTM32HALI2C(&hi2c1, 0x68);
	mpu = new (mmm) MPU9250FIFO(mpuHal);
	imu = new IMU(mpu,
	                [](float r, float p, float y){ /// IMU
	                    // none
	                },
                    [](float* vals, uint8_t n){ /// US
	                    rosserialp->sonar_pub(vals[0], vals[1]);
                    },
                    [](float* vals, uint8_t n){ /// TOF
                        /// none
                    });


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

	if (motorp)
	    motorp->startAO();
	if (rosserialp)
	    rosserialp->startAO();
	if (imu)
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
