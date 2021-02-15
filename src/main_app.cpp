/*
 * main_app.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Bogdan
 */
#include "qpcpp.hpp"
#include <new>
#include "main_app.h"

#include "BusinessLogicAO/BusinessLogic.h"
#include "BusinessLogicAO/VirtualComPort.h"
#include "orion_protocol/orion_frame_transport.h"
#include "orion_protocol/orion_cobs_framer.h"
#include "orion_protocol/orion_header.h"
#include "orion_protocol/orion_minor.h"

#include <Motor.h>
#include "TB6612FNG.h"
#include "RPMEncoderOptical.h"
#include "cmsis_os.h"
#include "MPU9250.h"
#include "MPU9250HALSTM32HALI2C.h"
#include "Sensors.h"
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

carmen_hardware::VirtualComPort *p_com_port;
orion::COBSFramer *p_cobs_framer;
orion::FrameTransport *p_frame_transport;
orion::Minor *p_minor;
TB6612FNG *driver;
RPMEncoderOptical *enc1;
RPMEncoderOptical *enc2;
MPU9250FIFO *mpu;
MPU9250HALSTM32HALI2C *mpuHal;
HallEncoder *encoder1;
WheelMotorEncoder *wheel_encode;
WheelMotorEncoder *wheel_encode2;
USTrigger *us_trigger;
USSensor *us_sensor;

/// AO
sensors::Sensors *sensorsp;
motor::Motor *motorp;
#ifdef USE_ROSSERIAL
    ros_serial::Rosserial *rosserialp;
#else
    business_logic::BusinessLogic *p_business_logic;
#endif

extern "C" void send_new_command_event(void)
{
#ifdef USE_ROSSERIAL
#else
  if (nullptr != p_business_logic)
  {
    p_business_logic->sendNewCommandEvent();
  }
#endif
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
	case (IMU_INT_Pin) : {
		if (sensorsp != nullptr)
		    sensorsp->data_ready();
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


    mpuHal = new MPU9250HALSTM32HALI2C(&hi2c1, 0x68);
    mpu = new (mmm) MPU9250FIFO(mpuHal);

    p_com_port = new carmen_hardware::VirtualComPort();
    p_cobs_framer = new orion::COBSFramer();
    p_frame_transport = new orion::FrameTransport(p_com_port, p_cobs_framer);
    p_minor = new orion::Minor(p_frame_transport);

    /// AO
#ifdef USE_ROSSERIAL
    rosserialp = new ros_serial::Rosserial(&htim16);
#else
    p_business_logic = new business_logic::BusinessLogic(p_minor);
#endif

#ifdef USE_ROSSERIAL
    motorp = new motor::Motor(driver, wheel_encode, wheel_encode2, rosserialp);
    rosserialp->setMotorAO(motorp);
#else
    motorp = new motor::Motor(driver, wheel_encode, wheel_encode2, p_business_logic);
    p_business_logic->setMotor(motorp);
#endif

#ifdef USE_ROSSERIAL
    sensorsp = new sensors::Sensors(mpu, us_sensor,
                                        rosserialp);
#else
    sensorsp = new sensors::Sensors(mpu, us_sensor,
                                    p_business_logic);
#endif

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
#ifdef USE_ROSSERIAL
	if (rosserialp)
	    rosserialp->startAO();
#else
    if (p_business_logic)
        p_business_logic->startAO();
#endif
	if (sensorsp)
	    sensorsp->startAO();

  //  motorp->SetSpeedL(10.0);
  //  motorp->SetSpeedR(10.0);

	QF::run();
}


void DefaultTask(void const * argument)
{
	  for(;;)
	  {
		  osDelay(1000);
	  }
}
