#pragma once

#include "hal.h"

class BMP280  {
	I2CDriver *i2cp;

	#define ADDRESS  	(0xEF>>1)
	#define TEMPXLSB	(0xFC)
	#define TEMP_LSB	(0xFB)
	#define TEMP_MSB	(0xFA)
	#define PRESS_XLSB	(0xF9)
	#define PRESS_LSB	(0xF8)
	#define PRESS_MSB	(0xF7)
	#define CONFIG		(0xF5)
	#define CNTRL_MEAS	(0xF4)
	#define STATUS		(0xF3)
	#define RESET		(0xE0)
	#define ID			(0xD0)
private:
	msg_t _writeRegister (uint8_t register_addr, uint8_t* data, uint8_t len);
	msg_t _readRegister (uint8_t register_addr, uint8_t* data, uint8_t len);
public:
  BMP280() {};
  void init (I2CDriver *i2cp) {
	  this->i2cp = i2cp;
  };
  bool checkPresence ();
  void softReset ();
  void writeConfigRegister (uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
};
