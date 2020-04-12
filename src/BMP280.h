#pragma once

#include "hal.h"

class BMP280  {
	I2CDriver *i2cp;

	int32_t t_fine;
	struct Compensation {
		uint16_t dig_T1;	//0x88/0x89
		int16_t dig_T2;
		int16_t dig_T3;
		uint16_t dig_P1;
		int16_t dig_P2;
		int16_t dig_P3;
		int16_t dig_P4;
		int16_t dig_P5;
		int16_t dig_P6;
		int16_t dig_P7;
		int16_t dig_P8;
		int16_t dig_P9;
		uint16_t reserved; //0xA0/0xA1
	} compensation;

	union Config {
		struct {
			uint8_t t_sb:3;
			uint8_t filter:3;
			uint8_t res:1;
			uint8_t spi3w_en:1;
		} d;
		uint8_t data;
	};

	union CntrlMeas {
		struct {
			uint8_t osrs_t:3;
			uint8_t osrs_p:3;
			uint8_t mode:2;
		} d;
		uint8_t data;
	};
	union Status {
		struct {
			uint8_t res:4;
			uint8_t measuring:1;
			uint8_t res2:2;
			uint8_t im_update:1;
		} d;
		uint8_t data;
	};

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
	#define CALIBRATION (0x88)
private:
	uint32_t swap_uint32( uint32_t val )
	{
	    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
	    return (val << 16) | (val >> 16);
	};
	void _writeRegister (uint8_t register_addr, uint8_t data);
	void _readRegisterOne (uint8_t register_addr, uint8_t* data);
	void _readRegister (uint8_t register_addr, uint8_t* data, uint8_t len);
	int32_t compensate_T_int32(int32_t adc_T);
	uint32_t compensate_P_int64(int32_t adc_P);
public:
  BMP280() {};
  void init (I2CDriver *i2cp) {
	  this->i2cp = i2cp;
  };
  bool checkPresence ();
  void softReset ();
  void writeConfigRegister (uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
  void readConfigRegister (uint8_t* t_sb, uint8_t* filter, uint8_t* spi3w_en);
  void writeCntrlMeasRegister (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
  void readCntrlMeasRegister (uint8_t* osrs_t, uint8_t* osrs_p, uint8_t* mode);
  void readStatusRegister (uint8_t* measuring, uint8_t* im_update);
  void readCompensationRegister (void);
  bool isMeasuring (void);
  float readTemperatureTimeout (void);
  float readPresserTimeout (void);

};
