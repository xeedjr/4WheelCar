#include "BMP280.h"

void BMP280::_writeRegister (uint8_t register_addr, uint8_t data) {

	uint8_t buffer[2] = {register_addr, data};

	i2cAcquireBus(i2cp);
	auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
								  buffer, sizeof(buffer),
								  NULL, 0,
								  TIME_MS2I(1000));
	osalDbgCheck(MSG_OK == status);
	i2cReleaseBus(i2cp);
}

void BMP280::_readRegisterOne (uint8_t register_addr, uint8_t* data) {

	uint8_t rx_buff[2] = {0};

	i2cAcquireBus(i2cp);
	auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
								  &register_addr, 1,
								  rx_buff, sizeof(rx_buff),
								  TIME_MS2I(1000));
	osalDbgCheck(MSG_OK == status);
	i2cReleaseBus(i2cp);
	*data = rx_buff[0];
}

void BMP280::_readRegister (uint8_t register_addr, uint8_t* data, uint8_t len) {
	i2cAcquireBus(i2cp);
	auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
								  &register_addr, 1,
								  data, len,
								  TIME_MS2I(1000));
	osalDbgCheck(MSG_OK == status);
	i2cReleaseBus(i2cp);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of вЂњ5123вЂќequals 51.23 DegC.
// t_fine carries fine temperature as global valueint32_tt_fine;
int32_t BMP280::compensate_T_int32(int32_t adc_T){
	int32_t var1, var2, T;

	var1=((((adc_T>>3) - ((int32_t)compensation.dig_T1<<1))) * ((int32_t)(compensation.dig_T2)))>>11;
	var2  = (((((adc_T>>4) - ((int32_t)compensation.dig_T1)) * ((adc_T>>4) - ((int32_t)compensation.dig_T1))) >> 12) * ((int32_t)compensation.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T  = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of вЂњ24674867вЂќrepresents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BMP280::compensate_P_int64(int32_t adc_P){
	int64_t var1, var2, p;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)compensation.dig_P6;
	var2 = var2 + ((var1*(int64_t)compensation.dig_P5)<<17);
	var2 = var2 + (((int64_t)compensation.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)compensation.dig_P3)>>8) + ((var1 * (int64_t)compensation.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)compensation.dig_P1)>>33;
	if(var1 == 0){
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)compensation.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)compensation.dig_P8) * p) >> 19;
	p= ((p + var1 + var2) >> 8) + (((int64_t)compensation.dig_P7)<<4);
	return(uint32_t)p;
}

void BMP280::softReset () {
	_writeRegister(RESET, 0);
}

bool BMP280::checkPresence () {
	uint8_t rx_data = {0};
	_readRegisterOne(ID, &rx_data);

	if (rx_data != 0x58) {
	  return false;
	}

	return true;
};


void BMP280::writeConfigRegister (uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {
	Config config = {0};

	config.d.t_sb = t_sb;
	config.d.filter = filter;
	config.d.spi3w_en = spi3w_en;

   _writeRegister(CONFIG, config.data);
};

void BMP280::readConfigRegister (uint8_t* t_sb, uint8_t* filter, uint8_t* spi3w_en) {
	Config config = {0};

	_readRegisterOne(CONFIG, &config.data);

	*t_sb = config.d.t_sb;
	*filter = config.d.filter;
	*spi3w_en = config.d.spi3w_en;
};

void BMP280::writeCntrlMeasRegister (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode) {
	CntrlMeas cntrl_meas = {0};

	cntrl_meas.d.osrs_t = osrs_t;
	cntrl_meas.d.osrs_p = osrs_p;
	cntrl_meas.d.mode = mode;

   _writeRegister(CNTRL_MEAS, cntrl_meas.data);
};

void BMP280::readCntrlMeasRegister (uint8_t* osrs_t, uint8_t* osrs_p, uint8_t* mode) {
	CntrlMeas cntrl_meas = {0};

	_readRegisterOne(CNTRL_MEAS, &cntrl_meas.data);

	*osrs_t = cntrl_meas.d.osrs_t;
	*osrs_p = cntrl_meas.d.osrs_p;
	*mode = cntrl_meas.d.mode;
};

void BMP280::readStatusRegister (uint8_t* measuring, uint8_t* im_update) {
	Status status = {0};

	_readRegisterOne(STATUS, &status.data);

	*measuring = status.d.measuring;
	*im_update = status.d.im_update;
};

void BMP280::readCompensationRegister (void) {
	_readRegister(CALIBRATION, reinterpret_cast<uint8_t*>(&compensation), sizeof(compensation));
};

bool BMP280::isMeasuring (void) {
	uint8_t measuring;
	uint8_t im_update;

	readStatusRegister (&measuring, &im_update);
	if (measuring == 0) {
		return false;
	};

	return true;
};

float BMP280::readTemperatureTimeout (void) {
	uint8_t adca[3] = {0}; // MSP8 LSP8 XLSP4
	uint32_t adc = 0;
	int32_t T = 0;

	writeCntrlMeasRegister(0b001, 0b001, 0b01);

	while(isMeasuring()) {osalThreadSleepMilliseconds(5);}

	_readRegister(TEMP_MSB, reinterpret_cast<uint8_t*>(&adca), sizeof(adca));

	adc = (adca[0] << (8+4)) + (adca[1] << (4)) + ((adca[2] >> 4) & 0x0F );

	T = compensate_T_int32(adc);

	return T / 100.0;
};

// Return hPa
float BMP280::readPresserTimeout (void) {
	uint8_t adca[3] = {0}; // MSP8 LSP8 XLSP4
	uint32_t adc = 0;
	uint32_t P = 0;

	writeCntrlMeasRegister(0b001, 0b001, 0b01);

	while(isMeasuring()) {osalThreadSleepMilliseconds(5);}

	_readRegister(PRESS_MSB, reinterpret_cast<uint8_t*>(&adca), sizeof(adca));

	adc = (adca[0] << (8+4)) + (adca[1] << (4)) + ((adca[2] >> 4) & 0x0F );

	P = compensate_P_int64(adc);

	return (P / 256.0)/100.0;
}

