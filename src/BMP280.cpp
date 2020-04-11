#include "BMP280.h"

msg_t BMP280::_writeRegister (uint8_t register_addr, uint8_t* data, uint8_t len) {

	i2cAcquireBus(i2cp);
	auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
								  &register_addr, 1,
								  NULL, 0,
								  TIME_MS2I(1000));
	if (status != MSG_OK) {
		i2cReleaseBus(i2cp);
		return status;
	}

	if (data != nullptr) {
		auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
									  data, len,
									  NULL, 0,
									  TIME_MS2I(1000));
		if (status != MSG_OK) {
			i2cReleaseBus(i2cp);
			return status;
		}
	};
	i2cReleaseBus(i2cp);

	return status;
}

msg_t BMP280::_readRegister (uint8_t register_addr, uint8_t* data, uint8_t len) {

	i2cAcquireBus(i2cp);
	auto status = i2cMasterTransmitTimeout(i2cp, ADDRESS,
								  &register_addr, 1,
								  data, len,
								  TIME_MS2I(1000));
	if (status != MSG_OK) {
		i2cReleaseBus(i2cp);
		return status;
	}
	i2cReleaseBus(i2cp);

	return status;
}

void BMP280::softReset () {
	_writeRegister(RESET, nullptr, 0);
}

bool BMP280::checkPresence () {
	uint8_t rx_data[1] = {0};
	auto status = _readRegister(ID, rx_data, sizeof(rx_data));

	if (status != MSG_OK) {
	  return false;
	}

	if (rx_data[0] != 0x58) {
	  return false;
	}

	return true;
};


void BMP280::writeConfigRegister (uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {
  uint8_t tx_data[1] = {ID};

  tx_data[0] = ((t_sb << 4) + (filter << 1) + (spi3w_en));

  _writeRegister(CONFIG, tx_data, sizeof(tx_data));
};
