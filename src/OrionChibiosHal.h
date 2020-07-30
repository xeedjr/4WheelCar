#pragma once

#include "orion_protocol/orion_communication.h"

class OrionChibiosHal: public orion::Communication {
public:
	  OrionChibiosHal()  {};

	  size_t receiveAvailableBuffer(uint8_t *buffer, uint32_t size);
	  size_t receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout);
	  bool hasAvailableBuffer();
	  bool sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout);
};
