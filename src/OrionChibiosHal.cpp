#include "hal.h"
#include "usbcfg.h"

#include "OrionChibiosHal.h"

size_t OrionChibiosHal::receiveAvailableBuffer(uint8_t *buffer, uint32_t size) {

	size_t received_size = 0;
	while (received_size <= 0) {
		received_size = chnRead(&SDU1, buffer, size);
	};

	return received_size;
}
size_t OrionChibiosHal::receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout) {

	size_t received_size = 0;
	while (received_size <= 0) {
		received_size = chnRead(&SDU1, buffer, size);
	};

	return received_size;

}
bool OrionChibiosHal::hasAvailableBuffer() {


}

bool OrionChibiosHal::sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout) {

	chnWrite(&SDU1, buffer, size);
}
