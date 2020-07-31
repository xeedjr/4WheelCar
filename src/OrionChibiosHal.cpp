#include "hal.h"
#include "usbcfg.h"

#include "OrionChibiosHal.h"

size_t OrionChibiosHal::receiveAvailableBuffer(uint8_t *buffer, uint32_t size) {

	chnRead(&SDU1, buffer, size);
}
size_t OrionChibiosHal::receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout) {

	chnRead(&SDU1, buffer, size);

}
bool OrionChibiosHal::hasAvailableBuffer() {


}

bool OrionChibiosHal::sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout) {

	chnWrite(&SDU1, buffer, size);
}
