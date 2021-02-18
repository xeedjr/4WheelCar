#include "VirtualComPort.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "logging.h"

namespace carmen_hardware
{

VirtualComPort::VirtualComPort()
{
}

size_t VirtualComPort::receiveAvailableBuffer(uint8_t *buffer, uint32_t size)
{
    size_t actual_size = dequeue_input_buffer(buffer, size);
    // DEBUG
    if (actual_size > 0)
    {
        LOG_DEBUG("VirtualComPort::receiveAvailableBuffer Received buffer\n");
    }
    // DEBUG
    return (actual_size);
}

size_t VirtualComPort::receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout)
{
    // TODO: Add logic to wait for timeout in case if no data is available
    size_t actual_size = this->receiveAvailableBuffer(buffer, size);
    // DEBUG
    if (actual_size > 0)
    {
        LOG_DEBUG("VirtualComPort::receiveBuffer Received buffer\n");
    }
    // DEBUG
    return (actual_size);
}

bool VirtualComPort::hasAvailableBuffer()
{
     bool result = has_items_input_buffer();
    // DEBUG
    if (true == result)
    {
        LOG_DEBUG("VirtualComPort::hasAvailableBuffer Has buffer\n");
    }
    // DEBUG
     return (result);
}

bool VirtualComPort::sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout)
{
    assert(NULL != buffer);
    assert(0 != size);
    assert(size == (size & 0xFFFF));

    LOG_DEBUG("VirtualComPort::sendBuffer Sending buffer\n");

    uint8_t status = CDC_Transmit_FS(buffer, (uint16_t)(size & 0xFFFF));
    return (USBD_OK == status);
}

}
