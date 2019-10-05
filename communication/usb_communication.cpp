#include "usb_communication.h"
#include "../peripheral/usb.h"
#include <cstring>

USB usb;

void USBCommunication::init() {
    usb_ = &usb;
}

int USBCommunication::receive_data(ReceiveData * const data) {
    return usb_->receive_data(2, reinterpret_cast<uint32_t * const>(data), sizeof(data)/sizeof(uint32_t));
}

void USBCommunication::send_data(const SendData &data) {
    usb_->send_data(2, reinterpret_cast<const uint8_t *>(&data), sizeof(SendData));
}
