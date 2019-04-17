#include "usb_communication.h"
#include "../peripheral/usb.h"

void USBCommunication::init() {
    usb_ = new USB;
}

int USBCommunication::receive_data(ReceiveData * const data) {

    return 0;
}

void USBCommunication::send_data(const SendData &data) {
    usb_->send_data(2, reinterpret_cast<const uint8_t *>(&data), sizeof(SendData));
}
