#include "usb_communication.h"
#include "../peripheral/usb.h"
#include <cstring>

USB usb;

void USBCommunication::init() {
    usb_ = &usb;
}

int USBCommunication::receive_data(ReceiveData * const data) {
    return usb_->receive_data(2, reinterpret_cast<uint32_t * const>(data), sizeof(*data)/sizeof(uint32_t));
}

void USBCommunication::send_data(const SendData &data) {
    usb_->send_data(2, reinterpret_cast<const uint8_t *>(&data), sizeof(SendData));
}

void send_string(const char * str) {
    if (!usb.tx_active(1)) {
        usb.send_data(1, (const uint8_t *) str, std::strlen(str)+1);
    }
}

char *get_string() {
    static char buf[64];
    int count = usb.receive_data(1, (uint32_t *) buf, 64);
    buf[count] = 0;
    if (count) {
        return buf;
    } else {
        return NULL;
    }
}