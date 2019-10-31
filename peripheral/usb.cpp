#include "usb.h"

uint8_t device_address_ = 0;
    uint8_t setup_data[64];
    uint16_t interface_ = 0;
    uint32_t rx_data_[4][16] = {};
   volatile int sending_=0;
    bool new_rx_data_[4] = {};