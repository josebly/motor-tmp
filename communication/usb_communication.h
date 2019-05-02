#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H

#include "communication.h"
class USB;

class USBCommunication : public Communication {
 public:
    virtual void init();
    virtual int receive_data(ReceiveData * const data);
    virtual void send_data(const SendData &data);
 private:
    USB * usb_;
};

#endif
