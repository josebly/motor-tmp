#ifndef USB_H
#define USB_H

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_usb.h"

extern uint8_t usb_connected;

#define USBx USB_OTG_FS
class USB {
 public:
    // limited to 64 bytes right now
    void send_data32(uint8_t endpoint, const uint32_t *data, uint8_t length32) 
    {
        if (!usb_connected) {
            return;
        }

        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {  
            int count = 0;
            do {     
                USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
                if (count++ > 100) {
                    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
                    return;
                } 
            } while(!(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_INEPNE) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_EPDISD));
            USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 1 << (USB_OTG_GRSTCTL_TXFNUM_Pos + endpoint - 1)));
            while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS));     // wait on flush
        }
   
        USBx_INEP(endpoint)->DIEPTSIZ = 0;  // TODO necessary?
        USBx_INEP(endpoint)->DIEPTSIZ = sizeof(uint32_t) * length32 | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
        USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK ;
        for(int i=0; i<length32; i++) {
            USBx_DFIFO(endpoint) = data[i]; 
        }

        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

    }

    void send_data(uint8_t endpoint, const uint8_t *data, uint8_t length) {
        send_data32(endpoint, (uint32_t *) data, (length+3)/4);
    }

    // receive up to length bytes from endpoint, return number of bytes read
    int receive_data(uint8_t endpoint, uint8_t * const data, uint8_t length) {
        return 0;
    }
};

#endif
