#ifndef USB_H
#define USB_H

#include <cstdint>
#include "stm32f4xx_ll_usb.h"

#define USBx USB_OTG_FS
class USB {
 public:
    // limited to 64 bytes right now
    void send_data32(uint8_t endpoint, uint32_t *data, uint8_t length32) 
    {
        //USBx_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;

        if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // TODO it would be nice to timeout/cancel old transmissions, instead just don't allow new transmissions
            
            // USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
            // while(!(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_INEPNE));    //wait on nak
            // USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 1 << (USB_OTG_GRSTCTL_TXFNUM_Pos + endpoint)));
            //  USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
            // while(USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);     // wait on flush
            // while(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA);

           return;
        }

// __disable_irq();
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);


        USBx_INEP(endpoint)->DIEPTSIZ = 0;  // TODO necessary?
        USBx_INEP(endpoint)->DIEPTSIZ = sizeof(uint32_t) * length32 | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
        USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK ;
        for(int i=0; i<length32; i++) {
            USBx_DFIFO(endpoint) = data[i]; 
        }
     //   USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;  
    //     __enable_irq();
             HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

    }

    void send_data(uint8_t endpoint, uint8_t *data, uint8_t length) {
        send_data32(endpoint, (uint32_t *) data, (length+3)/4);
    }
};

#endif
