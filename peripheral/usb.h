#ifndef USB_H
#define USB_H

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_usb.h"
#include <algorithm>

#include "usbd_cdc.h"

extern uint8_t usb_connected;

    extern uint8_t USBD_FS_DeviceDesc[0x12];
    extern uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ];

#define USBx USB_OTG_FS
class USB {
 public:
    // limited to 64 bytes right now
    void send_data32(uint8_t endpoint, const uint32_t *data, uint8_t length32, uint8_t length8=0) 
    {
        if (!(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP)) {
            return;
        }
        if (endpoint != 0) {
            if (sending_) {
                // endpoint zero has priority, I don't know why but sending on another 
                // endpoint at the same time seems to break the system
                return;
            }
        }
        // if (sending_)
        //     asm("bkpt");
        sending_ = 1;
        if (length8 == 0) {
            length8 = length32*4;
        }
        if (endpoint != 0) {
        if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {  
            int count = 0;
            do {     
                USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
                if (count++ > 100) {
                    return;
                } 
            } while(!(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_INEPNE) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_EPDISD));
           while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
           USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 1 << (USB_OTG_GRSTCTL_TXFNUM_Pos + endpoint - 1)));
           while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS));     // wait on flush
        }
        } else {
            while(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
                asm("bkpt");
            }
        }

        USBx_INEP(endpoint)->DIEPTSIZ = 0;  // TODO necessary?
        USBx_INEP(endpoint)->DIEPTSIZ = length8 | ((length8/64 + 1) << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
        USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK ;

        for(int i=0; i<length32; i++) {
            USBx_DFIFO(endpoint) = data[i]; 
        }
        // if (endpoint == 0) {
        // do {
        //     if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_NAKSTS) {
        //         USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK ;
        //         while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
        //         USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 1 << (USB_OTG_GRSTCTL_TXFNUM_Pos + endpoint - 1)));
        //         while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS));     // wait on flush
        //         sending_ = 0;
        //         return;
        //     }
        // } while(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA);
        // if (USBx_INEP(endpoint)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ)
        //     asm("bkpt");
        // }

        sending_ = 0;
    }

    void send_data(uint8_t endpoint, const uint8_t *data, uint8_t length) {
        send_data32(endpoint, (uint32_t *) data, (length+3)/4, length);
    }

    // receive up to length bytes from endpoint, return number of bytes read
    int receive_data(uint8_t endpoint, uint8_t * const data, uint8_t length) {
        return 0;
    }

    void send_stall(uint8_t endpoint) {
        USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    }

    void read_fifo(uint8_t byte_count, uint32_t *data) {
        for (int i=0; i < (byte_count+3)/4; i++) {
            data[i] = USBx_DFIFO(0);
        }
    }

    void interrupt() {
            /* Handle Reset Interrupt */
        if (USBx->GINTSTS & USB_OTG_GINTSTS_USBRST)
        {
            // flush all fifos
            USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 0x10 << 6)); 
            while (USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);
        
            for (int i = 0U; i < 4; i++)
            {
                USBx_INEP(i)->DIEPINT = 0xFFU;
                USBx_OUTEP(i)->DOEPINT = 0xFFU;
            }
            USBx_DEVICE->DAINT = 0xFFFFFFFFU;
            USBx_DEVICE->DAINTMSK |= 0x10001U;
        
            USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM);
            USBx_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
        
            /* Set Default Address to 0 */
            USBx_DEVICE->DCFG = USB_OTG_DCFG_DAD;
            device_address_ = 0;
            
            /* setup EP0 to receive SETUP packets */
            USBx_OUTEP(0U)->DOEPTSIZ = 0U;
            USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U)) ;
            USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
            USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

            USBx_INEP(2)->DIEPCTL = 0;  

            USBx->GINTSTS &= USB_OTG_GINTSTS_USBRST;
        }

        if (USBx->GINTSTS & USB_OTG_GINTSTS_USBSUSP) {
            USBx->GINTSTS &= USB_OTG_GINTSTS_USBSUSP;
        }

        if (USBx->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
        {
            USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;    
            USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

            USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
            /* hclk Clock Range between 32-180 MHz */
            USBx->GUSBCFG |= (uint32_t)((0x6U << 10U) & USB_OTG_GUSBCFG_TRDT);
        
            USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1U << (0)));
            USBx_INEP(0)->DIEPCTL |= ((64 & USB_OTG_DIEPCTL_MPSIZ ) | (0 << 18U) |\
                ((0) << 22U) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP)); 

            USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((1U << (0)) << 16U);
            USBx_OUTEP(0)->DOEPCTL |= ((64 & USB_OTG_DOEPCTL_MPSIZ ) | (0 << 18U) |\
                (USB_OTG_DIEPCTL_SD0PID_SEVNFRM)| (USB_OTG_DOEPCTL_USBAEP));
        
             USBx->GINTSTS &= USB_OTG_GINTSTS_ENUMDNE;
        }

    
        /* Handle RxQLevel Interrupt */
        if(USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
        {
            uint32_t data[64];
          //  USB_MASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
        
            uint32_t temp = USBx->GRXSTSP;
        
            uint8_t ep_number = temp & USB_OTG_GRXSTSP_EPNUM;
            uint8_t byte_count = (temp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
            uint8_t packet_status = (temp & USB_OTG_PKTSTS) >> USB_OTG_PKTSTS_Pos;
            
            if (packet_status == STS_DATA_UPDT) {
                read_fifo(byte_count, data);
            }
            if (packet_status == STS_SETUP_UPDT) {
                read_fifo(byte_count, reinterpret_cast<uint32_t *>(setup_data));
            }

        
      //  USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
        
        }

        if(USBx->GINTSTS & USB_OTG_GINTSTS_OEPINT)
        {
            uint16_t out_ep_interrupt = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) >> 16u;    // endpoints with out interrupts
            if (out_ep_interrupt & 1) { // endpoint 0 interrupt
                if (USBx_OUTEP(0)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
                    // transfer complete
                }
                if (USBx_OUTEP(0)->DOEPINT & USB_OTG_DOEPINT_STUP) {
                    // setup phase done
                    handle_setup_packet(setup_data);
                }
                USBx_OUTEP(0)->DOEPINT = 0xFFFF;
            }
        }

        if(USBx->GINTSTS & USB_OTG_GINTSTS_IEPINT)
        {
            uint16_t in_ep_interrupt = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) & 0xFFFF;    // endpoints with out interrupts
            if (in_ep_interrupt & 1) { // endpoint 0 interrupt
                if (USBx_INEP(0)->DIEPINT & USB_OTG_DOEPINT_XFRC) {
                    // transfer complete
                    if (device_address_) {
                        //asm("bkpt");
                        // USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
                        // USBx_DEVICE->DCFG |= device_address_ << USB_OTG_DCFG_DAD_Pos;
                    }
                }
                USBx_INEP(0)->DIEPINT = 0xFFFF;
            }
        }
    }



    void handle_setup_packet(uint8_t *setup_data) {
                    if (device_address_) {
              //  asm("bkpt");
            }
        switch(setup_data[0]) {
            case 0x80:  // standard request get
                switch (setup_data[1]) {
                    case 0x00:  // get status
                        send_data(0, reinterpret_cast<const uint8_t *>("\x1\x0"), 2);  // self powered
                        break;
                    case 0x06:  // get descriptor
                        switch (setup_data[3]) {
                            case 0x01:   // device descriptor
                                send_data(0, USBD_FS_DeviceDesc, std::min(static_cast<size_t>(setup_data[6]),sizeof(USBD_FS_DeviceDesc)));
                                break;
                            case 0x02:   // configuration descriptor
                                send_data(0, USBD_CDC_CfgFSDesc, std::min(static_cast<size_t>(setup_data[6]),sizeof(USBD_CDC_CfgFSDesc)));
                                break;
                            case 0x03:  // string descriptor
                                switch (setup_data[4]) {
                                    case 0x00: // language descriptor
                                        send_data(0, reinterpret_cast<const uint8_t *>("\x4\x3\x9\x4"), 4); // english
                                        break;
                                    default:
                                        send_data(0, reinterpret_cast<const uint8_t *>("\xa\x3" "b\0o\0r\0t\0o\0"), 12);
                                        break;
                                }
                                break;
                            default: 
                                send_stall(0);
                                break;
                        }
                        break;
                }
                break;
            case 0x00:  // standard request set
                switch (setup_data[1]) {
                    case 0x05:  // set address
                        device_address_ = setup_data[2];
                        USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD; 
                        USBx_DEVICE->DCFG |= device_address_ << USB_OTG_DCFG_DAD_Pos;
                        send_data(0,0,0); // core seems to know to still send this as address 0
                        break;
                    case 0x09: // set configuration
                        // enable endpoint 2
                        USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1U << (2)));
                        USBx_INEP(2)->DIEPCTL |= ((64 & USB_OTG_DIEPCTL_MPSIZ ) | (2 << 18U) |\
                            ((2) << USB_OTG_DIEPCTL_TXFNUM_Pos) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP)); 
                        send_data(0,0,0);
                        break;
                }
                break;
            default:
                send_stall(0);
                break;
        }
        USBx_OUTEP(0U)->DOEPTSIZ = 0U;
        USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U)) ;
        USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
        USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;  
        USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    }

private:
    uint8_t device_address_ = 0;
    uint8_t setup_data[64];
    int sending_=0;
};

#endif
