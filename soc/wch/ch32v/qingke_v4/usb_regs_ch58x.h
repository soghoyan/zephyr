#ifndef __CH583USBSFR_H__
#define __CH583USBSFR_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* usb addresses         
//      USB:     +8000H - 83FFH                                                    */
#define USB_BASE_ADDR              (0x40008000)
#define USB2_BASE_ADDR             (0x40008400)
#define BA_USB              ((PUINT8V)0x40008000)     // point USB base address
#define BA_USB2             ((PUINT8V)0x40008400)     // point USB2 base address

/*       USB  */
#define R32_USB_CONTROL     0x0000    // USB control & interrupt enable & device address
#define R8_USB_CTRL         0x0000    // USB base control
#define  RB_UC_HOST_MODE    0x80      // enable USB host mode: 0=device mode, 1=host mode
#define  RB_UC_LOW_SPEED    0x40      // enable USB low speed: 0=12Mbps, 1=1.5Mbps
#define  RB_UC_DEV_PU_EN    0x20      // USB device enable and internal pullup resistance enable
#define  RB_UC_SYS_CTRL1    0x20      // USB system control high bit
#define  RB_UC_SYS_CTRL0    0x10      // USB system control low bit
#define  MASK_UC_SYS_CTRL   0x30      // bit mask of USB system control
// bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
//   0 00: disable USB device and disable internal pullup resistance
//   0 01: enable USB device and disable internal pullup resistance, need RB_PIN_USB_DP_PU=1 or need external pullup resistance
//   0 1x: enable USB device and enable internal pullup resistance
//   1 00: enable USB host and normal status
//   1 01: enable USB host and force UDP/UDM output SE0 state
//   1 10: enable USB host and force UDP/UDM output J state
//   1 11: enable USB host and force UDP/UDM output resume or K state
#define  RB_UC_INT_BUSY     0x08      // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define  RB_UC_RESET_SIE    0x04      // force reset USB SIE, need software clear
#define  RB_UC_CLR_ALL      0x02      // force clear FIFO and count of USB
#define  RB_UC_DMA_EN       0x01      // DMA enable and DMA interrupt enable for USB

#define R8_UDEV_CTRL        0x0001    // USB device physical prot control
#define  RB_UD_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UD_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UD_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UD_LOW_SPEED    0x04      // enable USB physical port low speed: 0=full speed, 1=low speed
#define  RB_UD_GP_BIT       0x02      // general purpose bit
#define  RB_UD_PORT_EN      0x01      // enable USB physical port I/O: 0=disable, 1=enable

#define R8_UHOST_CTRL       R8_UDEV_CTRL              // USB host physical prot control
#define  RB_UH_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UH_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UH_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UH_LOW_SPEED    0x04      // enable USB port low speed: 0=full speed, 1=low speed
#define  RB_UH_BUS_RESET    0x02      // control USB bus reset: 0=normal, 1=force bus reset
#define  RB_UH_PORT_EN      0x01      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached

#define R8_USB_INT_EN       0x0002    // USB interrupt enable
#define  RB_UIE_DEV_NAK     0x40      // enable interrupt for NAK responded for USB device mode
#define  RB_UIE_FIFO_OV     0x10      // enable interrupt for FIFO overflow
#define  RB_UIE_HST_SOF     0x08      // enable interrupt for host SOF timer action for USB host mode
#define  RB_UIE_SUSPEND     0x04      // enable interrupt for USB suspend or resume event
#define  RB_UIE_TRANSFER    0x02      // enable interrupt for USB transfer completion
#define  RB_UIE_DETECT      0x01      // enable interrupt for USB device detected event for USB host mode
#define  RB_UIE_BUS_RST     0x01      // enable interrupt for USB bus reset event for USB device mode

#define R8_USB_DEV_AD       0x0003    // USB device address
#define  RB_UDA_GP_BIT      0x80      // general purpose bit
#define  MASK_USB_ADDR      0x7F      // bit mask for USB device address

#define R32_USB_STATUS      0x0004    // USB miscellaneous status & interrupt flag & interrupt status
#define R8_USB_MIS_ST       0x0005    // USB miscellaneous status
#define  RB_UMS_SOF_PRES    0x80      // RO, indicate host SOF timer presage status
#define  RB_UMS_SOF_ACT     0x40      // RO, indicate host SOF timer action status for USB host
#define  RB_UMS_SIE_FREE    0x20      // RO, indicate USB SIE free status
#define  RB_UMS_R_FIFO_RDY  0x10      // RO, indicate USB receiving FIFO ready status (not empty)
#define  RB_UMS_BUS_RESET   0x08      // RO, indicate USB bus reset status
#define  RB_UMS_SUSPEND     0x04      // RO, indicate USB suspend status
#define  RB_UMS_DM_LEVEL    0x02      // RO, indicate UDM level saved at device attached to USB host
#define  RB_UMS_DEV_ATTACH  0x01      // RO, indicate device attached status on USB host

#define R8_USB_INT_FG       0x0006  // USB interrupt flag
#define  RB_U_IS_NAK        0x80    // RO, indicate current USB transfer is NAK received
#define  RB_U_TOG_OK        0x40    // RO, indicate current USB transfer toggle is OK
#define  RB_U_SIE_FREE      0x20    // RO, indicate USB SIE free status
#define  RB_UIF_FIFO_OV     0x10    // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define  RB_UIF_HST_SOF     0x08    // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define  RB_UIF_SUSPEND     0x04    // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_TRANSFER    0x02    // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_DETECT      0x01    // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define  RB_UIF_BUS_RST     0x01    // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

#define R8_USB_INT_ST       0x0007  // USB interrupt status
#define  RB_UIS_SETUP_ACT   0x80    // RO, indicate SETUP token & 8 bytes setup request received for USB device mode
#define  RB_UIS_TOG_OK      0x40    // RO, indicate current USB transfer toggle is OK
#define  RB_UIS_TOKEN1      0x20    // RO, current token PID code bit 1 received for USB device mode
#define  RB_UIS_TOKEN0      0x10    // RO, current token PID code bit 0 received for USB device mode
#define  MASK_UIS_TOKEN     0x30    // RO, bit mask of current token PID code received for USB device mode
#define  UIS_TOKEN_OUT      0x00
#define  UIS_TOKEN_SOF      0x10
#define  UIS_TOKEN_IN       0x20
#define  UIS_TOKEN_SETUP    0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear RB_UIF_TRANSFER ( RB_UIF_TRANSFER from 1 to 0 ) to set free
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: free
#define  MASK_UIS_ENDP      0x0F      // RO, bit mask of current transfer endpoint number for USB device mode
#define  MASK_UIS_H_RES     0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received

#define R8_USB_RX_LEN       0x0008    // USB receiving length
#define  RB_UEP_RX_EN       0x02      // Enable receive (OUT) USB endpoint
#define  RB_UEP_TX_EN       0x01      // Enable transmit (IN) USB endpoint
#define R32_USB_BUF_MODE    0x000C    // USB endpoint buffer mode
#define R8_UEP4_1_MOD       0x000C    // endpoint 4/1 mode
#define  RB_UEP1_RX_EN      0x80      // enable USB endpoint 1 receiving (OUT)
#define  RB_UEP1_TX_EN      0x40      // enable USB endpoint 1 transmittal (IN)
#define  RB_UEP1_BUF_MOD    0x10      // buffer mode of USB endpoint 1
// bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define  RB_UEP4_RX_EN      0x08      // enable USB endpoint 4 receiving (OUT)
#define  RB_UEP4_TX_EN      0x04      // enable USB endpoint 4 transmittal (IN)
// bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes
#define  RB_UEP1_EN_SHIFT   6         // Bit position for RX and TX enable field of EP1
#define  RB_UEP4_EN_SHIFT   2         // Bit position for RX and TX enable field of EP4

#define R8_UEP2_3_MOD       0x000D    // endpoint 2/3 mode
#define  RB_UEP3_RX_EN      0x80      // enable USB endpoint 3 receiving (OUT)
#define  RB_UEP3_TX_EN      0x40      // enable USB endpoint 3 transmittal (IN)
#define  RB_UEP3_BUF_MOD    0x10      // buffer mode of USB endpoint 3
#define  RB_UEP2_RX_EN      0x08      // enable USB endpoint 2 receiving (OUT)
#define  RB_UEP2_TX_EN      0x04      // enable USB endpoint 2 transmittal (IN)
#define  RB_UEP2_BUF_MOD    0x01      // buffer mode of USB endpoint 2
#define  RB_UEP3_EN_SHIFT   6         // Bit position for RX and TX enable field of EP3
#define  RB_UEP2_EN_SHIFT   2         // Bit position for RX and TX enable field of EP2

#define R8_UEP567_MOD       0x000E    // endpoint 5/6/7 mode
#define  RB_UEP7_RX_EN      0x20      // enable USB endpoint 7 receiving (OUT)
#define  RB_UEP7_TX_EN      0x10      // enable USB endpoint 7 transmittal (IN)
#define  RB_UEP6_RX_EN      0x08      // enable USB endpoint 6 receiving (OUT)
#define  RB_UEP6_TX_EN      0x04      // enable USB endpoint 6 transmittal (IN)
#define  RB_UEP5_RX_EN      0x02      // enable USB endpoint 5 receiving (OUT)
#define  RB_UEP5_TX_EN      0x01      // enable USB endpoint 5 transmittal (IN)
// bUEPn_RX_EN & bUEPn_TX_EN: USB endpoint 5/6/7 buffer mode, buffer start address is UEPn_DMA
//   0 0:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (OUT endpoint)
//   0 1:  64 bytes buffer for transmittal (IN endpoint)
//   1 1:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
#define  RB_UEP7_EN_SHIFT   4         // Bit position for RX and TX enable field of EP7
#define  RB_UEP6_EN_SHIFT   2         // Bit position for RX and TX enable field of EP6
#define  RB_UEP5_EN_SHIFT   0         // Bit position for RX and TX enable field of EP5

#define R8_UH_EP_MOD        R8_UEP2_3_MOD             //host endpoint mode
#define  RB_UH_EP_TX_EN     0x40      // enable USB host OUT endpoint transmittal
#define  RB_UH_EP_TBUF_MOD  0x10      // buffer mode of USB host OUT endpoint
// bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for transmittal (OUT endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
#define  RB_UH_EP_RX_EN     0x08      // enable USB host IN endpoint receiving
#define  RB_UH_EP_RBUF_MOD  0x01      // buffer mode of USB host IN endpoint
// bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (IN endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes

#define R16_UEP0_DMA        0x0010    // endpoint 0 DMA buffer address
#define R16_UEP1_DMA        0x0014    // endpoint 1 DMA buffer address
#define R16_UEP2_DMA        0x0018    // endpoint 2 DMA buffer address
#define R16_UH_RX_DMA       R16_UEP2_DMA              // host rx endpoint buffer address
#define R16_UEP3_DMA        0x001C    // endpoint 3 DMA buffer address
#define R16_UH_TX_DMA       R16_UEP3_DMA              // host tx endpoint buffer address
#define R16_UEP5_DMA        0x0054    // endpoint 5 DMA buffer address
#define R16_UEP6_DMA        0x0058    // endpoint 6 DMA buffer address
#define R16_UEP7_DMA        0x005C    // endpoint 7 DMA buffer address
#define R32_USB_EP0_CTRL    0x0020    // endpoint 0 control & transmittal length
#define R8_UEP0_T_LEN       0x0020    // endpoint 0 transmittal length
#define R8_UEP0_CTRL        0x0022    // endpoint 0 control
#define R32_USB_EP1_CTRL    0x0024    // endpoint 1 control & transmittal length
#define R8_UEP1_T_LEN       0x0024    // endpoint 1 transmittal length
#define R8_UEP1_CTRL        0x0026    // endpoint 1 control
#define  RB_UEP_R_TOG       0x80      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define  RB_UEP_T_TOG       0x40      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define  RB_UEP_AUTO_TOG    0x10      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define  RB_UEP_R_RES1      0x08      // handshake response type high bit for USB endpoint X receiving (OUT)
#define  RB_UEP_R_RES0      0x04      // handshake response type low bit for USB endpoint X receiving (OUT)
#define  MASK_UEP_R_RES     0x0C      // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define  UEP_R_RES_ACK      0x00
#define  UEP_R_RES_TOUT     0x04
#define  UEP_R_RES_NAK      0x08
#define  UEP_R_RES_STALL    0x0C
// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
#define  RB_UEP_T_RES1      0x02      // handshake response type high bit for USB endpoint X transmittal (IN)
#define  RB_UEP_T_RES0      0x01      // handshake response type low bit for USB endpoint X transmittal (IN)
#define  MASK_UEP_T_RES     0x03      // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define  UEP_T_RES_ACK      0x00
#define  UEP_T_RES_TOUT     0x01
#define  UEP_T_RES_NAK      0x02
#define  UEP_T_RES_STALL    0x03
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)

#define R8_UH_SETUP         R8_UEP1_CTRL              // host aux setup
#define  RB_UH_PRE_PID_EN   0x80      // USB host PRE PID enable for low speed device via hub
#define  RB_UH_SOF_EN       0x40      // USB host automatic SOF enable

#define R32_USB_EP2_CTRL    0x0028    // endpoint 2 control & transmittal length
#define R8_UEP2_T_LEN       0x0028    // endpoint 2 transmittal length
#define R8_UEP2_CTRL        0x002A    // endpoint 2 control

#define R8_UH_EP_PID        R8_UEP2_T_LEN             // host endpoint and PID
#define  MASK_UH_TOKEN      0xF0      // bit mask of token PID for USB host transfer
#define  MASK_UH_ENDP       0x0F      // bit mask of endpoint number for USB host transfer

#define R8_UH_RX_CTRL       R8_UEP2_CTRL              // host receiver endpoint control
#define  RB_UH_R_TOG        0x80      // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define  RB_UH_R_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_R_RES        0x04      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions

#define R32_USB_EP3_CTRL    0x002c    // endpoint 3 control & transmittal length
#define R8_UEP3_T_LEN       0x002c    // endpoint 3 transmittal length
#define R8_UEP3_CTRL        0x002e    // endpoint 3 control
#define R8_UH_TX_LEN        R8_UEP3_T_LEN             // host transmittal endpoint transmittal length

#define R8_UH_TX_CTRL       R8_UEP3_CTRL              // host transmittal endpoint control
#define  RB_UH_T_TOG        0x40      // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define  RB_UH_T_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_T_RES        0x01      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions

#define R32_USB_EP4_CTRL    0x0030    // endpoint 4 control & transmittal length
#define R8_UEP4_T_LEN       0x0030    // endpoint 4 transmittal length
#define R8_UEP4_CTRL        0x0032    // endpoint 4 control

#define R32_USB_EP5_CTRL    0x0064    // endpoint 5 control & transmittal length
#define R8_UEP5_T_LEN       0x0064    // endpoint 5 transmittal length
#define R8_UEP5_CTRL        0x0066    // endpoint 5 control

#define R32_USB_EP6_CTRL    0x0068    // endpoint 6 control & transmittal length
#define R8_UEP6_T_LEN       0x0068    // endpoint 6 transmittal length
#define R8_UEP6_CTRL        0x006A    // endpoint 6 control

#define R32_USB_EP7_CTRL    0x006C    // endpoint 7 control & transmittal length
#define R8_UEP7_T_LEN       0x006C    // endpoint 7 transmittal length
#define R8_UEP7_CTRL        0x006E    // endpoint 7 control


#ifdef __cplusplus
}
#endif

#endif //__CH583USBSFR_H__
