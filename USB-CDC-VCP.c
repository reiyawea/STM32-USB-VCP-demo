/* USB-CDC-VCP - Simplified Virtual COM driver for STM32.
 * Copyright (C) 2023 reiyawea
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "USB-CDC-VCP.h"
/* Private typedef -----------------------------------------------------------*/
typedef __PACKED_STRUCT{
  uint8_t  bmRequest;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} SetupPacket_TypeDef;

typedef __PACKED_STRUCT{
  uint32_t dwDTERate;  //Data terminal rate, in bits per second.
  uint8_t bCharFormat; //Stop bits, 0 - 1 Stop bit, 1 - 1.5 Stop bits, 2 - 2 Stop bits
  uint8_t bParityType; //Parity, 0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
  uint8_t bDataBits;   //Data bits (5, 6, 7, 8 or 16)
} LineCoding_TypeDef;
/* Private define ------------------------------------------------------------*/
#define USB_REQ_CLEAR_FEATURE               0x01
#define USB_REQ_SET_ADDRESS                 0x05
#define USB_REQ_GET_DESCRIPTOR              0x06
#define USB_REQ_SET_CONFIGURATION           0x09

#define USB_DEVICE_DESCRIPTOR_TYPE          0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE   0x02
#define USB_STRING_DESCRIPTOR_TYPE          0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE       0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE        0x05
#define USB_DEVICE_QUALIFIER_DESCIPTOR_TYPE 0x06

#define USB_REQ_SET_LINE_CODING             0x20
#define USB_REQ_GET_LINE_CODING             0x21
#define USB_REQ_SET_CONTROL_LINE_STATE      0x22

#define USB_EPnR ((uint32_t *)USB_BASE)

#define EP_NUM      3 // EP number used
#define BTABLE_BASE 0x0000
#define BTABLE_SIZE (EP_NUM * 8)
#define EP0_RX_SIZE 64
#define EP0_TX_SIZE 64
#define EP1_RX_SIZE 160 // must be multiple of 32 bytes
#define EP1_TX_SIZE 160
#define EP2_RX_SIZE 8
#define EP2_TX_SIZE 8

#define USB_RX0_BASE ( BTABLE_BASE + BTABLE_SIZE)
#define USB_TX0_BASE (USB_RX0_BASE + EP0_RX_SIZE)
#define USB_RX1_BASE (USB_TX0_BASE + EP0_TX_SIZE)
#define USB_TX1_BASE (USB_RX1_BASE + EP1_RX_SIZE)
#define USB_RX2_BASE (USB_TX1_BASE + EP1_TX_SIZE)
#define USB_TX2_BASE (USB_RX2_BASE + EP2_RX_SIZE)
/* Private macro -------------------------------------------------------------*/
#define SET_USB_DP_PU(x) *(volatile uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(GPIOA->ODR) - PERIPH_BASE)*32 + 15*4) = x
/* Private variables ---------------------------------------------------------*/
const uint8_t Device_Descriptor[18] = {
  0x12, //Descriptor lenght = 18 byte.
  0x01, //Descriptor type = Device.
  0x00, //Desctiptor type USB 2.0.
  0x02, //Desctiptor type USB 2.0.
  0x02, //Class Communications and CDC Control.
  0x02, //Subclass Abstract Control Model.
  0x00, //Protocol No class specific protocol required.
  0x40, //EP0 Max Packet Size 64 byte.
  0x83, //Vendor ID 0483.
  0x04, //Vendor ID 0483.
  0x40, //Product ID 5740.
  0x57, //Product ID 5740.
  0x00, //bcdDevice 200.
  0x02, //bcdDevice 200.
  0x01, //iManufacter.
  0x02, //iProduct.
  0x00, //iSerialNumber.
  0x01, //Number of possible configurations = 1.
};

const uint8_t Configuration_Descriptor[] = {        //Little Endian.
  0x09, //Descriptor lenght = 9 byte.
  0x02, //Descriptor Type = Configuration.
  0x43, //Total lenght of this configuration descriptor (included Interface_Descriptor end Endpoint_Descriptor) = 67.
  0x00, //Total lenght of this configuration descriptor (included Interface_Descriptor end Endpoint_Descriptor) = 67.
  0x02, //Number of interfaces that belong to this configuration = 2.
  0x01, //Index of this configuration = 1.
  0x00, //iConfiguration.
  0xC0, //Self/Powered.
  0x32, //Maximum current consumption = 100 mA.
  /////////////Interface_Descriptor///////////
  0x09, //Descriptor lenght = 9 byte.
  0x04, //Descriptor Type = Interface.
  0x00, //Interface number = 0.
  0x00, //Alternative interface = 0.
  0x01, //Endopoints used by this interface = 1.
  0x02, //Class Communications and CDC Control.
  0x02, //Subclass Abstract Control Model.
  0x01, //Protocol AT Commands: V.250 etc.
  0x00, //iInterface.
  /////////////Class/Specific_Descriptor///////////
  0x05, //Descriptor lenght = 5 byte.
  0x24, //Descriptor Type = "CS_INTERFACE".
  0x00, //Descriptor Subtype = "Header".
  0x10, //USB 1.1.
  0x01, //USB 1.1.
  /////////////Class/Specific_Descriptor///////////
  0x05, //Descriptor lenght = 5 byte.
  0x24, //Descriptor Type = "CS_INTERFACE".
  0x01, //Descriptor SubType = "Call Management Functional Descriptor".
  0x00, //bmCapabilities. 
  0x01, //Indicates that multiplexed commands are handled via data interface 01.
  /////////////Class/Specific_Descriptor///////////
  0x04, //Descriptor lenght = 5 byte.
  0x24, //Descriptor Type = "CS_INTERFACE".
  0x02, //Descriptor SubType = "Abstract Control Management functional descriptor".
  0x02, //bmCapabilities.
  /////////////Class/Specific_Descriptor///////////
  0x05, //Descriptor lenght = 5 byte.
  0x24, //Descriptor Type = "CS_INTERFACE".
  0x06, //Descriptor SubType = "Union Descriptor Functional Descriptor".
  0x00, //bControlInterface. Interface number of the control.
  0x01, //bSubordinateInterface0. Interface number of the subordinate (Data Class) interface.
  /////////////Endpoint_Descriptor///////////
  0x07, //Descriptor lenght = 7 byte.
  0x05, //Descriptor Type = "Endpoint".
  0x82, //In endpoint. Endpoint 2.
  0x03, //Transfer Type = "Interrupt".
  EP2_TX_SIZE%256, //Endpoint size = 8 byte.
  EP2_TX_SIZE/256, //Endpoint size = 8 byte.
  0x10, //Interval for polling endpoint = 16 * 1ms.
  /////////////Interface_Descriptor///////////
  0x09, //Descriptor lenght = 9 byte.
  0x04, //Descriptor Type = Interface.
  0x01, //Interface number = 1.
  0x00, //Alternative interface = 0.
  0x02, //Endopoints used by this interface = 2.
  0x0A, //Class "Data Interface".
  0x00, //Subclass.
  0x00, //Protocol "Non specified".
  0x00, //iInterface.
  /////////////Endpoint_Descriptor///////////
  0x07, //Descriptor lenght = 7 byte.
  0x05, //Descriptor Type = "Endpoint".
  0x01, //OUT endpoint. Endpoint 1.
  0x02, //Transfer Type = "Bulk".
  EP1_RX_SIZE%256,   //Endpoint size = 64 byte.
  EP1_RX_SIZE/256,   //Endpoint size = 64 byte.
  0x00, //Interval for polling endpoint.
  /////////////Endpoint_Descriptor///////////
  0x07, //Descriptor lenght = 7 byte.
  0x05, //Descriptor Type = "Endpoint".
  0x81, //In endpoint. Endpoint 1.
  0x02, //Transfer Type = "Bulk".
  EP1_TX_SIZE%256,   //Endpoint size = 64 byte.
  EP1_TX_SIZE/256,   //Endpoint size = 64 byte.
  0x00, //Interval for polling endpoint.
};

const uint16_t String_Descriptor_0[] = {0x0306, 0x0804, 0x0409 };
const uint16_t String_Descriptor_1_cn[] = {0x030e, 20154, 20043, 21021, 24615, 26412, 21892};
const uint16_t String_Descriptor_2_cn[] = {0x030e, 24615, 30456, 36817, 20064, 30456, 36828};
const uint16_t String_Descriptor_1_en[] = {0x0310, 83, 84, 77, 105, 99, 114, 111};
const uint16_t String_Descriptor_2_en[] = {0x031c, 83, 84, 77, 51, 50, 70, 49, 48, 51, 67, 56, 84, 54};
const uint16_t String_Descriptor_default[] = {0x030A, 25105, 19981, 30693, 36947};

LineCoding_TypeDef Line_Coding;

/* control_line_state
D15..D2 RESERVED (Reset to zero)
D1 Carrier control for half duplex modems. This signal corresponds to V.24 signal 105 and RS-232 signal RTS.
0 - Deactivate carrier
1 - Activate carrierThe device ignores the value of this bit when operating in full duplex mode.
D0 Indicates to DCE if DTE is present or not. This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR.
0 - Not Present
1 - Present*/
uint16_t control_line_state;

uint8_t ep0_buffer[64]; //memory for control tranfers on endpoint 0
static const uint8_t* ep0_tx_ptr; //pointer to what to be sent on next IN token.
static uint16_t ep0_tx_len; //remaining length to be sent through ep0
static uint16_t ep1_rx_sz; //length of data received. updated when EP1 receives a packet.
static uint16_t ep1_tx_sz; //length of data to be sent. cleared when EP1 has sent a packet.

/* Private function prototypes -----------------------------------------------*/
void EN_RX_EP(uint8_t ep);
void Write_EP(uint8_t ep, const void *data , uint16_t len);
uint16_t Read_EP(uint8_t ep, void *data);
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Set endpoint OUT to VALID.
  * @param  ep: endpoint number
  * @retval None
  */
static void EN_RX_EP(uint8_t ep)
{
  uint32_t EPnR;

  EPnR = USB_EPnR[ep];
  EPnR &= USB_EP0R_STAT_RX | USB_EP0R_CTR_RX | USB_EP0R_EP_TYPE | USB_EP0R_CTR_TX | USB_EP0R_EA;
  EPnR ^= USB_EP0R_STAT_RX;
  USB_EPnR[ep] = EPnR;
}

/**
  * @brief  Copy data into Packet Memory and set endpoint IN to VALID.
  * @param  ep: endpoint number
  * @param  data: pointer to source data
  * @param  len: length of data in bytes
  * @retval None
  */
static void Write_EP(uint8_t ep, const void *data, uint16_t len)
{
  uint32_t EPnR, i;
  uint32_t *p_u32 = (uint32_t *)(USB_PMAADDR + *(uint32_t*)(USB_PMAADDR + ep*16) * 2);
  uint16_t *p_u16 = (uint16_t *)data;
  for(i = 0; i < len; i += 2) {
    *p_u32++ = *p_u16++;
  }
  *(volatile uint32_t *)(USB_PMAADDR + ep*16 + 4) = len;

  EPnR = USB_EPnR[ep];
  EPnR &= USB_EP0R_STAT_TX | USB_EP0R_CTR_RX | USB_EP0R_EP_TYPE | USB_EP0R_CTR_TX | USB_EP0R_EA;
  EPnR ^= USB_EP0R_STAT_TX; // set endpoint IN direction to VALID
  USB_EPnR[ep] = EPnR;
}

/**
  * @brief  Copy data from Packet Memory (endpoint OUT remain NAK).
  * @param  ep: endpoint number
  * @param  data: pointer to output buffer. may be NULL if no data copy is needed.
  * @param  len: 
  * @retval length of data in Packet Memory, in bytes
  */
static uint16_t Read_EP(uint8_t ep, void *data)
{
  uint32_t *p_u32 = (uint32_t *)(USB_PMAADDR + *(uint32_t*)(USB_PMAADDR + ep*16 + 8) * 2);
  uint16_t *p_u16;
  uint16_t len = (*(uint32_t *)(USB_PMAADDR + ep*16 + 12)) & 0x3ff;
  
  if(data){
    p_u16 = (uint16_t *)data;
    for (int i = 0; i < len; i += 2) {
      *p_u16++ = *p_u32++;
    }
  }

  return len;
}

/**
  * @brief  Reset USB registers and allocate Packet Memory to endpoints.
  * @param  None
  * @retval None
  */
void USB_VCP_init()
{
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 0UL * 8 + 0) * 2) = USB_TX0_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 0UL * 8 + 4) * 2) = USB_RX0_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 0UL * 8 + 6) * 2) = USB_COUNT0_RX_BLSIZE | ((EP0_RX_SIZE / 32 - 1)<<10);
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 1UL * 8 + 0) * 2) = USB_TX1_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 1UL * 8 + 4) * 2) = USB_RX1_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 1UL * 8 + 6) * 2) = USB_COUNT1_RX_BLSIZE | ((EP1_RX_SIZE / 32 - 1)<<10);
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 2UL * 8 + 0) * 2) = USB_TX2_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 2UL * 8 + 4) * 2) = USB_RX2_BASE;
  *(volatile uint32_t*)(USB_PMAADDR + (BTABLE_BASE + 2UL * 8 + 6) * 2) = (EP2_RX_SIZE / 2)<<10; // endpoint 2 rx buffer size = 8 bytes
  USB->CNTR = USB_CNTR_FRES;
  for(int wait = 0; wait < 256; wait++);
  USB->CNTR = 0x0000;
  USB->ISTR = 0x0000;
  USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
  USB->BTABLE = BTABLE_BASE;
  SET_USB_DP_PU(1);
}

/**
  * @brief  return number of bytes available from host.
  * @param  None
  * @retval number of bytes received in EP1 packet memory.
  */
uint16_t USB_VCP_available()
{
  return ep1_rx_sz;
}

/**
  * @brief  transfer data from Packet Memory to user buffer and set EP1 RX status to VALID.
  * @param  data: pointer to data buffer, must be large enough to hold the data.
  * @retval number of bytes written to the buffer
  */
uint16_t USB_VCP_readBytes(void *data)
{
  uint16_t len;
  
  len = Read_EP(1, data);
  ep1_rx_sz = 0;
  EN_RX_EP(1); //Enable the endpoint 1 reception buffer to receive any new packet.

  return len;
}

/**
  * @brief  transfer data to Packet Memory and set EP1 TX status to VALID.
  * @param  data: pointer to source data.
  * @param  len: length of data to be sent. must not exceed buffer size.
  * @retval status. length of data transferred to Packet Memory, or -1 if the previous packet is not finished.
  */
int USB_VCP_readBytes(void *data, uint16_t len)
{
  if (ep1_tx_sz){
    return -1;
  }
  ep1_tx_sz = len;
  if (ep1_tx_sz > EP1_TX_SIZE){
    ep1_tx_sz = EP1_TX_SIZE;
  }
  Write_EP(1, data, ep1_tx_sz);

  return ep1_tx_sz;
}

/**
  * @brief  USB interrupt service routine
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler()
{
  static uint8_t set_line_coding , sla_addr;
  SetupPacket_TypeDef *p = (SetupPacket_TypeDef *)ep0_buffer;
  uint16_t len, tx_sz;

  if (USB->ISTR & USB_ISTR_RESET) {
    USB->ISTR = ~USB_ISTR_RESET;
    USB->EP0R = USB_EP0R_STAT_RX | USB_EP0R_STAT_TX_1 | 0 | USB_EP0R_EP_TYPE_0; //set endpoint 0 type = CONTROL, OUT VALID, IN NAK.
    USB->EP1R = USB_EP1R_STAT_RX | USB_EP1R_STAT_TX_1 | 1;                      //set endpoint 1 type = BULK, OUT VALID, IN NAK.
    USB->EP2R = USB_EP2R_STAT_RX | USB_EP2R_STAT_TX_1 | 2 |USB_EP2R_EP_TYPE;   //set endpoint 2 type = INTERRUPT, OUT VALID, IN NAK.
    USB->DADDR = USB_DADDR_EF;
  }
  if(USB->ISTR & USB_ISTR_CTR) {
    if (USB->EP0R & USB_EP0R_CTR_RX) { // a packet has been received on endpoint 0
      if (USB->EP0R & USB_EP0R_SETUP) { // SETUP packet
        len = Read_EP(0, ep0_buffer);
        if (p->bmRequest == 0x80 && p->bRequest == USB_REQ_GET_DESCRIPTOR) {
          switch(p->wValue >> 8){
            case USB_DEVICE_DESCRIPTOR_TYPE:
              Write_EP(0, Device_Descriptor , sizeof Device_Descriptor);
              break;
            case USB_CONFIGURATION_DESCRIPTOR_TYPE:
              ep0_tx_ptr = Configuration_Descriptor;
              ep0_tx_len = sizeof Configuration_Descriptor;
              if (ep0_tx_len > p->wLength) {
                ep0_tx_len = p->wLength;
              }
              tx_sz = ep0_tx_len;
              if (tx_sz > 64) {
                tx_sz = 64;
              }
              Write_EP(0, ep0_tx_ptr , tx_sz);
              ep0_tx_len -= tx_sz;
              ep0_tx_ptr += tx_sz;
              break;
            case USB_DEVICE_QUALIFIER_DESCIPTOR_TYPE:
              uint32_t EPnR = USB->EP0R & (USB_EP0R_STAT_RX | USB_EP0R_CTR_RX | USB_EP0R_EP_TYPE | USB_EP0R_CTR_TX | USB_EP0R_EA);
              EPnR ^= USB_EP0R_STAT_TX_0; //set to STALL since not supported by USB1.1
              USB->EP0R = EPnR;
              break;
            case USB_STRING_DESCRIPTOR_TYPE:
              switch(p->wValue & 0xff){
                case 0:
                  Write_EP(0, String_Descriptor_0, sizeof String_Descriptor_0);
                  break;
                case 1:
                  if(p->wIndex == 0x0804){
                    Write_EP(0, String_Descriptor_1_cn, sizeof String_Descriptor_1_cn);
                  }else{
                    Write_EP(0, String_Descriptor_1_en, sizeof String_Descriptor_1_en);
                  };
                  break;
                case 2: 
                  if(p->wIndex == 0x0804){
                    Write_EP(0, String_Descriptor_2_cn, sizeof String_Descriptor_2_cn);
                  }else{
                    Write_EP(0, String_Descriptor_2_en, sizeof String_Descriptor_2_en);
                  };
                  break;
                default:
                  Write_EP(0, String_Descriptor_default, sizeof String_Descriptor_default);
              }
              break;
            default:
              Write_EP(0, NULL, 0);
          }
        }else if (p->bmRequest == 0x00 && p->bRequest == USB_REQ_SET_ADDRESS) {
          Write_EP(0, NULL, 0);
          sla_addr = (p->wValue & 0x7F); // save address and write to register after ACK packet is sent.
        }else if (p->bmRequest == 0x00 && p->bRequest == USB_REQ_SET_CONFIGURATION) {
          Write_EP(0, 0 , 0);
        }else if (p->bmRequest == 0xA1 && p->bRequest == USB_REQ_GET_LINE_CODING) {
          Write_EP(0, &Line_Coding , sizeof Line_Coding);
        }else if (p->bmRequest == 0x21 && p->bRequest == USB_REQ_SET_CONTROL_LINE_STATE) {
          control_line_state = p->wValue;
          Write_EP(0, 0, 0);
        }else if (p->bmRequest == 0x21 && p->bRequest == USB_REQ_SET_LINE_CODING) {
          set_line_coding = 1; //line coding data is in the next OUT packet. save status.
        }else if (p->bmRequest == 0x02 && p->bRequest == USB_REQ_CLEAR_FEATURE) {
          Write_EP(0, 0, 0);
        }else{
          Write_EP(0, 0, 0);
        }
      } else { // OUT packet
        len = Read_EP(0, ep0_buffer);
        if (set_line_coding) {
          memcpy(&Line_Coding , ep0_buffer , sizeof Line_Coding);
          Write_EP(0, NULL, 0);
          set_line_coding = 0;
        }
      }
      USB->EP0R = USB_EP0R_CTR_TX | USB_EP0R_EP_TYPE_0 | 0x0;
      EN_RX_EP(0); //Enable the endpoint 1 reception buffer to receive any new packet.
    }
    if (USB->EP0R & USB_EP0R_CTR_TX) { //a packet has been sent on endpoint 0
      if (sla_addr) {
        USB->DADDR = USB_DADDR_EF | sla_addr;
        sla_addr = 0;
      }
      if (ep0_tx_len) {
        tx_sz = ep0_tx_len;
        if (tx_sz > 64) {
          tx_sz = 64;
        }
        Write_EP(0, ep0_tx_ptr , tx_sz);
        ep0_tx_len -= tx_sz;
        ep0_tx_ptr += tx_sz;
      }
      USB->EP0R = USB_EP0R_CTR_RX | USB_EP0R_EP_TYPE_0 | 0x0;
    }
    if (USB->EP1R & USB_EP1R_CTR_RX) { //a packet has been received on endpoint 1
      ep1_rx_sz = Read_EP(1, NULL);
      USB->EP1R = USB_EP1R_CTR_TX | 0x1;
    }
    if (USB->EP1R & USB_EP1R_CTR_TX) { //a packet has been sent on endpoint 1
      ep1_tx_sz = 0;
      USB->EP1R = USB_EP1R_CTR_RX | 0x1;
    }
    if (USB->EP2R & USB_EP2R_CTR_RX) { //a packet has been received on endpoint 2
      EN_RX_EP(2);   //Enable the endpoint 1 reception buffer to receive any new packet.
      USB->EP2R = USB_EP2R_CTR_TX | USB_EP2R_EP_TYPE | 0x2;
    }
    if (USB->EP2R & USB_EP2R_CTR_TX) { //a packet has been sent on endpoint 2
      USB->EP2R = USB_EP2R_CTR_RX | USB_EP2R_EP_TYPE | 0x2;
    }
  }
}

/*****END OF FILE****/
