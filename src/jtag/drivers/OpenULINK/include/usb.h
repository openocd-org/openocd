/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef __USB_H
#define __USB_H

#include "shorttypes.h"
#include "reg_ezusb.h"

#include <stdbool.h>

#define NULL        (void*)0;

/* High and Low byte of a word (u16) */
#define HI8(word)   (u8)(((u16)word >> 8) & 0xff)
#define LO8(word)   (u8)((u16)word & 0xff)

/* Convenience functions */
#define STALL_EP0()   EP0CS |= EP0STALL
#define CLEAR_IRQ()   EXIF &= ~USBINT

/*********** USB descriptors. See section 9.5 of the USB 1.1 spec **********/

/* USB Descriptor Types. See USB 1.1 spec, page 187, table 9-5 */
#define DESCRIPTOR_TYPE_DEVICE         0x01
#define DESCRIPTOR_TYPE_CONFIGURATION  0x02
#define DESCRIPTOR_TYPE_STRING         0x03
#define DESCRIPTOR_TYPE_INTERFACE      0x04
#define DESCRIPTOR_TYPE_ENDPOINT       0x05

#define STR_DESCR(len,...) { len*2+2, DESCRIPTOR_TYPE_STRING, { __VA_ARGS__ } }

/** USB Device Descriptor. See USB 1.1 spec, pp. 196 - 198 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< DEVICE Descriptor Type.
  u16 bcdUSB;              ///< USB specification release number (BCD).
  u8  bDeviceClass;        ///< Class code.
  u8  bDeviceSubClass;     ///< Subclass code.
  u8  bDeviceProtocol;     ///< Protocol code.
  u8  bMaxPacketSize0;     ///< Maximum packet size for EP0 (8, 16, 32, 64).
  u16 idVendor;            ///< USB Vendor ID.
  u16 idProduct;           ///< USB Product ID.
  u16 bcdDevice;           ///< Device Release Number (BCD).
  u8  iManufacturer;       ///< Index of manufacturer string descriptor.
  u8  iProduct;            ///< Index of product string descriptor.
  u8  iSerialNumber;       ///< Index of string descriptor containing serial #.
  u8  bNumConfigurations;  ///< Number of possible configurations.
} usb_device_descriptor_t;

/** USB Configuration Descriptor. See USB 1.1 spec, pp. 199 - 200 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< CONFIGURATION descriptor type.
  u16 wTotalLength;        ///< Combined total length of all descriptors.
  u8  bNumInterfaces;      ///< Number of interfaces in this configuration.
  u8  bConfigurationValue; ///< Value used to select this configuration.
  u8  iConfiguration;      ///< Index of configuration string descriptor.
  u8  bmAttributes;        ///< Configuration characteristics.
  u8  MaxPower;            ///< Maximum power consumption in 2 mA units.
} usb_config_descriptor_t;

/** USB Interface Descriptor. See USB 1.1 spec, pp. 201 - 203 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< INTERFACE descriptor type.
  u8  bInterfaceNumber;    ///< Interface number.
  u8  bAlternateSetting;   ///< Value used to select alternate setting.
  u8  bNumEndpoints;       ///< Number of endpoints used by this interface.
  u8  bInterfaceClass;     ///< Class code.
  u8  bInterfaceSubclass;  ///< Subclass code.
  u8  bInterfaceProtocol;  ///< Protocol code.
  u8  iInterface;          ///< Index of interface string descriptor.
} usb_interface_descriptor_t;

/** USB Endpoint Descriptor. See USB 1.1 spec, pp. 203 - 204 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< ENDPOINT descriptor type.
  u8  bEndpointAddress;    ///< Endpoint Address: USB 1.1 spec, table 9-10.
  u8  bmAttributes;        ///< Endpoint Attributes: USB 1.1 spec, table 9-10.
  u16 wMaxPacketSize;      ///< Maximum packet size for this endpoint.
  u8  bInterval;           ///< Polling interval (in ms) for this endpoint.
} usb_endpoint_descriptor_t;

/** USB Language Descriptor. See USB 1.1 spec, pp. 204 - 205 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< STRING descriptor type.
  u16 wLANGID[];           ///< LANGID codes.
} usb_language_descriptor_t;

/** USB String Descriptor. See USB 1.1 spec, pp. 204 - 205 */
typedef struct {
  u8  bLength;             ///< Size of this descriptor in bytes.
  u8  bDescriptorType;     ///< STRING descriptor type.
  u16 bString[];           ///< UNICODE encoded string.
} usb_string_descriptor_t;

/********************** USB Control Endpoint 0 related *********************/

/** USB Control Setup Data. See USB 1.1 spec, pp. 183 - 185 */
typedef struct {
  u8  bmRequestType;       ///< Characteristics of a request.
  u8  bRequest;            ///< Specific request.
  u16 wValue;              ///< Field that varies according to request.
  u16 wIndex;              ///< Field that varies according to request.
  u16 wLength;             ///< Number of bytes to transfer in data stage.
} setup_data_t;

/* External declarations for variables that need to be accessed outside of
 * the USB module */
extern volatile bool EP2_out;
extern volatile bool EP2_in;
extern volatile __xdata __at 0x7FE8 setup_data_t setup_data;

/*
 * USB Request Types (bmRequestType): See USB 1.1 spec, page 183, table 9-2
 *
 * Bit 7: Data transfer direction
 *    0 = Host-to-device
 *    1 = Device-to-host
 * Bit 6...5: Type
 *    0 = Standard
 *    1 = Class
 *    2 = Vendor
 *    3 = Reserved
 * Bit 4...0: Recipient
 *    0 = Device
 *    1 = Interface
 *    2 = Endpoint
 *    3 = Other
 *    4...31 = Reserved
 */

#define USB_DIR_OUT             0x00
#define USB_DIR_IN              0x80

#define USB_REQ_TYPE_STANDARD   (0x00 << 5)
#define USB_REQ_TYPE_CLASS      (0x01 << 5)
#define USB_REQ_TYPE_VENDOR     (0x02 << 5)
#define USB_REQ_TYPE_RESERVED   (0x03 << 5)

#define USB_RECIP_DEVICE        0x00
#define USB_RECIP_INTERFACE     0x01
#define USB_RECIP_ENDPOINT      0x02
#define USB_RECIP_OTHER         0x03

/* bmRequestType for USB Standard Requests */

/* Clear Interface Request */
#define CF_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define CF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define CF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Get Configuration Request */
#define GC_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Descriptor Request */
#define GD_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Interface Request */
#define GI_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Get Status Request: See USB 1.1 spec, page 190 */
#define GS_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define GS_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define GS_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Address Request is handled by EZ-USB core */

/* Set Configuration Request */
#define SC_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Descriptor Request */
#define SD_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Feature Request */
#define SF_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define SF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define SF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Interface Request */
#define SI_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Synch Frame Request */
#define SY_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* USB Requests (bRequest): See USB 1.1 spec, table 9-4 on page 187 */
#define GET_STATUS               0
#define CLEAR_FEATURE            1
// Value '2' is reserved for future use
#define SET_FEATURE              3
// Value '4' is reserved for future use
#define SET_ADDRESS              5
#define GET_DESCRIPTOR           6
#define SET_DESCRIPTOR           7
#define GET_CONFIGURATION        8
#define SET_CONFIGURATION        9
#define GET_INTERFACE           10
#define SET_INTERFACE           11
#define SYNCH_FRAME             12

/* Standard Feature Selectors: See USB 1.1 spec, table 9-6 on page 188 */
#define DEVICE_REMOTE_WAKEUP     1
#define ENDPOINT_HALT            0

/************************** EZ-USB specific stuff **************************/

/** USB Interrupts. See AN2131-TRM, page 9-4 for details */
typedef enum {
  SUDAV_ISR = 13,
  SOF_ISR,
  SUTOK_ISR,
  SUSPEND_ISR,
  USBRESET_ISR,
  IBN_ISR,
  EP0IN_ISR,
  EP0OUT_ISR,
  EP1IN_ISR,
  EP1OUT_ISR,
  EP2IN_ISR,
  EP2OUT_ISR,
  EP3IN_ISR,
  EP3OUT_ISR,
  EP4IN_ISR,
  EP4OUT_ISR,
  EP5IN_ISR,
  EP5OUT_ISR,
  EP6IN_ISR,
  EP6OUT_ISR,
  EP7IN_ISR,
  EP7OUT_ISR
} USB_ISR;

/*************************** Function Prototypes ***************************/

__xdata u8* usb_get_endpoint_cs_reg(u8 ep);
void usb_reset_data_toggle(u8 ep);

bool usb_handle_get_status(void);
bool usb_handle_clear_feature(void);
bool usb_handle_set_feature(void);
bool usb_handle_get_descriptor(void);
void usb_handle_set_interface(void);

void usb_handle_setup_data(void);
void usb_init(void);

#endif
