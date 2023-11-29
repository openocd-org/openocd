/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
	File : usb.h															*
	Contents : usb communication handling header file for NanoXplore		*
	USB-JTAG ANGIE adapter hardware.										*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef __USB_H
#define __USB_H

#include "reg_ezusb.h"
#include <stdint.h>
#include <stdbool.h>

/* High and Low byte of a word (uint16_t) */
#define HI8(word)   (uint8_t)(((uint16_t)(word) >> 8) & 0xff)
#define LO8(word)   (uint8_t)((uint16_t)(word) & 0xff)

/* Convenience functions */
#define STALL_EP0()   (EP0CS |= EPSTALL)
#define CLEAR_IRQ()   (USBINT = 0)

/*********** USB descriptors. See USB 2.0 Spec **********/

/* USB Descriptor Types. See USB 2.0 Spec */
#define DESCRIPTOR_TYPE_DEVICE                      0x01
#define DESCRIPTOR_TYPE_CONFIGURATION               0x02
#define DESCRIPTOR_TYPE_STRING                      0x03
#define DESCRIPTOR_TYPE_INTERFACE                   0x04
#define DESCRIPTOR_TYPE_ENDPOINT                    0x05
#define DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION       0x0B

#define STR_DESCR(len, ...) { (len) * 2 + 2, DESCRIPTOR_TYPE_STRING, { __VA_ARGS__ } }

/** USB Device Descriptor. See USB 2.0 Spec */
struct usb_device_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< DEVICE Descriptor Type. */
	uint16_t bcdusb;		/**< USB specification release number (BCD). */
	uint8_t bdeviceclass;		/**< Class code. */
	uint8_t bdevicesubclass;	/**< Subclass code. */
	uint8_t bdeviceprotocol;	/**< Protocol code. */
	uint8_t bmaxpacketsize0;	/**< Maximum packet size for EP0 (8, 16, 32, 64). */
	uint16_t idvendor;		/**< USB Vendor ID. */
	uint16_t idproduct;		/**< USB Product ID. */
	uint16_t bcddevice;		/**< Device Release Number (BCD). */
	uint8_t imanufacturer;		/**< Index of manufacturer string descriptor. */
	uint8_t iproduct;		/**< Index of product string descriptor. */
	uint8_t iserialnumber;		/**< Index of string descriptor containing serial #. */
	uint8_t bnumconfigurations;	/**< Number of possible configurations. */
};

/** USB Configuration Descriptor. See USB 2.0 Spec */
struct usb_config_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< CONFIGURATION descriptor type. */
	uint16_t wtotallength;		/**< Combined total length of all descriptors. */
	uint8_t bnuminterfaces;		/**< Number of interfaces in this configuration. */
	uint8_t bconfigurationvalue;	/**< Value used to select this configuration. */
	uint8_t iconfiguration;		/**< Index of configuration string descriptor. */
	uint8_t bmattributes;		/**< Configuration characteristics. */
	uint8_t maxpower;		/**< Maximum power consumption in 2 mA units. */
};

/** USB Interface association Descriptor. See USB 2.0 Spec */
struct usb_interface_association_descriptor {
	uint8_t  blength;
	uint8_t  bdescriptortype;
	uint8_t  bfirstinterface;
	uint8_t  binterfacecount;
	uint8_t  bfunctionclass;
	uint8_t  bfunctionsubclass;
	uint8_t  bfunctionprotocol;
	uint8_t  ifunction;
};

/** USB Interface Descriptor. See USB 2.0 Spec */
struct usb_interface_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< INTERFACE descriptor type. */
	uint8_t binterfacenumber;	/**< Interface number. */
	uint8_t balternatesetting;	/**< Value used to select alternate setting. */
	uint8_t bnumendpoints;		/**< Number of endpoints used by this interface. */
	uint8_t binterfaceclass;	/**< Class code. */
	uint8_t binterfacesubclass;	/**< Subclass code. */
	uint8_t binterfaceprotocol;	/**< Protocol code. */
	uint8_t iinterface;		/**< Index of interface string descriptor. */
};

/** USB Endpoint Descriptor. See USB 2.0 Spec */
struct usb_endpoint_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< ENDPOINT descriptor type. */
	uint8_t bendpointaddress;	/**< Endpoint Address: IN/OUT + EP number. */
	uint8_t bmattributes;		/**< Endpoint Attributes: BULK/INTR/ISO/CTRL. */
	uint16_t wmaxpacketsize;	/**< Maximum packet size for this endpoint. */
	uint8_t binterval;		/**< Polling interval (in ms) for this endpoint. */
};

/** USB Language Descriptor. See USB 2.0 Spec */
struct usb_language_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< STRING descriptor type. */
	uint16_t wlangid[];		/**< LANGID codes. */
};

/** USB String Descriptor. See USB 2.0 Spec */
struct usb_string_descriptor {
	uint8_t blength;		/**< Size of this descriptor in bytes. */
	uint8_t bdescriptortype;	/**< STRING descriptor type. */
	uint16_t bstring[];		/**< UNICODE encoded string. */
};

/********************** USB Control Endpoint 0 related *********************/

/** USB Control Setup Data. See USB 2.0 Spec */
struct setup_data {
	uint8_t bmrequesttype;		/**< Characteristics of a request. */
	uint8_t brequest;		/**< Specific request. */
	uint16_t wvalue;		/**< Field that varies according to request. */
	uint16_t windex;		/**< Field that varies according to request. */
	uint16_t wlength;		/**< Number of bytes to transfer in data stage. */
};

/* External declarations for variables that need to be accessed outside of
 * the USB module */
extern volatile bool ep1_out;
extern volatile bool ep1_in;

extern volatile __xdata __at 0xE6B8 struct setup_data setup_data;

/*
 * USB Request Types (bmRequestType): See USB 2.0 Spec
 *
 * Bit 7: Data transfer direction
 *	0 = Host-to-device
 *	1 = Device-to-host
 * Bit 6...5: Type
 *	0 = Standard
 *	1 = Class
 *	2 = Vendor
 *	3 = Reserved
 * Bit 4...0: Recipient
 *	0 = Device
 *	1 = Interface
 *	2 = Endpoint
 *	3 = Other
 *	4...31 = Reserved
 */

#define USB_DIR_OUT         0x00
#define USB_DIR_IN          0x80

#define USB_REQ_TYPE_STANDARD   (0x00 << 5)
#define USB_REQ_TYPE_CLASS      (0x01 << 5)
#define USB_REQ_TYPE_VENDOR     (0x02 << 5)
#define USB_REQ_TYPE_RESERVED   (0x03 << 5)

#define USB_RECIP_DEVICE    0x00
#define USB_RECIP_INTERFACE 0x01
#define USB_RECIP_ENDPOINT  0x02
#define USB_RECIP_OTHER     0x03

/* Clear Interface Request */
#define CF_DEVICE	(USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define CF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define CF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Get Configuration Request */
#define GC_DEVICE	(USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Descriptor Request */
#define GD_DEVICE	(USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Interface Request */
#define GI_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Get Status Request: See USB 1.1 spec, page 190 */
#define GS_DEVICE	(USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define GS_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define GS_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Address Request is handled by EZ-USB core */

/* Set Configuration Request */
#define SC_DEVICE	(USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Descriptor Request */
#define SD_DEVICE	(USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Feature Request */
#define SF_DEVICE	(USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define SF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define SF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Interface Request */
#define SI_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Synch Frame Request */
#define SY_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* USB Requests (bRequest): See USB 2.0 Spec */
#define GET_STATUS              0
#define CLEAR_FEATURE           1
/* Value '2' is reserved for future use */
#define SET_FEATURE             3
/* Value '4' is reserved for future use */
#define SET_ADDRESS             5
#define GET_DESCRIPTOR          6
#define SET_DESCRIPTOR          7
#define GET_CONFIGURATION       8
#define SET_CONFIGURATION		9
#define GET_INTERFACE           10
#define SET_INTERFACE           11
#define SYNCH_FRAME             12

/* Standard Feature Selectors: See USB 2.0 Spec */
#define DEVICE_REMOTE_WAKEUP    1
#define ENDPOINT_HALT           0

/************************** EZ-USB specific stuff **************************/
/** USB Interrupts. See EZ-USB FX2-TRM, for details */
enum usb_isr {
	SUDAV_ISR = 13,
	SOF_ISR,
	SUTOK_ISR,
	SUSPEND_ISR,
	USBRESET_ISR,
	HIGHSPEED_ISR,
	EP0ACK_ISR,
	STUB_ISR,
	EP0IN_ISR,
	EP0OUT_ISR,
	EP1IN_ISR,
	EP1OUT_ISR,
	EP2_ISR,
	EP4_ISR,
	EP6_ISR,
	EP8_ISR,
	IBN_ISR,
	EP0PINGNAK_ISR,
	EP1PINGNAK_ISR,
	EP2PINGNAK_ISR,
	EP4PINGNAK_ISR,
	EP6PINGNAK_ISR,
	EP8PINGNAK_ISR,
	ERRORLIMIT_ISR,
	EP2PIDERROR_ISR,
	EP4PIDERROR_ISR,
	EP6PIDERROR_ISR,
	EP8PIDERROR_ISR,
	EP2PFLAG_ISR,
	EP4PFLAG_ISR,
	EP6PFLAG_ISR,
	EP8PFLAG_ISR,
	EP2EFLAG_ISR,
	EP4EFLAG_ISR,
	EP6EFLAG_ISR,
	EP8EFLAG_ISR,
	EP2FFLAG_ISR,
	EP4FFLAG_ISR,
	EP6FFLAG_ISR,
	EP8FFLAG_ISR,
	GPIFCOMPLETE_ISR,
	GPIFWAVEFORM_ISR
};

/*************************** Function Prototypes ***************************/
__xdata uint8_t *usb_get_endpoint_cs_reg(uint8_t ep);
void usb_reset_data_toggle(uint8_t ep);
bool usb_handle_get_status(void);
bool usb_handle_clear_feature(void);
bool usb_handle_set_feature(void);
bool usb_handle_get_descriptor(void);
void usb_handle_set_interface(void);
void usb_handle_setup_data(void);
void usb_handle_i2c_in(void);
void usb_handle_i2c_out(void);

void i2c_recieve(void);
void ep_init(void);
void interrupt_init(void);
void io_init(void);

#endif
