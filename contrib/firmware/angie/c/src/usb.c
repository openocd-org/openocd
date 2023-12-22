// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
	File : usb.c															*
	Contents : usb communication handling code for NanoXplore USB-JTAG		*
	ANGIE adapter hardware.													*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "usb.h"
#include "stdint.h"
#include "delay.h"
#include "io.h"
#include "reg_ezusb.h"
#include <fx2macros.h>
#include <serial.h>
#include <stdio.h>
#include "i2c.h"

/* Also update external declarations in "include/usb.h" if making changes to
 * these variables!
 */
volatile bool ep1_out;
volatile bool ep1_in;

volatile __xdata __at 0xE6B8 struct setup_data setup_data;

/* Define number of endpoints (except Control Endpoint 0) in a central place.
 * Be sure to include the necessary endpoint descriptors!
 */
#define NUM_ENDPOINTS 3

__code struct usb_device_descriptor device_descriptor = {
	.blength =		sizeof(struct usb_device_descriptor),
	.bdescriptortype =	DESCRIPTOR_TYPE_DEVICE,
	.bcdusb =			0x0200,	    /* BCD: 02.00 (Version 2.0 USB spec) */
	.bdeviceclass =		0xEF,
	.bdevicesubclass =	0x02,
	.bdeviceprotocol =	0x01,
	.bmaxpacketsize0 =	64,
	.idvendor =			0x584e,
	.idproduct =		0x424e,
	.bcddevice =		0x0000,
	.imanufacturer =	1,
	.iproduct =			2,
	.iserialnumber =	3,
	.bnumconfigurations =	1
};

/* WARNING: ALL config, interface and endpoint descriptors MUST be adjacent! */

__code struct usb_config_descriptor config_descriptor = {
	.blength =		sizeof(struct usb_config_descriptor),
	.bdescriptortype =	DESCRIPTOR_TYPE_CONFIGURATION,
	.wtotallength =		sizeof(struct usb_config_descriptor) +
		3 * sizeof(struct usb_interface_descriptor) +
		((NUM_ENDPOINTS + 2) * sizeof(struct usb_endpoint_descriptor)),
	.bnuminterfaces =	2,
	.bconfigurationvalue =	1,
	.iconfiguration =	1,	/* String describing this configuration */
	.bmattributes =		0x80,	/* Only MSB set according to USB spec */
	.maxpower =		50	/* 100 mA */
};

__code struct usb_interface_association_descriptor interface_association_descriptor = {
	.blength = sizeof(struct usb_interface_association_descriptor),
	.bdescriptortype =	DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,
	.bfirstinterface =	0x01,
	.binterfacecount =	0x02,
	.bfunctionclass =	0x02,
	.bfunctionsubclass =	0x00,
	.bfunctionprotocol =	0x00,
	.ifunction =	0x00
};

__code struct usb_interface_descriptor interface_descriptor00 = {
	.blength = sizeof(struct usb_interface_descriptor),
	.bdescriptortype =	DESCRIPTOR_TYPE_INTERFACE,
	.binterfacenumber =	0,
	.balternatesetting =	0,
	.bnumendpoints =	NUM_ENDPOINTS,
	.binterfaceclass =	0XFF,
	.binterfacesubclass =	0x00,
	.binterfaceprotocol =	0x00,
	.iinterface =		4
};

__code struct usb_endpoint_descriptor bulk_ep1_out_endpoint_descriptor = {
	.blength =		sizeof(struct usb_endpoint_descriptor),
	.bdescriptortype =	0x05,
	.bendpointaddress =	(1 | USB_DIR_OUT),
	.bmattributes =		0x02,
	.wmaxpacketsize =	64,
	.binterval =		0
};

__code struct usb_endpoint_descriptor bulk_ep1_in_endpoint_descriptor = {
	.blength =		sizeof(struct usb_endpoint_descriptor),
	.bdescriptortype =	0x05,
	.bendpointaddress =	(1 | USB_DIR_IN),
	.bmattributes =		0x02,
	.wmaxpacketsize =	64,
	.binterval =		0
};

__code struct usb_endpoint_descriptor bulk_ep2_endpoint_descriptor = {
	.blength =		sizeof(struct usb_endpoint_descriptor),
	.bdescriptortype =	0x05,
	.bendpointaddress =	(2 | USB_DIR_OUT),
	.bmattributes =		0x02,
	.wmaxpacketsize =	512,
	.binterval =		0
};

__code struct usb_interface_descriptor interface_descriptor01 = {
	.blength = sizeof(struct usb_interface_descriptor),
	.bdescriptortype =	DESCRIPTOR_TYPE_INTERFACE,
	.binterfacenumber =	1,
	.balternatesetting =	0,
	.bnumendpoints =	2,
	.binterfaceclass =	0x0A,
	.binterfacesubclass =	0x00,
	.binterfaceprotocol =	0x00,
	.iinterface =		0x00
};

__code struct usb_endpoint_descriptor bulk_ep6_out_endpoint_descriptor = {
	.blength =		sizeof(struct usb_endpoint_descriptor),
	.bdescriptortype =	0x05,
	.bendpointaddress =	(6 | USB_DIR_OUT),
	.bmattributes =		0x02,
	.wmaxpacketsize =	512,
	.binterval =		0
};

__code struct usb_endpoint_descriptor bulk_ep8_in_endpoint_descriptor = {
	.blength =		sizeof(struct usb_endpoint_descriptor),
	.bdescriptortype =	0x05,
	.bendpointaddress =	(8 | USB_DIR_IN),
	.bmattributes =		0x02,
	.wmaxpacketsize =	512,
	.binterval =		0
};
__code struct usb_language_descriptor language_descriptor = {
	.blength =		4,
	.bdescriptortype =	DESCRIPTOR_TYPE_STRING,
	.wlangid =		{0x0409} /* US English */
};

__code struct usb_string_descriptor strmanufacturer =
	STR_DESCR(16, 'N', 'a', 'n', 'o', 'X', 'p', 'l', 'o', 'r', 'e', ',', ' ', 'S', 'A', 'S', '.');

__code struct usb_string_descriptor strproduct =
	STR_DESCR(13, 'A', 'N', 'G', 'I', 'E', ' ', 'A', 'd', 'a', 'p', 't', 'e', 'r');

__code struct usb_string_descriptor strserialnumber =
	STR_DESCR(6, '0', '0', '0', '0', '0', '1');

__code struct usb_string_descriptor strconfigdescr  =
	STR_DESCR(12, 'J', 'T', 'A', 'G', ' ', 'A', 'd', 'a', 'p', 't', 'e', 'r');

/* Table containing pointers to string descriptors */
__code struct usb_string_descriptor *__code en_string_descriptors[4] = {
	&strmanufacturer,
	&strproduct,
	&strserialnumber,
	&strconfigdescr
};
void sudav_isr(void)__interrupt SUDAV_ISR
{
	EXIF &= ~0x10;  /* Clear USBINT: Main global interrupt */
	USBIRQ = SUDAVI;
	EP0CS |= HSNAK;
	usb_handle_setup_data();
}
void sof_isr(void)__interrupt	SOF_ISR
{
}
void sutok_isr(void)__interrupt	SUTOK_ISR
{
}
void suspend_isr(void)__interrupt	SUSPEND_ISR
{
}
void usbreset_isr(void)__interrupt	USBRESET_ISR
{
}
void highspeed_isr(void)__interrupt	HIGHSPEED_ISR
{
}
void ep0ack_isr(void)__interrupt	EP0ACK_ISR
{
}
void stub_isr(void)__interrupt	STUB_ISR
{
}
void ep0in_isr(void)__interrupt	EP0IN_ISR
{
}
void ep0out_isr(void)__interrupt	EP0OUT_ISR
{
}
void ep1in_isr(void)__interrupt	EP1IN_ISR
{
	ep1_in = true;

	EXIF &= ~0x10;  /* Clear USBINT: Main global interrupt */
	EPIRQ = 0x04;	/* Clear individual EP1IN IRQ */
}
void ep1out_isr(void)__interrupt	EP1OUT_ISR
{
	ep1_out = true;

	EXIF &= ~0x10;  /* Clear USBINT: Main global interrupt */
	EPIRQ = 0x08;	/* Clear individual EP1OUT IRQ */
}
void ep2_isr(void)__interrupt	EP2_ISR
{
	ep1_out = false; /* Does nothing but required by the compiler */
}
void ep4_isr(void)__interrupt	EP4_ISR
{
}
void ep6_isr(void)__interrupt	EP6_ISR
{
	i2c_recieve();
	EXIF &= ~0x10;  /* Clear USBINT: Main global interrupt */
	EPIRQ = 0x40;	/* Clear individual EP6OUT IRQ */

}
void ep8_isr(void)__interrupt	EP8_ISR
{
	EXIF &= ~0x10;  /* Clear USBINT: Main global interrupt */
	EPIRQ = 0x80;	/* Clear individual EP8IN IRQ */
}
void ibn_isr(void)__interrupt	IBN_ISR
{
}
void ep0pingnak_isr(void)__interrupt	EP0PINGNAK_ISR
{
}
void ep1pingnak_isr(void)__interrupt	EP1PINGNAK_ISR
{
}
void ep2pingnak_isr(void)__interrupt	EP2PINGNAK_ISR
{
}
void ep4pingnak_isr(void)__interrupt	EP4PINGNAK_ISR
{
}
void ep6pingnak_isr(void)__interrupt	EP6PINGNAK_ISR
{
}
void ep8pingnak_isr(void)__interrupt	EP8PINGNAK_ISR
{
}
void errorlimit_isr(void)__interrupt	ERRORLIMIT_ISR
{
}
void ep2piderror_isr(void)__interrupt	EP2PIDERROR_ISR
{
}
void ep4piderror_isr(void)__interrupt	EP4PIDERROR_ISR
{
}
void ep6piderror_isr(void)__interrupt	EP6PIDERROR_ISR
{
}
void ep8piderror_isr(void)__interrupt	EP8PIDERROR_ISR
{
}
void ep2pflag_isr(void)__interrupt	EP2PFLAG_ISR
{
}
void ep4pflag_isr(void)__interrupt	EP4PFLAG_ISR
{
}
void ep6pflag_isr(void)__interrupt	EP6PFLAG_ISR
{
}
void ep8pflag_isr(void)__interrupt	EP8PFLAG_ISR
{
}
void ep2eflag_isr(void)__interrupt	EP2EFLAG_ISR
{
}
void ep4eflag_isr(void)__interrupt	EP4EFLAG_ISR
{
}
void ep6eflag_isr(void)__interrupt	EP6EFLAG_ISR
{
}
void ep8eflag_isr(void)__interrupt	EP8EFLAG_ISR
{
}
void ep2fflag_isr(void)__interrupt	EP2FFLAG_ISR
{
}
void ep4fflag_isr(void)__interrupt	EP4FFLAG_ISR
{
}
void ep6fflag_isr(void)__interrupt	EP6FFLAG_ISR
{
}
void ep8fflag_isr(void)__interrupt	EP8FFLAG_ISR
{
}
void gpifcomplete_isr(void)__interrupt	GPIFCOMPLETE_ISR
{
}
void gpifwaveform_isr(void)__interrupt	GPIFWAVEFORM_ISR
{
}

/**
 * Return the control/status register for an endpoint
 *
 * @param ep endpoint address
 * @return on success: pointer to Control & Status register for endpoint
 *  specified in \a ep
 * @return on failure: NULL
 */
__xdata uint8_t *usb_get_endpoint_cs_reg(uint8_t ep)
{
	/* Mask direction bit */
	uint8_t ep_num = ep & ~0x80;

	switch (ep_num) {
	case 0:
		return	&EP0CS;
	case 1:
		return	ep & 0x80 ? &EP1INCS : &EP1OUTCS;
	case 2:
		return	&EP2CS;
	case 4:
		return	&EP4CS;
	case 6:
		return	&EP6CS;
	case 8:
		return	&EP8CS;
	default:
		return	NULL;
	}
}

void usb_reset_data_toggle(uint8_t ep)
{
	/* TOGCTL register:
		+----+-----+-----+------+-----+-------+-------+-------+
		| Q  |  S  |  R  |  IO  |  EP3  |  EP2  |  EP1  |  EP0  |
		+----+-----+-----+------+-----+-------+-------+-------+

		To reset data toggle bits, we have to write the endpoint direction (IN/OUT)
		to the IO bit and the endpoint number to the EP2..EP0 bits. Then, in a
		separate write cycle, the R bit needs to be set.
	*/
	TOGCTL = (((ep & 0x80) >> 3) + (ep & 0x0F));
	TOGCTL |= BMRESETTOGGLE;
}

/**
 * Handle GET_STATUS request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_get_status(void)
{
	uint8_t *ep_cs;
	switch (setup_data.bmrequesttype) {
	case GS_DEVICE:
		/* Two byte response: Byte 0, Bit 0 = self-powered, Bit 1 = remote wakeup.
		 *                    Byte 1: reserved, reset to zero */
		EP0BUF[0] = 0;
		EP0BUF[1] = 0;

		/* Send response */
		EP0BCH = 0;
		syncdelay(3);
		EP0BCL = 2;
		syncdelay(3);
		break;
	case GS_INTERFACE:
		/* Always return two zero bytes according to USB 1.1 spec, p. 191 */
		EP0BUF[0] = 0;
		EP0BUF[1] = 0;

		/* Send response */
		EP0BCH = 0;
		syncdelay(3);
		EP0BCL = 2;
		syncdelay(3);
		break;
	case GS_ENDPOINT:
		/* Get stall bit for endpoint specified in low byte of wIndex */
		ep_cs = usb_get_endpoint_cs_reg(setup_data.windex & 0xff);

		if (*ep_cs & EPSTALL)
			EP0BUF[0] = 0x01;
		else
			EP0BUF[0] = 0x00;

		/* Second byte sent has to be always zero */
		EP0BUF[1] = 0;

		/* Send response */
		EP0BCH = 0;
		syncdelay(3);
		EP0BCL = 2;
		syncdelay(3);
		break;
	default:
		return false;
	}
	return true;
}

/**
 * Handle CLEAR_FEATURE request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_clear_feature(void)
{
	__xdata uint8_t *ep_cs;

	switch (setup_data.bmrequesttype) {
	case CF_DEVICE:
		/* Clear remote wakeup not supported: stall EP0 */
		STALL_EP0();
		break;
	case CF_ENDPOINT:
		if (setup_data.wvalue == 0) {
			/* Unstall the endpoint specified in wIndex */
			ep_cs = usb_get_endpoint_cs_reg(setup_data.windex);
			if (!ep_cs)
				return false;
			*ep_cs &= ~EPSTALL;
		} else {
			/* Unsupported feature, stall EP0 */
			STALL_EP0();
		}
		break;
	default:
		/* Vendor commands... */
		break;
	}
	return true;
}

/**
 * Handle SET_FEATURE request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_set_feature(void)
{
	__xdata uint8_t *ep_cs;

	switch (setup_data.bmrequesttype) {
	case SF_DEVICE:
		if (setup_data.wvalue == 2)
			return true;
		break;
	case SF_ENDPOINT:
		if (setup_data.wvalue == 0) {
			/* Stall the endpoint specified in wIndex */
			ep_cs = usb_get_endpoint_cs_reg(setup_data.windex);
			if (!ep_cs)
				return false;
			*ep_cs |= EPSTALL;
		} else {
			/* Unsupported endpoint feature */
			return false;
		}
		break;
	default:
		/* Vendor commands... */
		break;
	}

	return true;
}

/**
 * Handle GET_DESCRIPTOR request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_get_descriptor(void)
{
	__xdata uint8_t descriptor_type;
	__xdata uint8_t descriptor_index;

	descriptor_type = (setup_data.wvalue & 0xff00) >> 8;
	descriptor_index = setup_data.wvalue & 0x00ff;

	switch (descriptor_type) {
	case DESCRIPTOR_TYPE_DEVICE:
		SUDPTRH = HI8(&device_descriptor);
		SUDPTRL = LO8(&device_descriptor);
		break;
	case DESCRIPTOR_TYPE_CONFIGURATION:
		SUDPTRH = HI8(&config_descriptor);
		SUDPTRL = LO8(&config_descriptor);
		break;
	case DESCRIPTOR_TYPE_STRING:
		if (setup_data.windex == 0) {
			/* Supply language descriptor */
			SUDPTRH = HI8(&language_descriptor);
			SUDPTRL = LO8(&language_descriptor);
		} else if (setup_data.windex == 0x0409 /* US English */) {
			/* Supply string descriptor */
			SUDPTRH = HI8(en_string_descriptors[descriptor_index - 1]);
			SUDPTRL = LO8(en_string_descriptors[descriptor_index - 1]);
		} else {
			return false;
		}
		break;
	default:
		/* Unsupported descriptor type */
		return false;
	}
	return true;
}

/**
 * Handle SET_INTERFACE request.
 */
void usb_handle_set_interface(void)
{
	/* Reset Data Toggle */
	usb_reset_data_toggle(USB_DIR_IN  | 4);
	usb_reset_data_toggle(USB_DIR_OUT | 2);

	/* Unstall & clear busy flag of all valid IN endpoints */
	EP1INCS = 0 | EPBSY;

	/* Unstall all valid OUT endpoints, reset bytecounts */
	EP1OUTCS = 0;
	EP1OUTBC = 0;
	syncdelay(3);
}

/* Initialize GPIF interface transfer count */
void set_gpif_cnt(uint32_t count)
{
	GPIFTCB3 = (uint8_t)(((uint32_t)(count) >> 24) & 0x000000ff);
	syncdelay(3);
	GPIFTCB2 = (uint8_t)(((uint32_t)(count) >> 16) & 0x000000ff);
	syncdelay(3);
	GPIFTCB1 = (uint8_t)(((uint32_t)(count) >> 8) & 0x000000ff);
	syncdelay(3);
	GPIFTCB0 = (uint8_t)((uint32_t)(count) & 0x000000ff);
}

/*
 * Vendor commands handling:
*/
#define VR_CFGOPEN		0xB0
#define VR_CFGCLOSE		0xB1

uint8_t	ix;
uint8_t bcnt;
uint8_t __xdata *eptr;
uint16_t wcnt;
uint32_t __xdata gcnt;
bool usb_handle_send_bitstream(void)
{
	eptr = EP0BUF;						/* points to EP0BUF 64-byte register */
	wcnt = setup_data.wlength;			/* total transfer count */

	/* Clear EP0BUF for OUT requests */
	if (setup_data.bmrequesttype & 0x80) {
		bcnt = ((wcnt > 64) ? 64 : wcnt);
		for (ix = 0; ix < bcnt; ix++)
			eptr[ix] = 0;
	}

	switch (setup_data.brequest) {
	case VR_CFGOPEN:
		/* Clear bytecount / to allow new data in / to stops NAKing */
		EP0BCH = 0;
		EP0BCL = 0;
		while (EP0CS & EPBSY)
			; /* wait to finish transferring in EP0BUF, until not busy */
		gcnt  = ((uint32_t)(eptr[0]) << 24) | ((uint32_t)(eptr[1]) << 16)
		| ((uint32_t)(eptr[2]) << 8) | (uint32_t)(eptr[3]);
		/* Angie board FPGA bitstream download */
		switch ((setup_data.wvalue) & 0x00C0) {
		case 0x00:
			PIN_PROGRAM_B = 0;		/* Apply RPGM- pulse */
			GPIFWFSELECT = 0xF2;	/* Restore Config mode waveforms select */
			syncdelay(3);
			EP2FIFOCFG = BMAUTOOUT;	/* and Automatic 8-bit GPIF OUT mode */
			syncdelay(3);
			PIN_PROGRAM_B = 1;		/* Negate RPGM- pulse */
			delay_ms(10);			/* FPGA init time < 10mS */
			set_gpif_cnt(gcnt);		/* Initialize GPIF interface transfer count */
			PIN_RDWR_B = 0;
			PIN_CSI_B = 0;
			GPIFTRIG = GPIF_EP2;	/* Trigger GPIF OUT transfer on EP2 */
			syncdelay(3);
			break;
		default:
			break;
		}
		break;
	case VR_CFGCLOSE:
		ix = 10;
		/* wait until GPIF transaction has been completed */
		while ((GPIFTRIG & BMGPIFDONE) == 0) {
			if (ix-- == 0) {
				break;
			}
			delay_ms(1);
		}
		switch ((setup_data.wvalue) & 0x00C0) {
			case 0x00:
				PIN_CSI_B = 1;
				PIN_RDWR_B = 1;
				IFCONFIG &= 0xFC; /* Exit gpif mode */
				break;
			default:
				break;
		}
		EP0BCH = 0;
		EP0BCL = (uint8_t)(setup_data.wlength);	/* Signal buffer is filled */
		break;
	default:
		return true;	/* Error: unknown VR command */
	}
	return false;		/* no error; command handled OK */
}

/**
 * Handle the arrival of a USB Control Setup Packet.
 */
void usb_handle_setup_data(void)
{
	switch (setup_data.brequest) {
	case GET_STATUS:
		if (!usb_handle_get_status())
			STALL_EP0();
		break;
	case CLEAR_FEATURE:
		if (!usb_handle_clear_feature())
			STALL_EP0();
		break;
	case 2: case 4:
		/* Reserved values */
		STALL_EP0();
		break;
	case SET_FEATURE:
		if (!usb_handle_set_feature())
			STALL_EP0();
		break;
	case SET_ADDRESS:
		/* Handled by USB core */
		break;
	case SET_DESCRIPTOR:
		/* Set Descriptor not supported. */
		STALL_EP0();
		break;
	case GET_DESCRIPTOR:
		if (!usb_handle_get_descriptor())
			STALL_EP0();
		break;
	case GET_CONFIGURATION:
		/* ANGIE has only one configuration, return its index */
		EP0BUF[0] = config_descriptor.bconfigurationvalue;
		EP0BCH = 0;
		EP0BCL = 1;
		syncdelay(3);
		break;
	case SET_CONFIGURATION:
		/* ANGIE has only one configuration -> nothing to do */
		break;
	case GET_INTERFACE:
		/* ANGIE only has one interface, return its number */
		EP0BUF[0] = interface_descriptor00.binterfacenumber;
		EP0BCH = 0;
		EP0BCL = 1;
		syncdelay(3);
		break;
	case SET_INTERFACE:
		usb_handle_set_interface();
		break;
	case SYNCH_FRAME:
		/* Isochronous endpoints not used -> nothing to do */
		break;
	default:
		/* if not Vendor command, Stall EndPoint 0 */
		if (usb_handle_send_bitstream())
			STALL_EP0();
		break;
	}
}

/**
 * Handle the initialization of endpoints.
 */
void ep_init(void)
{
	EP1INCFG = 0xA0;
	syncdelay(3);
	EP1OUTCFG = 0xA0;
	syncdelay(3);
	EP2CFG = 0xA0;
	syncdelay(3);
	EP4CFG = 0x00;
	syncdelay(3);
	EP6CFG = 0xA2;
	syncdelay(3);
	EP8CFG = 0xE2;
	syncdelay(3);

	/* arm EP1-OUT */
	EP1OUTBC = 0;
	syncdelay(3);
	EP1OUTBC = 0;
	syncdelay(3);

	/* arm EP1-IN */
	EP1INBC = 0;
	syncdelay(3);
	EP1INBC = 0;
	syncdelay(3);

	/* arm EP6-OUT */
	EP6BCL = 0x80;
	syncdelay(3);
	EP6BCL = 0x80;
	syncdelay(3);

	/* Standard procedure to reset FIFOs */
	FIFORESET = BMNAKALL;	/* NAK all transfers during the reset */
	syncdelay(3);
	FIFORESET = 0x02;		/* reset EP2 FIFO */
	syncdelay(3);
	FIFORESET = 0x00;		/* deactivate the NAK all */
	syncdelay(3);
	EP2FIFOCFG = 0x00;
	syncdelay(3);
	EP2FIFOCFG = BMAUTOOUT;	/* Automatic 8-bit GPIF OUT mode */
	syncdelay(3);
}

void i2c_recieve(void)
{
	PIN_SDA_DIR = 0;
	if (EP6FIFOBUF[0] == 1) {
		uint8_t rdwr = EP6FIFOBUF[0];   //read
		uint8_t data_count = EP6FIFOBUF[1]; //data sent count
		uint8_t count = EP6FIFOBUF[2];  //requested data count
		uint8_t adr = EP6FIFOBUF[3];    //address
		uint8_t address = get_address(adr, rdwr);   //address byte (read command)
		uint8_t address_2 = get_address(adr, 0);   //address byte 2 (write command)

		printf("%d\n", address - 1);

		/*  start:   */
		start_cd();
		/*  address:   */
		send_byte(address_2);   //write
		/*  ack:  */
		uint8_t ack = get_ack();

		/*   send data   */
		if (data_count) { //if there is a byte reg
			for (uint8_t i = 0; i < data_count; i++) {
				send_byte(EP6FIFOBUF[i + 4]);
				/*  ack():  */
				ack = get_ack();
			}
		}

		/*  repeated start:  */
		repeated_start();
		/*  address:   */
		send_byte(address);
		/*  get ack:  */
		ack = get_ack();

		/*   receive data   */
		for (uint8_t i = 0; i < count - 1; i++) {
			EP8FIFOBUF[i] = receive_byte();

			/*  send ack: */
			send_ack();
		}

		EP8FIFOBUF[count - 1] = receive_byte();

		/*  send Nack:  */
		send_nack();

		/*   stop   */
		stop_cd();

		EP8BCH = 0; //EP8
		syncdelay(3);
		EP8BCL = count; //EP8

		EP6BCL = 0x80; //EP6
		syncdelay(3);
		EP6BCL = 0x80; //EP6
	} else {
		uint8_t rdwr = EP6FIFOBUF[0];   //write
		uint8_t count = EP6FIFOBUF[1];  //data count
		uint8_t adr = EP6FIFOBUF[2];    //address
		uint8_t address = get_address(adr, rdwr);   //address byte (read command)
		uint8_t ack_cnt = 0;

/*  start():   */
		start_cd();
/*  address:   */
		send_byte(address);   //write
/*  ack():  */
		if (!get_ack())
			ack_cnt++;
/*   send data  */
		for (uint8_t i = 0; i < count; i++) {
			send_byte(EP6FIFOBUF[i + 3]);

			/*  get ack:  */
			if (!get_ack())
				ack_cnt++;
		}

/*   stop   */
		stop_cd();

		EP8FIFOBUF[0] = ack_cnt;

		EP8BCH = 0; //EP8
		syncdelay(3);
		EP8BCL = 1; //EP8

		EP6BCL = 0x80; //EP6
		syncdelay(3);
		EP6BCL = 0x80; //EP6
	}
}

/**
 * Interrupt initialization. Configures USB interrupts.
 **/
void interrupt_init(void)
{
	/* Enable Interrupts */
	EA = 1;

	/* Enable USB interrupt (EIE register) */
	EUSB = 1;
	EICON |= 0x20;

	/* Enable INT 2 & 4 Autovectoring */
	INTSETUP |= (AV2EN | AV4EN);

	/* Enable individual EP1OUT&IN & EP6&8 interrupts */
	EPIE |= 0xCC;

	/* Clear individual USB interrupt IRQ */
	EPIRQ = 0xCC;

	/* Enable SUDAV interrupt */
	USBIEN |= SUDAVI;

	/* Clear SUDAV interrupt */
	USBIRQ = SUDAVI;
}

/**
 * Handle the initialization of io ports.
 */
void io_init(void)
{
	/* PORT A */
	PORTACFG = 0x01;	/* 0: normal ou 1: alternate function (each bit) */
	OEA = 0xEF;	/* all OUT exept INIT_B IN */
	IOA = 0xFF;
	PIN_RDWR_B = 1;
	PIN_CSI_B = 1;
	PIN_PROGRAM_B = 1;

	/* PORT B */
	OEB = 0xEF;	/* all OUT exept TDO */
	IOB = 0xFF;
	PIN_TRST = 1;
	PIN_TMS = 0;
	PIN_TCK = 0;
	PIN_TDI = 0;
	PIN_SRST = 1;



	/* PORT C */
	PORTCCFG = 0x00;	/* 0: normal ou 1: alternate function (each bit) */
	OEC = 0xFF;
	IOC = 0xFF;

	/* PORT D */
	OED = 0xFF;
	IOD = 0xFF;
	PIN_SDA_DIR = 0;
}
