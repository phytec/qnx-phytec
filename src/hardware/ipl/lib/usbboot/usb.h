/*
 * $QNXLicenseC:
 * Copyright 2016 QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#ifndef		_USB_H_INCLUDED
#define		_USB_H_INCLUDED

#include <inttypes.h>
#include <sys/types.h>
#include <errno.h>
#include <stdlib.h>
#include <hw/inout.h>

#define USE_IPL
//#define LOG_DISABLE

#include "ipl.h"

#define USB_DESC_DEVICE					0x01
#define USB_DESC_CONFIGURATION			0x02
#define USB_DESC_STRING					0x03
#define USB_DESC_INTERFACE				0x04
#define USB_DESC_ENDPOINT				0x05
#define USB_DESC_DEVICE_QUALIFIER		0x06
#define USB_DESC_OTHER_SPEED_CONF		0x07
#define USB_DESC_INTERFACE_POWER		0x08
#define USB_DESC_INTERFACE_ASSOCIATION	0x0B

#define USB_DESC_HID				0x21
#define USB_DESC_REPORT				0x22
#define USB_DESC_PHYSICAL			0x23
#define USB_DESC_HUB				(USB_TYPE_CLASS | USB_CLASS_HUB)

#define USB_CLASS_AUDIO				0x01
#define USB_CLASS_COMM				0x02
#define USB_CLASS_HID				0x03
#define USB_CLASS_PRN				0x07
#define USB_CLASS_UMASS				0x08
#define USB_CLASS_HUB				0x09

#define USB_ENDPOINT_CONTROL		0

#define USB_ENDPOINT_MASK			0x0F
#define USB_ENDPOINT_IN				0x80
#define USB_ENDPOINT_OUT			0x00

#define USB_ENDPOINT_ATTRIB_ETYPE_MASK			0x03
#define USB_ATTRIB_CONTROL			0x00
#define USB_ATTRIB_ISOCHRONOUS		0x01
#define USB_ATTRIB_BULK				0x02
#define USB_ATTRIB_INTERRUPT		0x03

#define USB_ENDPOINT_MPS_MSK                0x7ff
#define USB_ENDPOINT_MPS( endp )            ((endp)->wMaxPacketSize & USB_ENDPOINT_MPS_MSK )
#define USB_ENDPOINT_HB_MULT_POS            11
#define USB_ENDPOINT_HB_MULT_MSK            ( 3 << USB_ENDPOINT_HB_MULT_POS )
#define USB_ENDPOINT_HB_MULT( endp )        ((((endp)->wMaxPacketSize & USB_ENDPOINT_HB_MULT_MSK)>>USB_ENDPOINT_HB_MULT_POS) + 1)

#define	USB_GET_STATUS				0
#define	USB_CLEAR_FEATURE			1
#define	USB_SET_FEATURE				3
#define	USB_SET_ADDRESS				5
#define	USB_GET_DESCRIPTOR			6
#define	USB_SET_DESCRIPTOR			7
#define	USB_GET_CONFIGURATION		8
#define	USB_SET_CONFIGURATION		9
#define	USB_GET_INTERFACE			10
#define	USB_SET_INTERFACE			11
#define	USB_SYNCH_FRAME				12

#define USB_DEVICE_STATUS_SELF_PWR	0x01
#define USB_DEVICE_STATUS_RMT_WKUP	0x02


#define USB_ENDPOINT_STATUS_HALTED	0x01

#define USB_FEATURE_EPT_HALT		0
#define USB_FEATURE_DEV_WAKEUP		1
#define USB_FEATURE_TEST_MODE		2
	#define USB_TEST_MODE_TEST_J			0x01
	#define USB_TEST_MODE_TEST_K			0x02
	#define USB_TEST_MODE_TEST_SE0_NAK		0x03
	#define USB_TEST_MODE_TEST_PACKET		0x04
	#define USB_TEST_MODE_TEST_FORCE_ENABLE		0x05

#define USB_HUB_FEATURE_C_HUB_LOCAL_POWER	0	// HUB
#define USB_HUB_FEATURE_C_HUB_OVER_CURRENT	1	// HUB

#define USB_HUB_FEATURE_PORT_ENABLE			1
#define USB_HUB_FEATURE_PORT_SUSPEND		2
#define USB_HUB_FEATURE_PORT_OVER_CURRENT	3
#define USB_HUB_FEATURE_PORT_RESET			4
#define USB_HUB_FEATURE_PORT_POWER			8
#define USB_HUB_FEATURE_C_PORT_CONNECTION	16
#define USB_HUB_FEATURE_C_PORT_ENABLE		17
#define USB_HUB_FEATURE_C_PORT_SUSPEND		18
#define USB_HUB_FEATURE_C_PORT_OVER_CURRENT	19
#define USB_HUB_FEATURE_C_PORT_RESET		20
#define USB_HUB_FEATURE_PORT_TEST			21
#define USB_HUB_FEATURE_PORT_INDICATOR		22

#define USB_RECIPIENT_DEVICE		(0 << 0)
#define USB_RECIPIENT_INTERFACE		(1 << 0)
#define USB_RECIPIENT_ENDPOINT		(2 << 0)
#define USB_RECIPIENT_OTHER			(3 << 0)
#define USB_TYPE_STANDARD			(0 << 5)
#define USB_TYPE_CLASS				(1 << 5)
#define USB_TYPE_VENDOR				(2 << 5)
#define USB_DIRECTION_DEVICE		(0 << 7)
#define USB_DIRECTION_HOST			(1 << 7)

#define USB_RECIP_MASK				0x1f
#define USB_TYPE_MASK				(0x03 << 5)

#define USB_REQUEST_RECIPIENT(s)	(s & USB_RECIPIENT_OTHER )

#define USB_CONFIG_ATTR_RMT_WAKEUP			0x20
#define USB_CONFIG_ATTR_SELF_POWERED		0x40

/*
 * Descriptor sizes per descriptor type
 */
#define USB_DESC_SIZE_DEVICE		18
#define USB_DESC_SIZE_CONFIG		9
#define USB_DESC_SIZE_INTERFACE		9
#define USB_DESC_SIZE_ENDPOINT		7
#define USB_DESC_SIZE_ENDPOINT_AUDIO	9
#define USB_DESC_SIZE_HUB_NONVAR		7
#define USB_DESC_SIZE_HID			9

/* device speed */
#define USB_DEVICE_FULL_SPEED		0
#define USB_DEVICE_LOW_SPEED		1
#define USB_DEVICE_HIGH_SPEED		2
#define USB_DEVICE_SUPER_SPEED		3
#define USB_DEVICE_MASK_SPEED		3

#define USB_ENDPOINT_STATE_ENABLE     1
#define USB_ENDPOINT_STATE_DISABLED   2
#define USB_ENDPOINT_STATE_RESET      3
#define USB_ENDPOINT_STATE_READY      4
#define USB_ENDPOINT_STATE_NAK        5
#define USB_ENDPOINT_STATE_STALLED    6

/* defines for DC BUS state */
#define USB_BUS_STATE_DISCONNECTED	1
#define USB_BUS_STATE_CONNECTED		2
#define USB_BUS_STATE_RESUME			3

/* defines for DC DEVICE state */
#define USB_DEVICE_STATE_DEFAULT		0
#define USB_DEVICE_STATE_INSERTED		1
#define USB_DEVICE_STATE_RESET			2
#define USB_DEVICE_STATE_ADDRESSED	3
#define USB_DEVICE_STATE_CONFIGED		4
#define USB_DEVICE_STATE_REMOVED		5

#define USB_EP_FLAGS_ENABLED		(1 << 0)
#define USB_EP_FLAGS_STALLED		(1 << 1)


#define USB100_FIELD(_name, _type) _Uint8t	_name[sizeof(_type)];

typedef struct _usb100_generic_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bGeneric,				_Uint8t[1]);
} usb100_generic_descriptor_t;

typedef struct _usb100_device_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bcdUSB,					_Uint16t);
	USB100_FIELD(	bDeviceClass,			_Uint8t);
	USB100_FIELD(	bDeviceSubClass,		_Uint8t);
	USB100_FIELD(	bDeviceProtocol,		_Uint8t);
	USB100_FIELD(	bMaxPacketSize0,		_Uint8t);
	USB100_FIELD(	idVendor,				_Uint16t);
	USB100_FIELD(	idProduct,				_Uint16t);
	USB100_FIELD(	bcdDevice,				_Uint16t);
	USB100_FIELD(	iManufacturer,			_Uint8t);
	USB100_FIELD(	iProduct,				_Uint8t);
	USB100_FIELD(	iSerialNumber,			_Uint8t);
	USB100_FIELD(	bNumConfigurations,		_Uint8t);
} usb100_device_descriptor_t;

typedef struct _usb100_configuration_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	wTotalLength,			_Uint16t);
	USB100_FIELD(	bNumInterfaces,			_Uint8t);
	USB100_FIELD(	bConfigurationValue,	_Uint8t);
	USB100_FIELD(	iConfiguration,			_Uint8t);
	USB100_FIELD(	bmAttributes,			_Uint8t);
	USB100_FIELD(	MaxPower,				_Uint8t);
} usb100_configuration_descriptor_t;

typedef struct _usb100_string_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bString,				_Uint16t[1]);
} usb100_string_descriptor_t;

typedef struct _usb100_interface_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bInterfaceNumber,		_Uint8t);
	USB100_FIELD(	bAlternateSetting,		_Uint8t);
	USB100_FIELD(	bNumEndpoints,			_Uint8t);
	USB100_FIELD(	bInterfaceClass,		_Uint8t);
	USB100_FIELD(	bInterfaceSubClass,		_Uint8t);
	USB100_FIELD(	bInterfaceProtocol,		_Uint8t);
	USB100_FIELD(	iInterface,				_Uint8t);
} usb100_interface_descriptor_t;

typedef struct _usb100_endpoint_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bEndpointAddress,		_Uint8t);
	USB100_FIELD(	bmAttributes,			_Uint8t);
	USB100_FIELD(	wMaxPacketSize,			_Uint16t);
	USB100_FIELD(	bInterval,				_Uint8t);
} usb100_endpoint_descriptor_t;

typedef struct _usb100_audio_endpoint_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bEndpointAddress,		_Uint8t);
	USB100_FIELD(	bmAttributes,			_Uint8t);
	USB100_FIELD(	wMaxPacketSize,			_Uint16t);
	USB100_FIELD(	bInterval,				_Uint8t);
	USB100_FIELD(	bRefresh,				_Uint8t);
	USB100_FIELD(	bSyncAddress,			_Uint8t);
} usb100_audio_endpoint_descriptor_t;

typedef struct _usb100_hid_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bcdHID,					_Uint16t);
	USB100_FIELD(	bCountryCode,			_Uint8t);
	USB100_FIELD(	bNumDescriptors,		_Uint8t);
	USB100_FIELD(	bReportDescriptorType,	_Uint8t);
	USB100_FIELD(	wDescriptorLength,		_Uint16t);
} usb100_hid_descriptor_t;

typedef struct _usb100_hub_descriptor {
	USB100_FIELD(	bLength,				_Uint8t);
	USB100_FIELD(	bDescriptorType,		_Uint8t);
	USB100_FIELD(	bNbrPorts,				_Uint8t);
	USB100_FIELD(	wHubCharacteristics,	_Uint16t);
	USB100_FIELD(	bPwrOn2PwrGood,			_Uint8t);
	USB100_FIELD(	bHubContrCurrent,		_Uint8t);
	USB100_FIELD(	DeviceRemovable,		_Uint8t[1]);
	USB100_FIELD(	PortPwrCtrlMask,		_Uint8t[1]);
} usb100_hub_descriptor_t;


typedef struct _usb100_request {
	USB100_FIELD(	bmRequestType,				_Uint8t);
	USB100_FIELD(	bRequestCode,				_Uint8t);
	USB100_FIELD(	wValue,						_Uint16t);
	USB100_FIELD(	wIndex,						_Uint16t);
	USB100_FIELD(	wLength,					_Uint16t);
} _usb100_request_t;


typedef struct _usb_setup_packet {
    uint8_t     bRequestType;
    uint8_t     bRequest;
    uint16_t    wValue;
    uint16_t    wIndex;
    uint16_t    wLength;
} usb_setup_packet_t;

#define SETUP_DESC_REQUESTTYPE          0
#define SETUP_DESC_REQUEST              1
#define SETUP_DESC_DESCRIPTORINDEX      2
#define SETUP_DESC_DESCRIPTORTYPE       3
#define SETUP_DESC_INDEX                4
#define SETUP_DESC_LENGTH               6

#define SIZE_SETUP_REQUEST              8

typedef struct usbd_device_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;
	_Uint16t				bcdUSB;
	_Uint8t					bDeviceClass;
	_Uint8t					bDeviceSubClass;
	_Uint8t					bDeviceProtocol;
	_Uint8t					bMaxPacketSize0;
	_Uint16t				idVendor;
	_Uint16t				idProduct;
	_Uint16t				bcdDevice;
	_Uint8t					iManufacturer;
	_Uint8t					iProduct;
	_Uint8t					iSerialNumber;
	_Uint8t					bNumConfigurations;
} usbd_device_descriptor_t;

typedef struct usbd_configuration_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;
	_Uint16t				wTotalLength;
	_Uint8t					bNumInterfaces;
	_Uint8t					bConfigurationValue;
	_Uint8t					iConfiguration;
	_Uint8t					bmAttributes;
	_Uint8t					bMaxPower;
} usbd_configuration_descriptor_t;

typedef struct usbd_string_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;
	_Uint16t				bString[1];
} usbd_string_descriptor_t;

typedef struct usbd_interface_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;
	_Uint8t					bInterfaceNumber;
	_Uint8t					bAlternateSetting;
	_Uint8t					bNumEndpoints;
	_Uint8t					bInterfaceClass;
	_Uint8t					bInterfaceSubClass;
	_Uint8t					bInterfaceProtocol;
	_Uint8t					iInterface;
} usbd_interface_descriptor_t;

typedef struct usbd_endpoint_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;
	_Uint8t					bEndpointAddress;
	_Uint8t					bmAttributes;
	_Uint16t					wMaxPacketSize;
	_Uint8t					bInterval;
} usbd_endpoint_descriptor_t;

typedef struct usbd_qualifier_descriptor {
	_Uint8t					bLength;
	_Uint8t					bDescriptorType;

	_Uint16t				bcdUSB;
	_Uint8t					bDeviceClass;
	_Uint8t					bDeviceSubClass;
	_Uint8t					bDeviceProtocol;
	_Uint8t					bMaxPacketSize0;
	_Uint8t					bNumConfigurations;
	_Uint8t					bRESERVED;
} usbd_qualifier_descriptor_t;

typedef struct _usb_endpoint_t  {
	usbd_endpoint_descriptor_t	*edesc;		/* USB defined endpoint descriptor information */
	uint32_t					flags;		/* Endpoint flags set by HW driver */
	uint32_t					phase;		/* current phase of the transfer */
	void						*user;		/* used by HW driver */
} usb_endpoint_t;

/* USB transfer flags */
#define  PIPE_FLAGS_TOKEN_SETUP		0x01
#define  PIPE_FLAGS_TOKEN_STATUS	0x02
#define  PIPE_FLAGS_TOKEN_IN		0x04
#define  PIPE_FLAGS_TOKEN_OUT		0x08
#define  PIPE_FLAGS_BUFFER_PHYS		0x10
#define  PIPE_FLAGS_MULTI_XFER		0x20
#define  PIPE_FLAGS_LAST_PACKET		0x80000000

#define DC_CAP_ENDP_STALL_AUTO_CLEARED	0x00000001
#define DC_CAP_LOW_SPEED				0x00000002
#define DC_CAP_FULL_SPEED				0x00000004
#define DC_CAP_HIGH_SPEED				0x00000008
#define DC_CAP_TEST_MODES_SUPPORTED			0x00000010
#define DC_CAP_SUPER_SPEED				0x00000020

#define USB_INTR_NONE					0
#define USB_INTR_BUS_STATE				1
#define USB_INTR_EP_COMPL				2

#define USBD_URB_STATUS_MASK		0xFF000000
#define USBD_STATUS_INPROG			0x00000000
#define USBD_STATUS_CMP				0x01000000
#define USBD_STATUS_CMP_ERR			0x02000000
#define USBD_STATUS_TIMEOUT			0x04000000
#define USBD_STATUS_ABORTED			0x08000000

#define USBD_USB_STATUS_MASK				0x00FFFFFF
#define	USBD_STATUS_CRC_ERR					0x00000001
#define	USBD_STATUS_BITSTUFFING				0x00000002
#define	USBD_STATUS_TOGGLE_MISMATCH			0x00000003
#define	USBD_STATUS_STALL					0x00000004
#define	USBD_STATUS_DEV_NOANSWER			0x00000005
#define	USBD_STATUS_PID_FAILURE				0x00000006
#define	USBD_STATUS_BAD_PID					0x00000007
#define	USBD_STATUS_DATA_OVERRUN			0x00000008
#define	USBD_STATUS_DATA_UNDERRUN			0x00000009
#define	USBD_STATUS_BUFFER_OVERRUN			0x0000000C
#define	USBD_STATUS_BUFFER_UNDERRUN			0x0000000E
#define	USBD_STATUS_NOT_ACCESSED			0x0000000F
#define USBD_STATUS_NO_MEM_FOR_TRANSFER		0x00000010
#define USBD_STATUS_ISOCH_MISSED_PERIOD		0x00000020
#define USBD_STATUS_NOT_SUPPORTED			0x00000040
#define USBD_STATUS_DMA_ERR					0x00000080

typedef struct _usb_dc_t usb_dc_t;

typedef struct _usb_transfer {
	uint32_t	status;
	uint32_t	phase;
	uint32_t	flags;
	uint8_t	*buffer;
	paddr64_t buffer_paddr;
	uint32_t	buffer_len;
	uint32_t	actual_len;
	void		(*complete_cbf)(usb_dc_t *udc, usb_endpoint_t *endp, struct _usb_transfer *urb );
	void		*xdata_ptr;
} usb_transfer_t;

struct _usb_dc_t {
	void				*base;		/* device controller base address */
	int				state;		/* current state */
	uint16_t			dev_vid;		// vendor ID
	uint16_t			dev_did;		// device ID
	uint32_t			capability;	/* dc controller capability */
	uint8_t			speed;		/* Speed */
	uint8_t			num_endpoints;	/* number of endpoints we are going to use */
	uint8_t			addr;		/* address */
	uint8_t			cfg;

	void				*dc_data;		/* device controller handle */
	void				*bs_data;		/* board specific handle */
	int					verbose;

	int				istatus;

	usb_endpoint_t	*endps; //array of endpoints
	usb_endpoint_t	*ep0;
	usb_transfer_t	*ep0_urb;

	uint8_t			*malloc_buffer;
	size_t			malloc_size;
	paddr64_t		malloc_buffer_phys;
	uint8_t			*bCurrent;
	uint8_t			*bEnd;

	int				(*init)(void *);
	int				(*fini)(usb_dc_t *);		// Shutdown

	int				(*usb_device_state_change)(usb_dc_t *, uint32_t device_state );
	int				(*usb_set_device_speed)(usb_dc_t *, uint32_t speed );

	int				(*address_device)(usb_dc_t *, uint8_t);
	int				(*config_device)(usb_dc_t *, uint8_t);
	int				(*set_bus_state)(usb_dc_t *, uint32_t device_state );
	int				(*endpoint_enable)(usb_dc_t *, usb_endpoint_t *);
	int				(*endpoint_disable)(usb_dc_t *, usb_endpoint_t *);
	int				(*set_endpoint_state)(usb_dc_t *, usb_endpoint_t *, uint32_t ep_state );
	int				(*clear_endpoint_state)(usb_dc_t *, usb_endpoint_t *, uint32_t ep_state );
	int				(*endpoint_transfer)(usb_dc_t *, usb_endpoint_t *, usb_transfer_t *urb, uint32_t flags);		// send command
	int				(*endpoint_abort)(usb_dc_t *, usb_endpoint_t *);	// command done
	int				(*interrupt)(usb_dc_t *, uint32_t *emask);	// return interrupt type, and emask store the endpoint number mask if it's for transfer cmplete


	/* below are device level (eg fastboot) callback */
	int				(*setup)(usb_dc_t *, usb_setup_packet_t *);
	int				(*connect)(usb_dc_t *, uint32_t); //insertion/removal processing
	int				(*reset)(usb_dc_t *); //bus reset

	/*below are board specific callouts */
	int				(*bs_init)(void **, uint32_t);
	int				(*bs_fini)(void *, uint32_t);
	int				(*bs_phy_reset)(void *, uint32_t);

	/*board specific utility routine*/
	void				(*bs_delay_ms)(uint32_t);
	void				(*bs_delay_ns)(uint32_t);
	void				(*bs_bus_sync)(void *, uint32_t);
};

#define LOG(udc, level, fmt, ... )

#ifndef LOG_DISABLE
#define LOGS(udc, level, str) \
	do { if(level <= udc->verbose) ser_putstr(str), ser_putstr("\n"); } while(0)

#define LOG1H(udc, level, str, hex) \
	do { if(level <= udc->verbose) ser_putstr(str), ser_puthex(hex), ser_putstr("\n"); } while(0)
#else
#define LOGS(udc, level, str)
#define LOG1H(udc, level, str, hex)
#endif

extern int usb_init_udc(usb_dc_t *udc);
extern int usb_exit_udc(usb_dc_t *udc);
extern int usb_handle_interrupts(usb_dc_t *udc);
extern int usb_malloc(usb_dc_t *udc, void **buf, paddr64_t *buf_phys, uint32_t flags, uint32_t size);
extern int usb_calloc(usb_dc_t *udc, void **buf, paddr64_t *buf_phys, uint32_t flags, uint32_t size);
extern int usb_free(usb_dc_t *udc, void *buf);
extern void usb_delay(usb_dc_t *udc, uint32_t);
extern void usb_delay_ns(usb_dc_t *udc, uint32_t);
extern void usb_bus_sync(usb_dc_t *udc, uint32_t);
extern int usb_ep_disable(usb_dc_t *udc, usb_endpoint_t *ep);
extern int usb_ep_enable(usb_dc_t *udc, usbd_endpoint_descriptor_t *edesc, usb_endpoint_t **ep);
extern int usb_ep_transfer(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb, uint32_t flags);
extern int usb_setup_packet_process(usb_dc_t *udc, uint8_t *buffer);

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/usbboot/usb.h $ $Rev: 808052 $")
#endif
