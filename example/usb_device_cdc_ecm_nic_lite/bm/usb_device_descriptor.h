/**
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_DEVICE_SPECIFIC_BCD_VERSION (0x0200U)
#define USB_DEVICE_DEMO_BCD_VERSION (0x0001U)

#define USB_DEVICE_CLASS (0x02U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)

#define USB_DEVICE_VID (0x1FC9U)
#define USB_DEVICE_PID (0x1790U)

#define USB_DEVICE_MAX_POWER (0x32U)

#define USB_DEVICE_STRING_COUNT (0x05U)
#define USB_DEVICE_MANUFACTURER_STRING_INDEX (0x01U)
#define USB_DEVICE_PRODUCT_STRING_INDEX (0x02U)
#define USB_DEVICE_SERIAL_NUMBER_STRING_INDEX (0x03U)
#define USB_DEVICE_MAC_ADDRESS_STRING_INDEX (0X04U)

#define USB_DEVICE_LANGID (0x0409U)
#define USB_DEVICE_MANUFACTURER_STRING ("NXP Semiconductors")
#define USB_DEVICE_PRODUCT_STRING ("USB CDC-ECM Network Interface Card Demo")
#define USB_DEVICE_SERIAL_NUMBER_STRING ("2412#YANOOFLY")

#define USB_DEVICE_CONFIGURATION_COUNT (0x01U)

#define USB_DEVICE_CDC_CLASS_COMM_CODE (0x02U)
#define USB_DEVICE_CDC_CLASS_DATA_CODE (0x0AU)
#define USB_DEVICE_CDC_ECM_SUBCLASS_CODE (0x06U)
#define USB_DEVICE_CDC_ECM_PROTOCOL_CODE (0x00U)

#define USB_DEVICE_CDC_ECM_INTERFACE_COUNT (0x02U)
#define USB_DEVICE_CDC_ECM_CONFIGURATION_VALUE (0x01U)

#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_NUMBER (0x00U)
#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_ALTERNATE (0x00U)
#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_ALTERNATE_COUNT (0x01U)
#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_CLASS_CODE (USB_DEVICE_CDC_CLASS_COMM_CODE)
#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_SUBCLASS_CODE (USB_DEVICE_CDC_ECM_SUBCLASS_CODE)
#define USB_DEVICE_CDC_ECM_COMM_INTERFACE_PROTOCOL_CODE (USB_DEVICE_CDC_ECM_PROTOCOL_CODE)
#define USB_DEVICE_CDC_ECM_COMM_ENDPOINT_NUMBER (0x01U)

#define USB_DEVICE_CDC_ECM_COMM_INTERRUPT_IN_EP_NUMBER (0x03U)
#define USB_DEIVCE_CDC_ECM_COMM_INTERRUPT_IN_EP_MAXPKT_SIZE (0x0010U)
#define USB_DEIVCE_CDC_ECM_COMM_INTERRUPT_IN_EP_INTERVAL_FS (0xA0U)
#define USB_DEIVCE_CDC_ECM_COMM_INTERRUPT_IN_EP_INTERVAL_HS (0x0BU)

#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_NUMBER (0x01U)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_ALTERNATE_COUNT (0x02U)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_ALTERNATE0 (0x00U)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_ALTERNATE1 (0x01U)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_CLASS_CODE (USB_DEVICE_CDC_CLASS_DATA_CODE)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_SUBCLASS_CODE (0x00U)
#define USB_DEVICE_CDC_ECM_DATA_INTERFACE_PROTOCOL_CODE (0x00U)
#define USB_DEVICE_CDC_ECM_DATA_ENDPOINT_NUMBER (0x02U)

#define USB_DEVICE_CDC_ECM_DATA_BULK_IN_EP_NUMBER (0x01U)
#define USB_DEVICE_CDC_ECM_DATA_BULK_IN_EP_MAXPKT_SIZE_FS (0x0040U)
#define USB_DEVICE_CDC_ECM_DATA_BULK_IN_EP_MAXPKT_SIZE_HS (0x0200U)
#define USB_DEIVCE_CDC_ECM_DATA_BULK_IN_EP_INTERVAL (0x00U)

#define USB_DEVICE_CDC_ECM_DATA_BULK_OUT_EP_NUMBER (0x02U)
#define USB_DEVICE_CDC_ECM_DATA_BULK_OUT_EP_MAXPKT_SIZE_FS (0x0040U)
#define USB_DEVICE_CDC_ECM_DATA_BULK_OUT_EP_MAXPKT_SIZE_HS (0x0200U)
#define USB_DEIVCE_CDC_ECM_DATA_BULK_OUT_EP_INTERVAL (0x00U)

#define USB_DEVICE_CDC_FUNC_LENGTH (0x05U)
#define USB_DEVICE_CDC_FUNC_TYPE_CS_INTERFACE (0x24U)
#define USB_DEVICE_CDC_FUNC_TYPE_CS_ENDPOINT (0x25U)
#define USB_DEVICE_CDC_FUNC_SUBTYPE_HEADER (0x00U)
#define USB_DEVICE_CDC_FUNC_SUBTYPE_UNION_FUNC (0x06U)
#define USB_DEVICE_CDC_FUNC_SUBTYPE_ETHERNET_NETWORKING (0x0FU)

#define USB_DEVICE_CDC_FUNC_UNION_LENGTH (0x05U)
#define USB_DEVICE_CDC_FUNC_UNION_TYPE (USB_DEVICE_CDC_FUNC_TYPE_CS_INTERFACE)
#define USB_DEVICE_CDC_FUNC_UNION_SUBTYPE (USB_DEVICE_CDC_FUNC_SUBTYPE_UNION_FUNC)
#define USB_DEVICE_CDC_FUNC_UNION_CTRL_INTERFACE (0x00U)

#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_LENGTH (0x0DU)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_TYPE (USB_DEVICE_CDC_FUNC_TYPE_CS_INTERFACE)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_SUBTYPE (USB_DEVICE_CDC_FUNC_SUBTYPE_ETHERNET_NETWORKING)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_MAC_ADDRESS_STRING_INDEX (USB_DEVICE_MAC_ADDRESS_STRING_INDEX)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_ETHERNET_STATISTICS (0x00000000U)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_MAX_SEGMENT_SIZE (0x05EAU)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_MULTICAST_FILTERS_NUMBER (0x0000U)
#define USB_DEVICE_CDC_ECM_CLASS_DESCRIPTOR_POWER_FILTERS_NUMBER (0x00U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed);

void USB_FillStringDescriptorBuffer(void);

usb_status_t USB_DeviceGetDescriptor(usb_device_handle handle, usb_setup_struct_t *setup, uint32_t *length, uint8_t **buffer);

usb_status_t USB_DeviceGetConfigure(usb_device_handle handle, uint8_t *configure);

usb_status_t USB_DeviceSetConfigure(usb_device_handle handle, uint8_t configure);

usb_status_t USB_DeviceGetInterface(usb_device_handle handle, uint8_t interface, uint8_t *alternateSetting);

usb_status_t USB_DeviceSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting);

usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status);

usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

#endif