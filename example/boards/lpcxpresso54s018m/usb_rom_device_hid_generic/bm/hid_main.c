/*
 * @brief HID generic example
 *
 * @note
 * Copyright  2013, NXP
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <stdio.h>
#include <string.h>
#include "app_usbd_cfg.h"
#include "usbd_rom_api.h"

#include "hid_generic.h"
#include "romapi_5460x.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_fro_calib.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* USB descriptor arrays defined *_desc.c file */
extern const uint8_t USB_DeviceDescriptor[];
extern uint8_t USB_HsConfigDescriptor[];
extern uint8_t USB_FsConfigDescriptor[];
extern const uint8_t USB_StringDescriptor[];
extern const uint8_t USB_DeviceQualifier[];
/**
 * @brief    Find the address of interface descriptor for given class type.
 * @param    pDesc: Pointer to configuration descriptor in which the desired class
 *            interface descriptor to be found.
 * @param    intfClass: Interface class type to be searched.
 * @return   If found returns the address of requested interface else returns NULL.
 */
extern USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass);
void BOARD_InitHardware(void);
void HW_TimerInit(void);

static USBD_HANDLE_T g_hUsb;
ALIGN(2048) uint8_t g_memUsbStack[USB_STACK_MEM_SIZE];

/*******************************************************************************
 * Variables
 ******************************************************************************/

const USBD_API_T *g_pUsbApi;

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief	Handle interrupt from USB
 * @return	Nothing
 */
void USB0_IRQHandler(void)
{
    USBD_API->hw->EnableEvent(g_hUsb, 0, USB_EVT_SOF, 1);
    USBD_API->hw->ISR(g_hUsb);
}

/**
 * @brief	Find the address of interface descriptor for given class type.
 * @return	If found returns the address of requested interface else returns NULL.
 */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
    USB_COMMON_DESCRIPTOR *pD;
    USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
    uint32_t next_desc_adr;

    pD            = (USB_COMMON_DESCRIPTOR *)pDesc;
    next_desc_adr = (uint32_t)pDesc;

    while (pD->bLength)
    {
        /* is it interface descriptor */
        if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE)
        {
            pIntfDesc = (USB_INTERFACE_DESCRIPTOR *)pD;
            /* did we find the right interface descriptor */
            if (pIntfDesc->bInterfaceClass == intfClass)
            {
                break;
            }
        }
        pIntfDesc     = 0;
        next_desc_adr = (uint32_t)pD + pD->bLength;
        pD            = (USB_COMMON_DESCRIPTOR *)next_desc_adr;
    }

    return pIntfDesc;
}

static void USB_DeviceApplicationInit(void)
{
    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
    ErrorCode_t ret = LPC_OK;

    /* initialize USBD ROM API pointer. */
    g_pUsbApi = (const USBD_API_T *)LPC_ROM_API->usbdApiBase;

    /* initialize call back structures */
    memset((void *)&usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));

    usb_param.usb_reg_base  = USB0_BASE;
    usb_param.max_num_ep    = 2;
    usb_param.mem_base      = (uint32_t)&g_memUsbStack;
    usb_param.mem_size      = USB_STACK_MEM_SIZE;
    usb_param.double_buffer = FALSE;
    usb_param.USB_SOF_Event = USB_SOF_Event;

    /* Set the USB descriptors */
    desc.device_desc = (uint8_t *)&USB_DeviceDescriptor[0];
    desc.string_desc = (uint8_t *)&USB_StringDescriptor[0];
    /* Note, to pass USBCV test full-speed only devices should have both
       descriptor arrays point to same location and device_qualifier set to 0.
     */
    usb_param.high_speed_capable = FALSE;
    desc.high_speed_desc         = (uint8_t *)&USB_FsConfigDescriptor[0];
    desc.full_speed_desc         = (uint8_t *)&USB_FsConfigDescriptor[0];
    desc.device_qualifier        = 0;

    /* USB Initialization */
    ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
    if (ret == LPC_OK)
    {
        /* WORKAROUND for artf32219 ROM driver BUG:
          The mem_base parameter part of USB_param structure returned
          by Init() routine is not accurate causing memory allocation issues for
          further components.
        */
        usb_param.mem_base = (uint32_t)&g_memUsbStack + (USB_STACK_MEM_SIZE - usb_param.mem_size);

        ret = usb_hid_init(g_hUsb,
                           (USB_INTERFACE_DESCRIPTOR *)&USB_FsConfigDescriptor[sizeof(USB_CONFIGURATION_DESCRIPTOR)],
                           &usb_param.mem_base, &usb_param.mem_size);
        if (ret == LPC_OK)
        {
            /* Install isr, set priority, and enable IRQ. */
            NVIC_SetPriority(USB0_IRQn, USB_DEVICE_INTERRUPT_PRIORITY);
            /*  enable USB interrupts */
            NVIC_EnableIRQ(USB0_IRQn);
            /* now connect */
            USBD_API->hw->Connect(g_hUsb, 1);
        }
    }
    if (LPC_OK == ret)
    {
        PRINTF("USB HID generic example!\r\n");
    }
    else
    {
        PRINTF("USB HID generic initialization failed!\r\n");
        return;
    }
}

/**
 * @brief	main routine for USB device example
 * @return	Function should not exit.
 */
int main(void)
{
    SystemCoreClockUpdate();
    BOARD_InitHardware();
    HW_TimerInit();
    USB_DeviceApplicationInit();

    while (1)
    {
        /* Sleep until next IRQ happens */
        __WFI();
    }
}
