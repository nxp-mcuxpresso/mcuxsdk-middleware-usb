/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${standard_header:start}*/
#include <stdbool.h>
/*${standard_header:end}*/
/*${header:start}*/

#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "app.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_power.h"
/*${header:end}*/

extern usb_host_handle g_hostHandle;

/*${function:start}*/
void BOARD_InitHardware(void)
{
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* reset USB0 and USB1 device */
    RESET_PeripheralReset(kUSB0D_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB1D_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HMR_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HSL_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB1H_RST_SHIFT_RSTn);

    NVIC_ClearPendingIRQ(USB0_IRQn);
    NVIC_ClearPendingIRQ(USB0_NEEDCLK_IRQn);
    NVIC_ClearPendingIRQ(USB1_IRQn);
    NVIC_ClearPendingIRQ(USB1_NEEDCLK_IRQn);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*< Turn on USB Phy */
    POWER_DisablePD(kPDRUNCFG_PD_USB1_PHY); /*< Turn on USB Phy */
}
#if (defined(USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI > 0U))
void USB0_IRQHandler(void)
{
    USB_HostOhciIsrFunction(g_hostHandle);
}
#endif /* USB_HOST_CONFIG_OHCI */
#if (defined(USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS > 0U))
void USB1_IRQHandler(void)
{
    USB_HostIp3516HsIsrFunction(g_hostHandle);
}
#endif /* USB_HOST_CONFIG_IP3516HS */

void USB_HostClockInit(void)
{
#if ((defined USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI > 0U))
    CLOCK_EnableUsbfs0HostClock(kCLOCK_UsbSrcUsbPll, 48000000U);
#if ((defined FSL_FEATURE_USBFSH_USB_RAM) && (FSL_FEATURE_USBFSH_USB_RAM > 0U))
    for (int i = 0; i < (FSL_FEATURE_USBFSH_USB_RAM >> 2); i++)
    {
        ((uint32_t *)FSL_FEATURE_USBFSH_USB_RAM_BASE_ADDRESS)[i] = 0U;
    }
#endif
#endif /* USB_HOST_CONFIG_OHCI */

#if ((defined USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS > 0U))
    CLOCK_EnableUsbhs0HostClock(kCLOCK_UsbSrcUsbPll, 48000000U);
#if ((defined FSL_FEATURE_USBHSH_USB_RAM) && (FSL_FEATURE_USBHSH_USB_RAM > 0U))
    for (int i = 0; i < (FSL_FEATURE_USBHSH_USB_RAM >> 2); i++)
    {
        ((uint32_t *)FSL_FEATURE_USBHSH_USB_RAM_BASE_ADDRESS)[i] = 0U;
    }
#endif
#endif /* USB_HOST_CONFIG_IP3511HS */
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;
#if ((defined USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI > 0U))
    IRQn_Type usbHsIrqs[] = {(IRQn_Type)USB0_IRQn};
    irqNumber             = usbHsIrqs[CONTROLLER_ID - kUSB_ControllerOhci0];
#endif /* USB_HOST_CONFIG_OHCI */
#if ((defined USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS > 0U))
    IRQn_Type usbHsIrqs[] = {(IRQn_Type)USB1_IRQn};
    irqNumber             = usbHsIrqs[CONTROLLER_ID - kUSB_ControllerIp3516Hs0];
#endif /* USB_HOST_CONFIG_IP3511HS */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_HostTaskFn(void *param)
{
#if ((defined USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI > 0U))
    USB_HostOhciTaskFunction(param);
#endif /* USB_HOST_CONFIG_OHCI */
#if ((defined USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS > 0U))
    USB_HostIp3516HsTaskFunction(param);
#endif /* USB_HOST_CONFIG_IP3516HS */
}
/*${function:end}*/
