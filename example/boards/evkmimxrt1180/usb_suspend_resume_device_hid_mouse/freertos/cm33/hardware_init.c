/*
 * Copyright 2022 - 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${standard_header:start}*/
#include <stdbool.h>
/*${standard_header:end}*/
/*${header:start}*/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_hid.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "fsl_device_registers.h"
#include "mouse.h"
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "mouse.h"
#include "pin_mux.h"
#include "usb_phy.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_lpit.h"
#include "fsl_gpc.h"
#include "fsl_adapter_timer.h"
/*${header:end}*/
/*${variable:start}*/
#define TIMER_SOURCE_CLOCK CLOCK_GetRootClockFreq(kCLOCK_Root_Bus_Aon)
extern usb_hid_mouse_struct_t g_UsbDeviceHidMouse;
uint32_t g_halTimerHandle[(HAL_TIMER_HANDLE_SIZE + 3) / 4];
static uint32_t systemTickControl;
/*${variable:end}*/
/*${prototype:start}*/
void USB_WaitClockLocked(void);
/*${prototype:end}*/
extern usb_hid_mouse_struct_t g_UsbDeviceHidMouse;
/*${function:start}*/
void BOARD_InitHardware(void)
{
    BOARD_ConfigMPU();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Define the init structure for the input switch pin */
    rgpio_pin_config_t sw_config = {
        kRGPIO_DigitalInput,
        0,
    };

    /* Workaround: Disable interrupt which might be enabled by ROM. */
    RGPIO_SetPinInterruptConfig(RGPIO1, 9U, kRGPIO_InterruptOutput0, kRGPIO_InterruptOrDMADisabled);
    NVIC_ClearPendingIRQ(GPIO1_0_IRQn);

    /* Init input switch GPIO. */
    RGPIO_SetPinInterruptConfig(BOARD_USER_BUTTON_GPIO, BOARD_USER_BUTTON_GPIO_PIN, kRGPIO_InterruptOutput0,
                                kRGPIO_InterruptFallingEdge);

    EnableIRQ(BOARD_USER_BUTTON_IRQ);
    RGPIO_PinInit(BOARD_USER_BUTTON_GPIO, BOARD_USER_BUTTON_GPIO_PIN, &sw_config);

    GPC_AssignCpuDomain(kGPC_CPU0, kGPC_Domain0);
    GPC_CM_EnableIrqWakeup(kGPC_CPU0, BOARD_USER_BUTTON_IRQ, false);
    GPC_CM_EnableIrqWakeup(kGPC_CPU0, BOARD_USER_BUTTON_IRQ, true);
}

/*!
 * @brief De-initialize all pins used in this example
 *
 * @param disablePortClockAfterInit disable port clock after pin
 * initialization or not.
 */
void BOARD_DeinitPins(void)
{
}
void BOARD_USER_BUTTON_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    RGPIO_ClearPinsInterruptFlags(BOARD_USER_BUTTON_GPIO, kRGPIO_InterruptOutput0, 1U << BOARD_USER_BUTTON_GPIO_PIN);
    g_UsbDeviceHidMouse.selfWakeup = 1U;
    SDK_ISR_EXIT_BARRIER;
}
void SW_IntControl(uint8_t enable)
{
}
void SW_Callback(void)
{
    g_UsbDeviceHidMouse.selfWakeup = 1U;
    SW_IntControl(0);
}
void SW_Init(void)
{
    NVIC_SetPriority(BOARD_USER_BUTTON_IRQ, 1U);
    NVIC_EnableIRQ(BOARD_USER_BUTTON_IRQ);
}
char *SW_GetName(void)
{
    return BOARD_USER_BUTTON_NAME;
}
void HW_TimerCallback(void *param)
{
    g_UsbDeviceHidMouse.hwTick++;
    USB_DeviceUpdateHwTick(g_UsbDeviceHidMouse.deviceHandle, g_UsbDeviceHidMouse.hwTick);
}
void HW_TimerInit(void)
{
    hal_timer_config_t halTimerConfig;
    halTimerConfig.timeout            = 1000;
    halTimerConfig.srcClock_Hz        = TIMER_SOURCE_CLOCK;
    halTimerConfig.instance           = 1U;
    hal_timer_handle_t halTimerHandle = &g_halTimerHandle[0];
    HAL_TimerInit(halTimerHandle, &halTimerConfig);
    HAL_TimerInstallCallback(halTimerHandle, HW_TimerCallback, NULL);
}
void HW_TimerControl(uint8_t enable)
{
    if (enable)
    {
        HAL_TimerEnable(g_halTimerHandle);
    }
    else
    {
        HAL_TimerDisable(g_halTimerHandle);
    }
}
void USB_LowpowerModeInit(void)
{
#if (defined(FSL_FEATURE_SIM_OPT_HAS_USB_PHY) && (FSL_FEATURE_SIM_OPT_HAS_USB_PHY > 0))
    SIM->SOPT2 |= SIM_SOPT2_USBSLSRC_MASK;
#endif
    SW_Init();
    HW_TimerInit();
}
void USB_PreLowpowerMode(void)
{
    if (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
    {
        systemTickControl = SysTick->CTRL;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }
    __disable_irq();
}
/*
 * Execute the instrument to enter low power.
 */
static void stop(void)
{
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __asm("WFI");
}
/*
 * Enter the LowPower mode.
 */
void APP_LowPower_EnterLowPower(void)
{
    GPC_CM_SetNextCpuMode(kGPC_CPU0, kGPC_StopMode);
    stop();
}
uint8_t USB_EnterLowpowerMode(void)
{
    APP_LowPower_EnterLowPower();
    return 0;
}
void USB_WaitClockLocked(void)
{
}
void USB_PostLowpowerMode(void)
{
    __enable_irq();
    SysTick->CTRL = systemTickControl;
    USB_WaitClockLocked();
}
void USB_ControllerSuspended(void)
{
}

void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_UsbDeviceHidMouse.deviceHandle);
}

void USB_OTG2_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_UsbDeviceHidMouse.deviceHandle);
}

void USB_DeviceClockInit(void)
{
    uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
    usbClockFreq = 24000000;
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    USB_EhciLowPowerPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber                  = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
    USB_DeviceEhciTaskFunction(deviceHandle);
}
#endif
/*${function:end}*/
