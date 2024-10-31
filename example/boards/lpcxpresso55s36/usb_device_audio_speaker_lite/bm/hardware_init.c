/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${header:start}*/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_ch9.h"
#include "usb_audio_config.h"
#include "usb_device_descriptor.h"
#include "usb_device_audio.h"
#include "audio_speaker.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "app.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_adapter_audio.h"
#include "fsl_codec_common.h"
#include "fsl_wm8904.h"
#include "fsl_codec_adapter.h"
#include "fsl_power.h"
#include "fsl_debug_console.h"
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#include "fsl_ctimer.h"
#endif
/*${header:end}*/

/*${prototype:start}*/
extern uint32_t USB_AudioSpeakerBufferSpaceUsed(void);
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
void CTIMER_SOF_TOGGLE_HANDLER_FRO(uint32_t i);
#endif
void CTIMER_SOF_TOGGLE_HANDLER_PLL(uint32_t i);
#else
extern void USB_DeviceCalculateFeedback(void);
#endif
extern void WM8904_USB_Audio_Init(void *I2CBase);
void WM8904_Config_Audio_Formats(uint32_t samplingRate);
/*${prototype:end}*/

/*${variable:start}*/
extern usb_audio_speaker_struct_t g_UsbDeviceAudioSpeaker;
extern uint8_t audioPlayDataBuff[AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT_NORMAL * AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME];
hal_audio_transfer_t s_TxTransfer;
HAL_AUDIO_HANDLE_DEFINE(audioTxHandle);
hal_audio_config_t audioTxConfig;
hal_audio_dma_config_t dmaTxConfig;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t audioPlayDMATempBuff[AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME];
codec_handle_t codecHandle;

wm8904_config_t wm8904Config = {
    .i2cConfig    = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .recordSource = kWM8904_RecordSourceLineInput,
    .recordChannelLeft  = kWM8904_RecordChannelLeft2,
    .recordChannelRight = kWM8904_RecordChannelRight2,
    .playSource         = kWM8904_PlaySourceDAC,
    .slaveAddress       = WM8904_I2C_ADDRESS,
    .protocol           = kWM8904_ProtocolI2S,
    .format             = {.sampleRate = kWM8904_SampleRate48kHz, .bitWidth = kWM8904_BitWidth16},
    .mclk_HZ            = DEMO_I2S_MASTER_CLOCK_FREQUENCY,
    .master             = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8904, .codecDevConfig = &wm8904Config};

#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
static ctimer_config_t ctimerInfoFro;
ctimer_callback_t *cb_func_fro[] = {(ctimer_callback_t *)CTIMER_SOF_TOGGLE_HANDLER_FRO};
#endif
ctimer_callback_t *cb_func_pll[] = {(ctimer_callback_t *)CTIMER_SOF_TOGGLE_HANDLER_PLL};
static ctimer_config_t ctimerInfoPll;
#endif
/*${variable:end}*/

/*${function:start}*/
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
void audio_fro_trim_up(void)
{
    static uint8_t usbClkAdj = 0;
    if (0U == usbClkAdj)
    {
        /* USBCLKADJ is turned off, start using software adjustment */
        ANACTRL->FRO192M_CTRL = (ANACTRL->FRO192M_CTRL & ~(ANACTRL_FRO192M_CTRL_USBCLKADJ_MASK));
        usbClkAdj             = 1U;
    }
    uint32_t val          = ANACTRL->FRO192M_CTRL;
    val                   = (val & ~(0xff << 16)) | ((((val >> 16) & 0xFF) + 1) << 16) | (1UL << 31);
    ANACTRL->FRO192M_CTRL = val;
}

void audio_fro_trim_down(void)
{
    static uint8_t usbClkAdj = 0;
    if (0U == usbClkAdj)
    {
        /* USBCLKADJ is turned off, start using software adjustment */
        ANACTRL->FRO192M_CTRL = (ANACTRL->FRO192M_CTRL & ~(ANACTRL_FRO192M_CTRL_USBCLKADJ_MASK));
        usbClkAdj             = 1U;
    }
    uint32_t val          = ANACTRL->FRO192M_CTRL;
    ANACTRL->FRO192M_CTRL = (val & ~(0xff << 16)) | ((((val >> 16) & 0xFF) - 1) << 16) | (1UL << 31);
}
#endif

void BOARD_DMA_EDMA_Set_AudioFormat(void)
{
    return;
}

void BOARD_DMA_EDMA_Enable_Audio_Interrupts(void)
{
    return;
}

void APP_InitDebugConsole(void)
{
    /* attach 12 MHz clock to FLEXCOMM6 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM6);

    RESET_ClearPeripheralReset(kFC6_RST_SHIFT_RSTn);

    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

    DbgConsole_Init(6U, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOARD_InitHardware(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    /* USART clock */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom6Clk, 0u, false);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom6Clk, 1u, true);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM6);
    /* I2C clock */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom7Clk, 0U, true);  /*!< Reset FCCLKDIV7 divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom7Clk, 1U, false); /*!< Set FCCLKDIV7 divider to value 1 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
    /*!< Configure XTAL32M */
    POWER_DisablePD(kPDRUNCFG_PD_XTALHF);                   /* Ensure XTAL32M is powered */
    POWER_DisablePD(kPDRUNCFG_PD_LDOXTALHF);                /* Ensure XTAL32M is powered */
    CLOCK_SetupExtClocking(16000000U);                      /* Enable clk_in clock */
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK; /*!< Ensure CLK_IN is on  */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_SYSTEM_CLK_OUT_MASK;

    /*!< Set up PLL, use FRO for full speed */
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
    CLOCK_AttachClk(kFRO12M_to_PLL0);   /*!< Switch PLL0CLKSEL to FRO12M */
    POWER_DisablePD(kPDRUNCFG_PD_PLL0); /* Ensure PLL is on  */
    POWER_DisablePD(kPDRUNCFG_PD_PLL0_SSCG);
    const pll_setup_t pll0Setup = {
        .pllctrl = SYSCON_PLL0CTRL_LIMUPOFF_MASK | SYSCON_PLL0CTRL_CLKEN_MASK | SYSCON_PLL0CTRL_SELI(4U) |
                   SYSCON_PLL0CTRL_SELP(3U),
        .pllndec = SYSCON_PLL0NDEC_NDIV(3U),
        .pllpdec = SYSCON_PLL0PDEC_PDIV(8U),
        .pllsscg = {(SYSCON_PLL0SSCG0_MD_LBS(3298534883U)), (SYSCON_PLL0SSCG1_MD_MBS(0U) | (uint32_t)(kSS_MF_512) |
                                                             (uint32_t)(kSS_MR_K0) | (uint32_t)(kSS_MC_NOC))},
        .pllRate = 24576000U,
        .flags   = PLL_SETUPFLAG_POWERUP};
#endif

    /*!< Configure PLL to the desired values */
    CLOCK_SetPLL0Freq(&pll0Setup);

    CLOCK_SetClkDiv(kCLOCK_DivPllClk, 0U, true);
    CLOCK_SetClkDiv(kCLOCK_DivPllClk, 1U, false);
    CLOCK_SetClkDiv(kCLOCK_DivFlexFrg0, 0U, false);
    CLOCK_SetClkDiv(kCLOCK_DivFlexFrg3, 0U, false);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom0Clk, 0U, true);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom0Clk, 1U, false);

    /* I2S clocks */
    CLOCK_AttachClk(kPLL0_to_PLLCLKDIV);             /*!< Switch PLLCLKDIV to PLL0 */
    CLOCK_AttachClk(kPLL_CLK_DIV_FRG0_to_FLEXCOMM0); /*!< Switch FLEXCOMM0 to PLL_CLK_DIV_FRG0 */

    /* Attach PLL clock to MCLK for I2S, no divider */
    CLOCK_SetClkDiv(kCLOCK_DivMclk, 0U, true);
    CLOCK_SetClkDiv(kCLOCK_DivMclk, 1U, false);
    CLOCK_AttachClk(kPLL0_to_MCLK);
    SYSCON->MCLKIO = 1U;

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for DMA0 */
    RESET_PeripheralReset(kDMA0_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for I2S */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);

    /* reset NVIC for FLEXCOMM0 */
    NVIC_ClearPendingIRQ(FLEXCOMM0_IRQn);

    /* Enable interrupts for I2S */
    EnableIRQ(FLEXCOMM0_IRQn);

    /* Initialize the rest */
    BOARD_InitBootPins();
    BOARD_BootClockFROHF96M();
    APP_InitDebugConsole();

    NVIC_ClearPendingIRQ(USB0_IRQn);
    NVIC_ClearPendingIRQ(USB0_NEEDCLK_IRQn);
    POWER_DisablePD(kPDRUNCFG_PD_USBFSPHY); /*< Turn on USB0 Phy */

    /* reset the IP to make sure it's in reset state. */
    RESET_PeripheralReset(kUSB0_DEV_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HSL_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HMR_RST_SHIFT_RSTn);

#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
    POWER_DisablePD(kPDRUNCFG_PD_USBFSPHY); /*< Turn on USB Phy */
    CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 1, false);
    CLOCK_AttachClk(kFRO_HF_to_USB0);
    /* enable usb0 host clock */
    CLOCK_EnableClock(kCLOCK_Usbhsl0);
    /*According to reference mannual, device mode setting has to be set by access usb host register */
    *((uint32_t *)(USBFSH_BASE + 0x5C)) |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
    /* disable usb0 host clock */
    CLOCK_DisableClock(kCLOCK_Usbhsl0);
#endif
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 0U, true);  /*!< Reset CTIMER0CLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1U, false); /*!< Set CTIMER0CLKDIV divider to value 1 */
    CLOCK_AttachClk(kPLL0_to_CTIMER0);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 0U, true);  /*!< Reset CTIMER1CLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 1U, false); /*!< Set CTIMER1CLKDIV divider to value 1 */
    CLOCK_AttachClk(kFRO_HF_to_CTIMER1);
#endif
    dmaTxConfig.instance            = DEMO_DMA_INSTANCE_INDEX;
    dmaTxConfig.channel             = DEMO_I2S_TX_CHANNEL;
    dmaTxConfig.priority            = kHAL_AudioDmaChannelPriority3;
    audioTxConfig.dmaConfig         = &dmaTxConfig;
    audioTxConfig.instance          = DEMO_I2S_TX_INSTANCE_INDEX;
    audioTxConfig.srcClock_Hz       = CLOCK_GetPll0OutFreq();
    audioTxConfig.sampleRate_Hz     = 48000;
    audioTxConfig.masterSlave       = kHAL_AudioMaster;
    audioTxConfig.bclkPolarity      = kHAL_AudioSampleOnRisingEdge;
    audioTxConfig.frameSyncPolarity = kHAL_AudioBeginAtFallingEdge;
    audioTxConfig.frameSyncWidth    = kHAL_AudioFrameSyncWidthHalfFrame;
    audioTxConfig.dataFormat        = kHAL_AudioDataFormatI2sClassic;
    audioTxConfig.bitWidth          = (uint8_t)kHAL_AudioWordWidth16bits;
    audioTxConfig.lineChannels      = kHAL_AudioStereo;
}

void BOARD_Codec_Init()
{
    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
    {
        assert(false);
    }
    if (CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, 0x0020) !=
        kStatus_Success)
    {
        assert(false);
    }
}

static void TxCallback(hal_audio_handle_t handle, hal_audio_status_t completionStatus, void *callbackParam)
{
    uint32_t audioSpeakerPreReadDataCount = 0U;
    uint32_t preAudioSendCount            = 0U;

    if ((USB_AudioSpeakerBufferSpaceUsed() < (g_UsbDeviceAudioSpeaker.audioPlayTransferSize)) &&
        (g_UsbDeviceAudioSpeaker.startPlayFlag == 1U))
    {
        g_UsbDeviceAudioSpeaker.startPlayFlag          = 0;
        g_UsbDeviceAudioSpeaker.speakerDetachOrNoInput = 1;
    }
    if (0U != g_UsbDeviceAudioSpeaker.startPlayFlag)
    {
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
        USB_DeviceCalculateFeedback();
#endif
        s_TxTransfer.dataSize = g_UsbDeviceAudioSpeaker.audioPlayTransferSize;
        s_TxTransfer.data     = audioPlayDataBuff + g_UsbDeviceAudioSpeaker.tdReadNumberPlay;
        preAudioSendCount     = g_UsbDeviceAudioSpeaker.audioSendCount[0];
        g_UsbDeviceAudioSpeaker.audioSendCount[0] += g_UsbDeviceAudioSpeaker.audioPlayTransferSize;
        if (preAudioSendCount > g_UsbDeviceAudioSpeaker.audioSendCount[0])
        {
            g_UsbDeviceAudioSpeaker.audioSendCount[1] += 1U;
        }
        g_UsbDeviceAudioSpeaker.audioSendTimes++;
        g_UsbDeviceAudioSpeaker.tdReadNumberPlay += g_UsbDeviceAudioSpeaker.audioPlayTransferSize;
        if (g_UsbDeviceAudioSpeaker.tdReadNumberPlay >= g_UsbDeviceAudioSpeaker.audioPlayBufferSize)
        {
            g_UsbDeviceAudioSpeaker.tdReadNumberPlay = 0;
        }
        audioSpeakerPreReadDataCount = g_UsbDeviceAudioSpeaker.audioSpeakerReadDataCount[0];
        g_UsbDeviceAudioSpeaker.audioSpeakerReadDataCount[0] += g_UsbDeviceAudioSpeaker.audioPlayTransferSize;
        if (audioSpeakerPreReadDataCount > g_UsbDeviceAudioSpeaker.audioSpeakerReadDataCount[0])
        {
            g_UsbDeviceAudioSpeaker.audioSpeakerReadDataCount[1] += 1U;
        }
    }
    else
    {
        if (0U != g_UsbDeviceAudioSpeaker.audioPlayTransferSize)
        {
            s_TxTransfer.dataSize = g_UsbDeviceAudioSpeaker.audioPlayTransferSize;
        }
        else
        {
            s_TxTransfer.dataSize = HS_ISO_OUT_ENDP_PACKET_SIZE;
        }
        s_TxTransfer.data = audioPlayDMATempBuff;
    }
    HAL_AudioTransferSendNonBlocking(handle, &s_TxTransfer);
}

void AUDIO_DMA_EDMA_Start()
{
    usb_echo("Init Audio I2S and CODEC\r\n");
    s_TxTransfer.dataSize = HS_ISO_OUT_ENDP_PACKET_SIZE;
    s_TxTransfer.data     = audioPlayDMATempBuff;
    HAL_AudioTxInstallCallback(&audioTxHandle[0], TxCallback, (void *)&s_TxTransfer);
    HAL_AudioTransferSendNonBlocking((hal_audio_handle_t)&audioTxHandle[0], &s_TxTransfer);
}

#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
void USB_AudioPllChange(void)
{
    SYSCON->PLL0SSCG0 = g_UsbDeviceAudioSpeaker.curAudioPllFrac;
}

void CTIMER_CaptureInit(void)
{
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
    INPUTMUX->TIMER0CAP[0] = 0x14U; /* 0x15U for USB1 and 0x14U for USB0. */
#endif
    CTIMER_GetDefaultConfig(&ctimerInfoPll);

    /* Initialize CTimer module */
    CTIMER_Init(CTIMER0, &ctimerInfoPll);

    CTIMER_SetupCapture(CTIMER0, kCTIMER_Capture_0, kCTIMER_Capture_RiseEdge, true);

    CTIMER_RegisterCallBack(CTIMER0, (ctimer_callback_t *)&cb_func_pll[0], kCTIMER_SingleCallback);

    /* Start the L counter */
    CTIMER_StartTimer(CTIMER0);

    /* if full speed controller, use another ctimer */
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
    INPUTMUX->TIMER1CAP[0] = 0x14U; /* 0x15U for USB1 and 0x14U for USB0. */

    CTIMER_GetDefaultConfig(&ctimerInfoFro);

    CTIMER_Init(CTIMER1, &ctimerInfoFro);

    CTIMER_SetupCapture(CTIMER1, kCTIMER_Capture_0, kCTIMER_Capture_RiseEdge, true);

    CTIMER_RegisterCallBack(CTIMER1, (ctimer_callback_t *)&cb_func_fro[0], kCTIMER_SingleCallback);

    /* Start the L counter */
    CTIMER_StartTimer(CTIMER1);
#endif
}
#endif

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
void USB0_IRQHandler(void)
{
    USB_DeviceLpcIp3511IsrFunction(g_UsbDeviceAudioSpeaker.deviceHandle);
}
#endif
void USB_DeviceClockInit(void)
{
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
    /* enable USB IP clock */
    CLOCK_EnableUsbfs0DeviceClock(kCLOCK_UsbfsSrcFro, CLOCK_GetFroHfFreq());
    ANACTRL->FRO192M_CTRL = (ANACTRL->FRO192M_CTRL & ~(ANACTRL_FRO192M_CTRL_USBCLKADJ_MASK));
#endif
}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
    uint8_t usbDeviceIP3511Irq[] = USB_IRQS;
    irqNumber                    = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Fs0];
#endif
    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
    USB_DeviceLpcIp3511TaskFunction(deviceHandle);
#endif
}
#endif
/*${function:end}*/