/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v10.0
processor: MIMXRT685S
package_id: MIMXRT685SFVKB
mcu_data: ksdk2_0
processor_version: 0.10.7
board: MIMXRT685-AUD-EVK
pin_labels:
- {pin_num: B1, pin_signal: PIO1_9/FC5_SSEL3/SCT0_GPI7/UTICK_CAP1/CTIMER1_MAT3/ADC0_12, label: WL_REG_ON, identifier: BOARD_INITPINS_WL_REG_ON;WL_REG_ON}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iopctl.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
    BOARD_InitCS42448Pins();
    BOARD_InitDebugConsolePins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitCS42448Pins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list:
  - {pin_num: K16, peripheral: SYSCON, signal: MCLK, pin_signal: PIO1_10/MCLK/FREQME_GPIO_CLK/CTIMER_INP10/CLKOUT, pupdena: disabled, pupdsel: pullDown, ibena: disabled,
    slew_rate: normal, drive: full, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: L3, peripheral: FLEXCOMM1, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_9/FC1_RXD_SDA_MOSI_DATA/SCT0_GPI6/SCT0_OUT6/CTIMER1_MAT2/I2S_BRIDGE_DATA_OUT/SEC_PIO0_9,
    pupdena: disabled, pupdsel: pullDown, ibena: enabled, slew_rate: normal, drive: full, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: K4, peripheral: FLEXCOMM1, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_8/FC1_TXD_SCL_MISO_WS/SCT0_GPI5/SCT0_OUT5/CTIMER1_MAT1/I2S_BRIDGE_WS_OUT/SEC_PIO0_8,
    pupdena: disabled, pupdsel: pullDown, ibena: enabled, slew_rate: normal, drive: full, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: J2, peripheral: FLEXCOMM1, signal: SCK, pin_signal: PIO0_7/FC1_SCK/SCT0_GPI4/SCT0_OUT4/CTIMER1_MAT0/I2S_BRIDGE_CLK_OUT/SEC_PIO0_7, pupdena: disabled,
    pupdsel: pullDown, ibena: enabled, slew_rate: normal, drive: full, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: C9, peripheral: FLEXCOMM3, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_23/FC3_RXD_SDA_MOSI_DATA/CTIMER3_MAT2/TRACEDATA(1)/SEC_PIO0_23, pupdsel: pullDown,
    ibena: enabled, slew_rate: normal, drive: full, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: C7, peripheral: FLEXCOMM3, signal: SCK, pin_signal: PIO0_21/FC3_SCK/CTIMER3_MAT0/TRACECLK/SEC_PIO0_21, ibena: enabled, drive: full}
  - {pin_num: D8, peripheral: FLEXCOMM3, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_22/FC3_TXD_SCL_MISO_WS/CTIMER3_MAT1/TRACEDATA(0)/SEC_PIO0_22, ibena: enabled, drive: full}
  - {pin_num: D6, peripheral: FLEXCOMM2, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_16/FC2_RXD_SDA_MOSI_DATA/SCT0_GPI2/SCT0_OUT2/CTIMER2_MAT2/I2S_BRIDGE_DATA_IN/SEC_PIO0_16,
    pupdena: enabled, pupdsel: pullUp, ibena: enabled, odena: enabled}
  - {pin_num: A5, peripheral: FLEXCOMM2, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_15/FC2_TXD_SCL_MISO_WS/SCT0_GPI1/SCT0_OUT1/CTIMER2_MAT1/I2S_BRIDGE_WS_IN/SEC_PIO0_15,
    pupdena: enabled, pupdsel: pullUp, ibena: enabled, odena: enabled}
  - {pin_num: G15, peripheral: GPIO, signal: 'PIO1, 1', pin_signal: PIO1_1/FC4_SSEL2/SCT0_GPI2/SCT0_OUT8/CTIMER1_MAT0, pupdena: disabled, pupdsel: pullDown, ibena: enabled,
    slew_rate: normal, drive: normal, amena: disabled, odena: disabled, iiena: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitCS42448Pins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitCS42448Pins(void)
{

    const uint32_t port0_pin15_config = (/* Pin is configured as FC2_TXD_SCL_MISO_WS */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Enable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_EN |
                                         /* Enable pull-up function */
                                         IOPCTL_PIO_PULLUP_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Normal drive */
                                         IOPCTL_PIO_FULLDRIVE_DI |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is enabled */
                                         IOPCTL_PIO_PSEDRAIN_EN |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT0 PIN15 (coords: A5) is configured as FC2_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 15U, port0_pin15_config);

    const uint32_t port0_pin16_config = (/* Pin is configured as FC2_RXD_SDA_MOSI_DATA */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Enable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_EN |
                                         /* Enable pull-up function */
                                         IOPCTL_PIO_PULLUP_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Normal drive */
                                         IOPCTL_PIO_FULLDRIVE_DI |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is enabled */
                                         IOPCTL_PIO_PSEDRAIN_EN |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT0 PIN16 (coords: D6) is configured as FC2_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 16U, port0_pin16_config);

    const uint32_t port0_pin21_config = (/* Pin is configured as FC3_SCK */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Full drive enable */
                                         IOPCTL_PIO_FULLDRIVE_EN |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT0 PIN21 (coords: C7) is configured as FC3_SCK */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 21U, port0_pin21_config);

    const uint32_t port0_pin22_config = (/* Pin is configured as FC3_TXD_SCL_MISO_WS */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Full drive enable */
                                         IOPCTL_PIO_FULLDRIVE_EN |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT0 PIN22 (coords: D8) is configured as FC3_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 22U, port0_pin22_config);

    const uint32_t port0_pin23_config = (/* Pin is configured as FC3_RXD_SDA_MOSI_DATA */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Full drive enable */
                                         IOPCTL_PIO_FULLDRIVE_EN |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT0 PIN23 (coords: C9) is configured as FC3_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 23U, port0_pin23_config);

    const uint32_t port0_pin7_config = (/* Pin is configured as FC1_SCK */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Full drive enable */
                                        IOPCTL_PIO_FULLDRIVE_EN |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN7 (coords: J2) is configured as FC1_SCK */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 7U, port0_pin7_config);

    const uint32_t port0_pin8_config = (/* Pin is configured as FC1_TXD_SCL_MISO_WS */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Full drive enable */
                                        IOPCTL_PIO_FULLDRIVE_EN |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN8 (coords: K4) is configured as FC1_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 8U, port0_pin8_config);

    const uint32_t port0_pin9_config = (/* Pin is configured as FC1_RXD_SDA_MOSI_DATA */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Full drive enable */
                                        IOPCTL_PIO_FULLDRIVE_EN |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN9 (coords: L3) is configured as FC1_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 9U, port0_pin9_config);

    const uint32_t SW1 = (/* Pin is configured as PIO1_1 */
                          IOPCTL_PIO_FUNC0 |
                          /* Disable pull-up / pull-down function */
                          IOPCTL_PIO_PUPD_DI |
                          /* Enable pull-down function */
                          IOPCTL_PIO_PULLDOWN_EN |
                          /* Enables input buffer function */
                          IOPCTL_PIO_INBUF_EN |
                          /* Normal mode */
                          IOPCTL_PIO_SLEW_RATE_NORMAL |
                          /* Normal drive */
                          IOPCTL_PIO_FULLDRIVE_DI |
                          /* Analog mux is disabled */
                          IOPCTL_PIO_ANAMUX_DI |
                          /* Pseudo Output Drain is disabled */
                          IOPCTL_PIO_PSEDRAIN_DI |
                          /* Input function is not inverted */
                          IOPCTL_PIO_INV_DI);
    /* PORT1 PIN1 (coords: G15) is configured as PIO1_1 */
    IOPCTL_PinMuxSet(IOPCTL, BOARD_INITPINS_SW1_PORT, BOARD_INITPINS_SW1_PIN, SW1);

    const uint32_t port1_pin10_config = (/* Pin is configured as MCLK */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Disable input buffer function */
                                         IOPCTL_PIO_INBUF_DI |
                                         /* Normal mode */
                                         IOPCTL_PIO_SLEW_RATE_NORMAL |
                                         /* Full drive enable */
                                         IOPCTL_PIO_FULLDRIVE_EN |
                                         /* Analog mux is disabled */
                                         IOPCTL_PIO_ANAMUX_DI |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI);
    /* PORT1 PIN10 (coords: K16) is configured as MCLK */
    IOPCTL_PinMuxSet(IOPCTL, 1U, 10U, port1_pin10_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitDebugConsolePins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list:
  - {pin_num: G4, peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_2/FC0_RXD_SDA_MOSI_DATA/CTIMER0_MAT2/I2S_BRIDGE_DATA_IN/SEC_PIO0_2, pupdena: disabled,
    pupdsel: pullDown, ibena: enabled, slew_rate: normal, drive: normal, amena: disabled, odena: disabled, iiena: disabled}
  - {pin_num: G2, peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_1/FC0_TXD_SCL_MISO_WS/CTIMER0_MAT1/I2S_BRIDGE_WS_IN/SEC_PIO0_1, pupdena: disabled,
    pupdsel: pullDown, ibena: disabled, slew_rate: normal, drive: normal, amena: disabled, odena: disabled, iiena: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitDebugConsolePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitDebugConsolePins(void)
{

    const uint32_t port0_pin1_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Disable input buffer function */
                                        IOPCTL_PIO_INBUF_DI |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN1 (coords: G2) is configured as FC0_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 1U, port0_pin1_config);

    const uint32_t port0_pin2_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN2 (coords: G4) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 2U, port0_pin2_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/