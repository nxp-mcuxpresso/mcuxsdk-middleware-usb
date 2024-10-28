/*
 * Copyright 2024 NXP
 * All rights reserved.
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
product: Pins v15.0
processor: MIMXRT798S
package_id: MIMXRT798SGFOA
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
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
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: N4, peripheral: LP_FLEXCOMM0, signal: P0, pin_signal: PIO0_31, InputBufferEnable: enable}
  - {pin_num: N5, peripheral: LP_FLEXCOMM0, signal: P1, pin_signal: PIO1_0, InputBufferEnable: disable}
  - {pin_num: N6, peripheral: GPIO0, signal: 'IO, 17', pin_signal: PIO0_17}
  - {pin_num: L7, peripheral: GPIO0, signal: 'IO, 9', pin_signal: PIO0_9, InputBufferEnable: enable}  
  - {pin_num: J1, peripheral: SAI0, signal: RX_BCLK, pin_signal: PIO0_0, InputBufferEnable: enable}
  - {pin_num: J2, peripheral: SAI0, signal: RX_SYNC, pin_signal: PIO0_1, InputBufferEnable: enable}
  - {pin_num: J3, peripheral: SAI0, signal: RX_DATA, pin_signal: PIO0_2, InputBufferEnable: enable}
  - {pin_num: J4, peripheral: SAI0, signal: TX_BCLK, pin_signal: PIO0_3, InputBufferEnable: enable}
  - {pin_num: K4, peripheral: SAI0, signal: TX_DATA, pin_signal: PIO0_4, InputBufferEnable: enable}
  - {pin_num: K3, peripheral: SAI0, signal: TX_SYNC, pin_signal: PIO0_5, InputBufferEnable: enable}
  - {pin_num: R2, peripheral: LP_FLEXCOMM2, signal: P0, pin_signal: PIO1_11, OpenDrainEnable: enable, InputBufferEnable: enable}
  - {pin_num: R1, peripheral: LP_FLEXCOMM2, signal: P1, pin_signal: PIO1_12, OpenDrainEnable: enable, InputBufferEnable: enable}
  - {pin_num: M2, peripheral: CLKCTL0, signal: MCLK, pin_signal: PIO0_21, SelectsTransmitterCurrentDrive: O_33, InputBufferEnable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void BOARD_InitPins(void)
{
    /* Reset IOPCTL0 module */
    RESET_ClearPeripheralReset(kIOPCTL0_RST_SHIFT_RSTn);
    const uint32_t port0_pin17_config = (/* Pin is configured as GPIO0_IO17 */
                                         IOPCTL_PIO_FUNC0 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Disable input buffer function */
                                         IOPCTL_PIO_INBUF_DI |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI |
                                         /* Selects transmitter current drive 100ohm */
                                         IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN17 (coords: N6) is configured as GPIO0_IO17 */
    IOPCTL_PinMuxSet(0U, 17U, port0_pin17_config);

    const uint32_t port0_pin9_config = (/* Pin is configured as GPIO0_IO9 */
                                        IOPCTL_PIO_FUNC0 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN9 (coords: L7) is configured as GPIO0_IO9 */
    IOPCTL_PinMuxSet(0U, 9U, port0_pin9_config);	

    const uint32_t port0_pin0_config = (/* Pin is configured as SAI0_RX_BCLK */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN0 (coords: J1) is configured as SAI0_RX_BCLK */
    IOPCTL_PinMuxSet(0U, 0U, port0_pin0_config);

    const uint32_t port0_pin1_config = (/* Pin is configured as SAI0_RX_SYNC */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN1 (coords: J2) is configured as SAI0_RX_SYNC */
    IOPCTL_PinMuxSet(0U, 1U, port0_pin1_config);

    const uint32_t port0_pin2_config = (/* Pin is configured as SAI0_RX_DATA */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN2 (coords: J3) is configured as SAI0_RX_DATA */
    IOPCTL_PinMuxSet(0U, 2U, port0_pin2_config);

    const uint32_t port0_pin21_config = (/* Pin is configured as CLKCTL0_MCLK */
                                         IOPCTL_PIO_FUNC6 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI |
                                         /* Selects transmitter current drive 33ohm */
                                         IOPCTL_PIO_DRIVE_33OHM);
    /* PORT0 PIN21 (coords: M2) is configured as CLKCTL0_MCLK */
    IOPCTL_PinMuxSet(0U, 21U, port0_pin21_config);

    const uint32_t port0_pin3_config = (/* Pin is configured as SAI0_TX_BCLK */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN3 (coords: J4) is configured as SAI0_TX_BCLK */
    IOPCTL_PinMuxSet(0U, 3U, port0_pin3_config);

    const uint32_t port0_pin31_config = (/* Pin is configured as LP_FLEXCOMM0_P0 */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Pseudo Output Drain is disabled */
                                         IOPCTL_PIO_PSEDRAIN_DI |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI |
                                         /* Selects transmitter current drive 100ohm */
                                         IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN31 (coords: N4) is configured as LP_FLEXCOMM0_P0 */
    IOPCTL_PinMuxSet(0U, 31U, port0_pin31_config);

    const uint32_t port0_pin4_config = (/* Pin is configured as SAI0_TX_DATA */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN4 (coords: K4) is configured as SAI0_TX_DATA */
    IOPCTL_PinMuxSet(0U, 4U, port0_pin4_config);

    const uint32_t port0_pin5_config = (/* Pin is configured as SAI0_TX_SYNC */
                                        IOPCTL_PIO_FUNC5 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT0 PIN5 (coords: K3) is configured as SAI0_TX_SYNC */
    IOPCTL_PinMuxSet(0U, 5U, port0_pin5_config);

    const uint32_t port1_pin0_config = (/* Pin is configured as LP_FLEXCOMM0_P1 */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Disable input buffer function */
                                        IOPCTL_PIO_INBUF_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI |
                                        /* Selects transmitter current drive 100ohm */
                                        IOPCTL_PIO_DRIVE_100OHM);
    /* PORT1 PIN0 (coords: N5) is configured as LP_FLEXCOMM0_P1 */
    IOPCTL_PinMuxSet(1U, 0U, port1_pin0_config);

    const uint32_t port1_pin11_config = (/* Pin is configured as LP_FLEXCOMM2_P0 */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Pseudo Output Drain is enabled */
                                         IOPCTL_PIO_PSEDRAIN_EN |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI |
                                         /* Selects transmitter current drive 100ohm */
                                         IOPCTL_PIO_DRIVE_100OHM);
    /* PORT1 PIN11 (coords: R2) is configured as LP_FLEXCOMM2_P0 */
    IOPCTL_PinMuxSet(1U, 11U, port1_pin11_config);

    const uint32_t port1_pin12_config = (/* Pin is configured as LP_FLEXCOMM2_P1 */
                                         IOPCTL_PIO_FUNC1 |
                                         /* Disable pull-up / pull-down function */
                                         IOPCTL_PIO_PUPD_DI |
                                         /* Enable pull-down function */
                                         IOPCTL_PIO_PULLDOWN_EN |
                                         /* Enables input buffer function */
                                         IOPCTL_PIO_INBUF_EN |
                                         /* Pseudo Output Drain is enabled */
                                         IOPCTL_PIO_PSEDRAIN_EN |
                                         /* Input function is not inverted */
                                         IOPCTL_PIO_INV_DI |
                                         /* Selects transmitter current drive 100ohm */
                                         IOPCTL_PIO_DRIVE_100OHM);
    /* PORT1 PIN12 (coords: R1) is configured as LP_FLEXCOMM2_P1 */
    IOPCTL_PinMuxSet(1U, 12U, port1_pin12_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
