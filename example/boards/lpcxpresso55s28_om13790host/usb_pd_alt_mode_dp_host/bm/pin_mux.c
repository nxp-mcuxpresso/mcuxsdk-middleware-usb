/*
 * Copyright 2017-2019 NXP
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
product: Pins v6.0
processor: LPC55S28
package_id: LPC55S28JBD100
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_iocon.h"
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
  - {pin_num: '92', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '94', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, mode: inactive,
    slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '24', peripheral: GPIO, signal: 'PIO1, 8', pin_signal: PIO1_8/FC0_CTS_SDA_SSEL0/SD0_CLK/SCT0_OUT1/FC4_SSEL2/ADC0_4, mode: pullUp, slew_rate: standard,
    invert: disabled, open_drain: disabled, asw: disabled}
  - {pin_num: '9', peripheral: GPIO, signal: 'PIO1, 7', pin_signal: PIO1_7/FC0_RTS_SCL_SSEL1/SD0_D1/CTIMER2_MAT2/SCT_GPI4, mode: pullUp, slew_rate: standard, invert: disabled,
    open_drain: disabled}
  - {pin_num: '40', peripheral: GPIO, signal: 'PIO1, 10', pin_signal: PIO1_10/FC1_RXD_SDA_MOSI_DATA/CTIMER1_MAT0/SCT0_OUT3, mode: pullUp, slew_rate: standard, invert: disabled,
    open_drain: disabled}
  - {pin_num: '1', peripheral: GPIO, signal: 'PIO1, 4', pin_signal: PIO1_4/FC0_SCK/SD0_D0/CTIMER2_MAT1/SCT0_OUT0/FREQME_GPIO_CLK_A, mode: pullUp, slew_rate: standard,
    invert: disabled, open_drain: disabled}
  - {pin_num: '31', peripheral: GPIO, signal: 'PIO1, 5', pin_signal: PIO1_5/FC0_RXD_SDA_MOSI_DATA/SD0_D2/CTIMER2_MAT0/SCT_GPI0, mode: pullUp, slew_rate: standard,
    invert: disabled, open_drain: disabled}
  - {pin_num: '5', peripheral: GPIO, signal: 'PIO1, 6', pin_signal: PIO1_6/FC0_TXD_SCL_MISO_WS/SD0_D3/CTIMER2_MAT1/SCT_GPI3, mode: pullUp, slew_rate: standard, invert: disabled,
    open_drain: disabled}
  - {pin_num: '22', peripheral: GPIO, signal: 'PIO0, 15', pin_signal: PIO0_15/FC6_CTS_SDA_SSEL0/UTICK_CAP2/CT_INP16/SCT0_OUT2/SD0_WR_PRT/SECURE_GPIO0_15/ADC0_2, mode: pullUp,
    slew_rate: standard, invert: disabled, open_drain: disabled, asw: enabled}
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
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin15_config = (/* Pin is configured as PIO0_15 */
                                         IOCON_PIO_FUNC0 |
                                         /* Selects pull-up function */
                                         IOCON_PIO_MODE_PULLUP |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* Analog switch is closed (enabled) */
                                         IOCON_PIO_ASW_EN);
    /* PORT0 PIN15 (coords: 22) is configured as PIO0_15 */
    IOCON_PinMuxSet(IOCON, 0U, 15U, port0_pin15_config);

    const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);

    const uint32_t port1_pin10_config = (/* Pin is configured as PIO1_10 */
                                         IOCON_PIO_FUNC0 |
                                         /* Selects pull-up function */
                                         IOCON_PIO_MODE_PULLUP |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN10 (coords: 40) is configured as PIO1_10 */
    IOCON_PinMuxSet(IOCON, 1U, 10U, port1_pin10_config);

    const uint32_t port1_pin4_config = (/* Pin is configured as PIO1_4 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
    IOCON_PinMuxSet(IOCON, 1U, 4U, port1_pin4_config);

    const uint32_t port1_pin5_config = (/* Pin is configured as PIO1_5 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN5 (coords: 31) is configured as PIO1_5 */
    IOCON_PinMuxSet(IOCON, 1U, 5U, port1_pin5_config);

    const uint32_t port1_pin6_config = (/* Pin is configured as PIO1_6 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN6 (coords: 5) is configured as PIO1_6 */
    IOCON_PinMuxSet(IOCON, 1U, 6U, port1_pin6_config);

    const uint32_t port1_pin7_config = (/* Pin is configured as PIO1_7 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN7 (coords: 9) is configured as PIO1_7 */
    IOCON_PinMuxSet(IOCON, 1U, 7U, port1_pin7_config);

    const uint32_t port1_pin8_config = (/* Pin is configured as PIO1_8 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI |
                                        /* Analog switch is open (disabled) */
                                        IOCON_PIO_ASW_DI);
    /* PORT1 PIN8 (coords: 24) is configured as PIO1_8 */
    IOCON_PinMuxSet(IOCON, 1U, 8U, port1_pin8_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C4_InitPins:
- options: {callFromInitBoot: 'false', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '4', peripheral: FLEXCOMM4, signal: TXD_SCL_MISO_WS, pin_signal: PIO1_20/FC7_RTS_SCL_SSEL1/CT_INP14/FC4_TXD_SCL_MISO_WS/PLU_OUT2, mode: inactive, slew_rate: standard,
    invert: disabled, open_drain: enabled}
  - {pin_num: '30', peripheral: FLEXCOMM4, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO1_21/FC7_CTS_SDA_SSEL0/CTIMER3_MAT2/FC4_RXD_SDA_MOSI_DATA/PLU_OUT3, mode: inactive,
    slew_rate: standard, invert: disabled, open_drain: enabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C4_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void I2C4_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin20_config = (/* Pin is configured as FC4_TXD_SCL_MISO_WS */
                                         IOCON_PIO_FUNC5 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is enabled */
                                         IOCON_PIO_OPENDRAIN_EN);
    /* PORT1 PIN20 (coords: 4) is configured as FC4_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

    const uint32_t port1_pin21_config = (/* Pin is configured as FC4_RXD_SDA_MOSI_DATA */
                                         IOCON_PIO_FUNC5 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is enabled */
                                         IOCON_PIO_OPENDRAIN_EN);
    /* PORT1 PIN21 (coords: 30) is configured as FC4_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C4_DeinitPins:
- options: {callFromInitBoot: 'false', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '4', peripheral: GPIO, signal: 'PIO1, 20', pin_signal: PIO1_20/FC7_RTS_SCL_SSEL1/CT_INP14/FC4_TXD_SCL_MISO_WS/PLU_OUT2, mode: inactive, slew_rate: standard,
    invert: disabled, open_drain: disabled}
  - {pin_num: '30', peripheral: GPIO, signal: 'PIO1, 21', pin_signal: PIO1_21/FC7_CTS_SDA_SSEL0/CTIMER3_MAT2/FC4_RXD_SDA_MOSI_DATA/PLU_OUT3, mode: inactive, slew_rate: standard,
    invert: disabled, open_drain: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C4_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void I2C4_DeinitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin20_config = (/* Pin is configured as PIO1_20 */
                                         IOCON_PIO_FUNC0 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN20 (coords: 4) is configured as PIO1_20 */
    IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

    const uint32_t port1_pin21_config = (/* Pin is configured as PIO1_21 */
                                         IOCON_PIO_FUNC0 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN21 (coords: 30) is configured as PIO1_21 */
    IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
