/*
 * Copyright 2022 - 2023 NXP
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
product: Pins v14.0
processor: MCXN947
package_id: MCXN947VDF
mcu_data: ksdk2_0
processor_version: 0.15.3
pin_labels:
- {pin_num: F8, pin_signal: PIO0_29/FC1_P5/FC0_P5/CT_INP1/ADC0_B21, label: DP4LANE, identifier: DP4LANE}
- {pin_num: M4, pin_signal: PIO1_23/FC4_P3/CT_INP15/SCT0_OUT5/FLEXIO0_D31/ADC1_A23, label: DP_HPD, identifier: DP_HPD}
- {pin_num: E7, pin_signal: PIO0_30/FC1_P6/FC0_P6/CT_INP2/ADC0_B22, label: EXTRA_EN_SRCn, identifier: EXTRA_EN_SRCn}
- {pin_num: L5, pin_signal: PIO1_21/TRIG_OUT2/FC5_P5/FC4_P1/CT3_MAT3/SCT0_OUT9/FLEXIO0_D29/PLU_OUT7/ENET0_MDIO/SAI1_MCLK/CAN1_RXD/ADC1_A21/CMP2_IN3, label: P0_XSD,
  identifier: P0_XSD}
- {pin_num: C4, pin_signal: PIO1_2/TRIG_OUT0/FC3_P2/FC4_P6/CT1_MAT0/SCT0_IN6/FLEXIO0_D10/ENET0_MDC/SAI1_TXD0/CAN0_TXD/TSI0_CH2/ADC0_A18/CMP2_IN0, label: DC_BARREL_PRES,
  identifier: DC_BARREL_PRES}
- {pin_num: E8, pin_signal: PIO0_28/FC1_P4/FC0_P4/CT_INP0/ADC0_B20, label: nALERT, identifier: nALERT}
- {pin_num: B12, pin_signal: PIO0_10/FC0_P6/CT0_MAT0/FLEXIO0_D2/ADC0_B10, label: ORIENT, identifier: ORIENT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
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
  - {pin_num: A1, peripheral: LP_FLEXCOMM4, signal: LPFLEXCOMM_P0, pin_signal: PIO1_8/WUU0_IN10/LPTMR1_ALT3/TRACE_DATA0/FC4_P0/FC5_P4/CT_INP8/SCT0_OUT2/FLEXIO0_D16/PLU_OUT0/ENET0_TXD2/I3C1_SDA/TSI0_CH17/ADC1_A8,
    slew_rate: fast, open_drain: disable, drive_strength: low, pull_select: down, pull_enable: disable, passive_filter: disable, pull_value: low, input_buffer: enable,
    invert_input: normal}
  - {pin_num: B1, peripheral: LP_FLEXCOMM4, signal: LPFLEXCOMM_P1, pin_signal: PIO1_9/TRACE_DATA1/FC4_P1/FC5_P5/CT_INP9/SCT0_OUT3/FLEXIO0_D17/PLU_OUT1/ENET0_TXD3/I3C1_SCL/TSI0_CH18/ADC1_A9,
    slew_rate: fast, open_drain: disable, drive_strength: low, pull_select: down, pull_enable: disable, passive_filter: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: F8, peripheral: GPIO0, signal: 'GPIO, 29', pin_signal: PIO0_29/FC1_P5/FC0_P5/CT_INP1/ADC0_B21, direction: OUTPUT, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: M4, peripheral: GPIO1, signal: 'GPIO, 23', pin_signal: PIO1_23/FC4_P3/CT_INP15/SCT0_OUT5/FLEXIO0_D31/ADC1_A23, direction: OUTPUT, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: E7, peripheral: GPIO0, signal: 'GPIO, 30', pin_signal: PIO0_30/FC1_P6/FC0_P6/CT_INP2/ADC0_B22, direction: OUTPUT, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: L5, peripheral: GPIO1, signal: 'GPIO, 21', pin_signal: PIO1_21/TRIG_OUT2/FC5_P5/FC4_P1/CT3_MAT3/SCT0_OUT9/FLEXIO0_D29/PLU_OUT7/ENET0_MDIO/SAI1_MCLK/CAN1_RXD/ADC1_A21/CMP2_IN3,
    direction: OUTPUT, slew_rate: fast, open_drain: disable, drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: C4, peripheral: GPIO1, signal: 'GPIO, 2', pin_signal: PIO1_2/TRIG_OUT0/FC3_P2/FC4_P6/CT1_MAT0/SCT0_IN6/FLEXIO0_D10/ENET0_MDC/SAI1_TXD0/CAN0_TXD/TSI0_CH2/ADC0_A18/CMP2_IN0,
    direction: INPUT, slew_rate: fast, open_drain: disable, drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: E8, peripheral: GPIO0, signal: 'GPIO, 28', pin_signal: PIO0_28/FC1_P4/FC0_P4/CT_INP0/ADC0_B20, direction: INPUT, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: B12, peripheral: GPIO0, signal: 'GPIO, 10', pin_signal: PIO0_10/FC0_P6/CT0_MAT0/FLEXIO0_D2/ADC0_B10, direction: OUTPUT, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Enables the clock for GPIO0: Enables clock */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    /* Enables the clock for GPIO1: Enables clock */
    CLOCK_EnableClock(kCLOCK_Gpio1);
    /* Enables the clock for PORT0 controller: Enables clock */
    CLOCK_EnableClock(kCLOCK_Port0);
    /* Enables the clock for PORT1: Enables clock */
    CLOCK_EnableClock(kCLOCK_Port1);

    gpio_pin_config_t ORIENT_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_10 (pin B12)  */
    GPIO_PinInit(BOARD_INITPINS_ORIENT_GPIO, BOARD_INITPINS_ORIENT_PIN, &ORIENT_config);

    gpio_pin_config_t nALERT_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_28 (pin E8)  */
    GPIO_PinInit(BOARD_INITPINS_nALERT_GPIO, BOARD_INITPINS_nALERT_PIN, &nALERT_config);

    gpio_pin_config_t DP4LANE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_29 (pin F8)  */
    GPIO_PinInit(BOARD_INITPINS_DP4LANE_GPIO, BOARD_INITPINS_DP4LANE_PIN, &DP4LANE_config);

    gpio_pin_config_t EXTRA_EN_SRCn_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_30 (pin E7)  */
    GPIO_PinInit(BOARD_INITPINS_EXTRA_EN_SRCn_GPIO, BOARD_INITPINS_EXTRA_EN_SRCn_PIN, &EXTRA_EN_SRCn_config);

    gpio_pin_config_t DC_BARREL_PRES_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_2 (pin C4)  */
    GPIO_PinInit(BOARD_INITPINS_DC_BARREL_PRES_GPIO, BOARD_INITPINS_DC_BARREL_PRES_PIN, &DC_BARREL_PRES_config);

    gpio_pin_config_t P0_XSD_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_21 (pin L5)  */
    GPIO_PinInit(BOARD_INITPINS_P0_XSD_GPIO, BOARD_INITPINS_P0_XSD_PIN, &P0_XSD_config);

    gpio_pin_config_t DP_HPD_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_23 (pin M4)  */
    GPIO_PinInit(BOARD_INITPINS_DP_HPD_GPIO, BOARD_INITPINS_DP_HPD_PIN, &DP_HPD_config);

    const port_pin_config_t ORIENT = {/* Internal pull-up/down resistor is disabled */
                                      kPORT_PullDisable,
                                      /* Low internal pull resistor value is selected. */
                                      kPORT_LowPullResistor,
                                      /* Fast slew rate is configured */
                                      kPORT_FastSlewRate,
                                      /* Passive input filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Open drain output is disabled */
                                      kPORT_OpenDrainDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PIO0_10 */
                                      kPORT_MuxAlt0,
                                      /* Digital input enabled */
                                      kPORT_InputBufferEnable,
                                      /* Digital input is not inverted */
                                      kPORT_InputNormal,
                                      /* Pin Control Register fields [15:0] are not locked */
                                      kPORT_UnlockRegister};
    /* PORT0_10 (pin B12) is configured as PIO0_10 */
    PORT_SetPinConfig(BOARD_INITPINS_ORIENT_PORT, BOARD_INITPINS_ORIENT_PIN, &ORIENT);

    const port_pin_config_t nALERT = {/* Internal pull-up/down resistor is disabled */
                                      kPORT_PullDisable,
                                      /* Low internal pull resistor value is selected. */
                                      kPORT_LowPullResistor,
                                      /* Fast slew rate is configured */
                                      kPORT_FastSlewRate,
                                      /* Passive input filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Open drain output is disabled */
                                      kPORT_OpenDrainDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PIO0_28 */
                                      kPORT_MuxAlt0,
                                      /* Digital input enabled */
                                      kPORT_InputBufferEnable,
                                      /* Digital input is not inverted */
                                      kPORT_InputNormal,
                                      /* Pin Control Register fields [15:0] are not locked */
                                      kPORT_UnlockRegister};
    /* PORT0_28 (pin E8) is configured as PIO0_28 */
    PORT_SetPinConfig(BOARD_INITPINS_nALERT_PORT, BOARD_INITPINS_nALERT_PIN, &nALERT);

    const port_pin_config_t DP4LANE = {/* Internal pull-up/down resistor is disabled */
                                       kPORT_PullDisable,
                                       /* Low internal pull resistor value is selected. */
                                       kPORT_LowPullResistor,
                                       /* Fast slew rate is configured */
                                       kPORT_FastSlewRate,
                                       /* Passive input filter is disabled */
                                       kPORT_PassiveFilterDisable,
                                       /* Open drain output is disabled */
                                       kPORT_OpenDrainDisable,
                                       /* Low drive strength is configured */
                                       kPORT_LowDriveStrength,
                                       /* Pin is configured as PIO0_29 */
                                       kPORT_MuxAlt0,
                                       /* Digital input enabled */
                                       kPORT_InputBufferEnable,
                                       /* Digital input is not inverted */
                                       kPORT_InputNormal,
                                       /* Pin Control Register fields [15:0] are not locked */
                                       kPORT_UnlockRegister};
    /* PORT0_29 (pin F8) is configured as PIO0_29 */
    PORT_SetPinConfig(BOARD_INITPINS_DP4LANE_PORT, BOARD_INITPINS_DP4LANE_PIN, &DP4LANE);

    const port_pin_config_t EXTRA_EN_SRCn = {/* Internal pull-up/down resistor is disabled */
                                             kPORT_PullDisable,
                                             /* Low internal pull resistor value is selected. */
                                             kPORT_LowPullResistor,
                                             /* Fast slew rate is configured */
                                             kPORT_FastSlewRate,
                                             /* Passive input filter is disabled */
                                             kPORT_PassiveFilterDisable,
                                             /* Open drain output is disabled */
                                             kPORT_OpenDrainDisable,
                                             /* Low drive strength is configured */
                                             kPORT_LowDriveStrength,
                                             /* Pin is configured as PIO0_30 */
                                             kPORT_MuxAlt0,
                                             /* Digital input enabled */
                                             kPORT_InputBufferEnable,
                                             /* Digital input is not inverted */
                                             kPORT_InputNormal,
                                             /* Pin Control Register fields [15:0] are not locked */
                                             kPORT_UnlockRegister};
    /* PORT0_30 (pin E7) is configured as PIO0_30 */
    PORT_SetPinConfig(BOARD_INITPINS_EXTRA_EN_SRCn_PORT, BOARD_INITPINS_EXTRA_EN_SRCn_PIN, &EXTRA_EN_SRCn);

    const port_pin_config_t DC_BARREL_PRES = {/* Internal pull-up/down resistor is disabled */
                                              kPORT_PullDisable,
                                              /* Low internal pull resistor value is selected. */
                                              kPORT_LowPullResistor,
                                              /* Fast slew rate is configured */
                                              kPORT_FastSlewRate,
                                              /* Passive input filter is disabled */
                                              kPORT_PassiveFilterDisable,
                                              /* Open drain output is disabled */
                                              kPORT_OpenDrainDisable,
                                              /* Low drive strength is configured */
                                              kPORT_LowDriveStrength,
                                              /* Pin is configured as PIO1_2 */
                                              kPORT_MuxAlt0,
                                              /* Digital input enabled */
                                              kPORT_InputBufferEnable,
                                              /* Digital input is not inverted */
                                              kPORT_InputNormal,
                                              /* Pin Control Register fields [15:0] are not locked */
                                              kPORT_UnlockRegister};
    /* PORT1_2 (pin C4) is configured as PIO1_2 */
    PORT_SetPinConfig(BOARD_INITPINS_DC_BARREL_PRES_PORT, BOARD_INITPINS_DC_BARREL_PRES_PIN, &DC_BARREL_PRES);

    const port_pin_config_t P0_XSD = {/* Internal pull-up/down resistor is disabled */
                                      kPORT_PullDisable,
                                      /* Low internal pull resistor value is selected. */
                                      kPORT_LowPullResistor,
                                      /* Fast slew rate is configured */
                                      kPORT_FastSlewRate,
                                      /* Passive input filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Open drain output is disabled */
                                      kPORT_OpenDrainDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PIO1_21 */
                                      kPORT_MuxAlt0,
                                      /* Digital input enabled */
                                      kPORT_InputBufferEnable,
                                      /* Digital input is not inverted */
                                      kPORT_InputNormal,
                                      /* Pin Control Register fields [15:0] are not locked */
                                      kPORT_UnlockRegister};
    /* PORT1_21 (pin L5) is configured as PIO1_21 */
    PORT_SetPinConfig(BOARD_INITPINS_P0_XSD_PORT, BOARD_INITPINS_P0_XSD_PIN, &P0_XSD);

    const port_pin_config_t DP_HPD = {/* Internal pull-up/down resistor is disabled */
                                      kPORT_PullDisable,
                                      /* Low internal pull resistor value is selected. */
                                      kPORT_LowPullResistor,
                                      /* Fast slew rate is configured */
                                      kPORT_FastSlewRate,
                                      /* Passive input filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Open drain output is disabled */
                                      kPORT_OpenDrainDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PIO1_23 */
                                      kPORT_MuxAlt0,
                                      /* Digital input enabled */
                                      kPORT_InputBufferEnable,
                                      /* Digital input is not inverted */
                                      kPORT_InputNormal,
                                      /* Pin Control Register fields [15:0] are not locked */
                                      kPORT_UnlockRegister};
    /* PORT1_23 (pin M4) is configured as PIO1_23 */
    PORT_SetPinConfig(BOARD_INITPINS_DP_HPD_PORT, BOARD_INITPINS_DP_HPD_PIN, &DP_HPD);

    const port_pin_config_t port1_8_pinA1_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as FC4_P0 */
                                                    kPORT_MuxAlt2,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT1_8 (pin A1) is configured as FC4_P0 */
    PORT_SetPinConfig(PORT1, 8U, &port1_8_pinA1_config);

    const port_pin_config_t port1_9_pinB1_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as FC4_P1 */
                                                    kPORT_MuxAlt2,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT1_9 (pin B1) is configured as FC4_P1 */
    PORT_SetPinConfig(PORT1, 9U, &port1_9_pinB1_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C2_InitPins:
- options: {createDeInit: 'true', callFromInitBoot: 'false', coreID: cm33_core0, enableClock: 'true', customDeInitName: I2C2_DeinitPins}
- pin_list:
  - {pin_num: P1, peripheral: LP_FLEXCOMM2, signal: LPFLEXCOMM_P0, pin_signal: PIO4_0/WUU0_IN18/TRIG_IN6/FC2_P0/CT_INP16/PLU_IN0/SINC0_MCLK3, slew_rate: fast, open_drain: disable,
    drive_strength: low, pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
  - {pin_num: P2, peripheral: LP_FLEXCOMM2, signal: LPFLEXCOMM_P1, pin_signal: PIO4_1/TRIG_IN7/FC2_P1/CT_INP17/PLU_IN1, slew_rate: fast, open_drain: disable, drive_strength: low,
    pull_select: down, pull_enable: disable, input_buffer: enable, invert_input: normal}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C2_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C2_InitPins(void)
{
    /* Enables the clock for PORT4: Enables clock */
    CLOCK_EnableClock(kCLOCK_Port4);

    const port_pin_config_t port4_0_pinP1_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as FC2_P0 */
                                                    kPORT_MuxAlt2,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT4_0 (pin P1) is configured as FC2_P0 */
    PORT_SetPinConfig(PORT4, 0U, &port4_0_pinP1_config);

    const port_pin_config_t port4_1_pinP2_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as FC2_P1 */
                                                    kPORT_MuxAlt2,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT4_1 (pin P2) is configured as FC2_P1 */
    PORT_SetPinConfig(PORT4, 1U, &port4_1_pinP2_config);
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C2_DeinitPins
 * Description   : This is a de-initialization function for 'I2C2_InitPins' function.
 * It sets all pins features (routing, direction and electrical) to their after-reset state.
 * It also tries to route the previous peripheral signals to their default pins.
 *
 * END ****************************************************************************************************************/
void I2C2_DeinitPins(void)
{
    /* Enables the clock for PORT4: Enables clock */
    CLOCK_EnableClock(kCLOCK_Port4);

    /* EFT detect interrupts configuration on PORT4_ */
    PORT_DisableEFTDetectInterrupts(PORT4, 0x03u);

    const port_pin_config_t port4_0_pinP1_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is disabled */
                                                    kPORT_MuxAlt0,
                                                    /* Digital input disabled; it is required for analog functions */
                                                    kPORT_InputBufferDisable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT4_0 (pin P1) is disabled */
    PORT_SetPinConfig(PORT4, 0U, &port4_0_pinP1_config);

    const port_pin_config_t port4_1_pinP2_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is disabled */
                                                    kPORT_MuxAlt0,
                                                    /* Digital input disabled; it is required for analog functions */
                                                    kPORT_InputBufferDisable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT4_1 (pin P2) is disabled */
    PORT_SetPinConfig(PORT4, 1U, &port4_1_pinP2_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
