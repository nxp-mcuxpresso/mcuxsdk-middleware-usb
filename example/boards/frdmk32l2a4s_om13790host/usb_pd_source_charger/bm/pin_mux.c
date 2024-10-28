/*
 * Copyright 2019 NXP
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
processor: K32L2A41xxxxA
package_id: K32L2A41VLL1A
mcu_data: ksdk2_0
processor_version: 0.0.0
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
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '62', peripheral: LPUART0, signal: RX, pin_signal: TSI0_CH9/PTB16/LPSPI1_SOUT/LPUART0_RX/TPM0_CLKIN/LPSPI2_PCS3/FXIO0_D16}
  - {pin_num: '63', peripheral: LPUART0, signal: TX, pin_signal: TSI0_CH10/PTB17/LPSPI1_SIN/LPUART0_TX/TPM1_CLKIN/LPSPI2_PCS2/FXIO0_D17}
  - {pin_num: '99', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/LPSPI1_SOUT/LPUART0_RX/FXIO0_D6, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '60', peripheral: GPIOB, signal: 'GPIO, 10', pin_signal: PTB10/LPSPI1_PCS0/FXIO0_D14, direction: OUTPUT, gpio_init_state: 'false', pull_select: up,
    pull_enable: enable}
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
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);

    gpio_pin_config_t gpiob_pin60_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB10 (pin 60)  */
    GPIO_PinInit(GPIOB, 10U, &gpiob_pin60_config);

    gpio_pin_config_t gpiod_pin99_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTD6 (pin 99)  */
    GPIO_PinInit(GPIOD, 6U, &gpiod_pin99_config);

    /* PORTB10 (pin 60) is configured as PTB10 */
    PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio);

    PORTB->PCR[10] = ((PORTB->PCR[10] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                       * corresponding PE field is set. */
                      | (uint32_t)(kPORT_PullUp));

    /* PORTB16 (pin 62) is configured as LPUART0_RX */
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt3);

    /* PORTB17 (pin 63) is configured as LPUART0_TX */
    PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt3);

    /* PORTD6 (pin 99) is configured as PTD6 */
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAsGpio);

    PORTD->PCR[6] = ((PORTD->PCR[6] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | (uint32_t)(kPORT_PullUp));
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
LPI2C1_InitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '71', peripheral: LPI2C1, signal: SCL, pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/LPI2C1_SCL/LPUART1_RTS_b/TPM0_CH0, slew_rate: fast, open_drain: enable,
    pull_select: up, pull_enable: enable}
  - {pin_num: '72', peripheral: LPI2C1, signal: SDA, pin_signal: ADC0_SE11/CMP1_IN0/TSI0_CH15/PTC2/LPI2C1_SDA/LPUART1_CTS_b/TPM0_CH1, slew_rate: fast, open_drain: enable,
    pull_select: up, pull_enable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : LPI2C1_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void LPI2C1_InitPins(void)
{
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    const port_pin_config_t portc1_pin71_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullUp,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Open drain is enabled */
                                                   kPORT_OpenDrainEnable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as LPI2C1_SCL */
                                                   kPORT_MuxAlt2,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   kPORT_UnlockRegister};
    /* PORTC1 (pin 71) is configured as LPI2C1_SCL */
    PORT_SetPinConfig(PORTC, 1U, &portc1_pin71_config);

    const port_pin_config_t portc2_pin72_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullUp,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Open drain is enabled */
                                                   kPORT_OpenDrainEnable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as LPI2C1_SDA */
                                                   kPORT_MuxAlt2,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   kPORT_UnlockRegister};
    /* PORTC2 (pin 72) is configured as LPI2C1_SDA */
    PORT_SetPinConfig(PORTC, 2U, &portc2_pin72_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
LPI2C1_DeinitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '71', peripheral: TSI0, signal: 'CH, 14', pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/LPI2C1_SCL/LPUART1_RTS_b/TPM0_CH0}
  - {pin_num: '72', peripheral: TSI0, signal: 'CH, 15', pin_signal: ADC0_SE11/CMP1_IN0/TSI0_CH15/PTC2/LPI2C1_SDA/LPUART1_CTS_b/TPM0_CH1}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : LPI2C1_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void LPI2C1_DeinitPins(void)
{
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* PORTC1 (pin 71) is configured as TSI0_CH14 */
    PORT_SetPinMux(PORTC, 1U, kPORT_PinDisabledOrAnalog);

    /* PORTC2 (pin 72) is configured as TSI0_CH15 */
    PORT_SetPinMux(PORTC, 2U, kPORT_PinDisabledOrAnalog);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
