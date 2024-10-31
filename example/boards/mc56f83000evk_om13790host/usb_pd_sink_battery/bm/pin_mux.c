/*
 * Copyright 2020-2021 NXP
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
product: Pins v10.0
processor: MC56F83789
package_id: MC56F83789VLL
mcu_data: ksdk2_0
processor_version: 0.10.10
board: MC56F83000-EVK
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
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
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '3', peripheral: OCCS, signal: EXTAL, pin_signal: GPIOC0/EXTAL/CLKIN0}
  - {pin_num: '4', peripheral: OCCS, signal: XTAL, pin_signal: GPIOC1/XTAL}
  - {pin_num: '9', peripheral: QSCI2, signal: TXD, pin_signal: GPIOD6/TXD2/XB_IN4/XB_OUT8}
  - {pin_num: '10', peripheral: QSCI2, signal: RXD, pin_signal: GPIOD5/RXD2/XB_IN5/XB_OUT9, open_drain: disable, pull_enable: disable}
  - {pin_num: '73', peripheral: GPIOE, signal: 'GPIO, 9', pin_signal: GPIOE9/PWMB_2A/PWMA_FAULT1/XB_OUT9, pull_enable: enable}
  - {pin_num: '78', peripheral: GPIOG, signal: 'GPIO, 0', pin_signal: GPIOG0/PWMB_1B/XB_OUT6}
  - {pin_num: '79', peripheral: GPIOG, signal: 'GPIO, 1', pin_signal: GPIOG1/PWMB_1A/XB_OUT7, pull_enable: enable}
  - {pin_num: '70', peripheral: GPIOG, signal: 'GPIO, 2', pin_signal: GPIOG2/PWMB_0B/XB_OUT4}
  - {pin_num: '71', peripheral: GPIOG, signal: 'GPIO, 3', pin_signal: GPIOG3/PWMB_0A/XB_OUT5}
  - {pin_num: '80', peripheral: GPIOG, signal: 'GPIO, 4', pin_signal: GPIOG4/PWMB_3B/PWMA_FAULT2/XB_OUT10}
  - {pin_num: '81', peripheral: GPIOG, signal: 'GPIO, 5', pin_signal: GPIOG5/PWMB_3A/PWMA_FAULT3/XB_OUT11, pull_enable: enable}
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
    /* GPIOC IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOC);
    /* GPIOD IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOD);
    /* GPIOE IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOE);
    /* GPIOG IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOG);
    /* Enable peripheral functionality on pin GPIOC0 (pin 3) */
    GPIO_PinSetPeripheralMode(BOARD_EXTAL_GPIO, BOARD_EXTAL_PIN_MASK, kGPIO_ModePeripheral);
    /* Peripheral functionality on pin GPIOC0 (pin 3) */
    GPIO_PinSetPeripheralMux(kGPIO_Peri_C0_EXTAL);
    /* Enable peripheral functionality on pin GPIOC1 (pin 4) */
    GPIO_PinSetPeripheralMode(BOARD_XTAL_GPIO, BOARD_XTAL_PIN_MASK, kGPIO_ModePeripheral);
    /* Enable peripheral functionality on pin GPIOD5 (pin 10) */
    GPIO_PinSetPeripheralMode(BOARD_RXD_BDM_GPIO, BOARD_RXD_BDM_PIN_MASK, kGPIO_ModePeripheral);
    /* Peripheral functionality on pin GPIOD5 (pin 10) */
    GPIO_PinSetPeripheralMux(kGPIO_Peri_D5_RXD2);
    /* Enable peripheral functionality on pin GPIOD6 (pin 9) */
    GPIO_PinSetPeripheralMode(BOARD_TXD_BDM_GPIO, BOARD_TXD_BDM_PIN_MASK, kGPIO_ModePeripheral);
    /* Peripheral functionality on pin GPIOD6 (pin 9) */
    GPIO_PinSetPeripheralMux(kGPIO_Peri_D6_TXD2);
    /* GPIO functionality on pin GPIOE9 (pin 73) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_2A_GPIO, BOARD_PWMB_2A_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG0 (pin 78) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_1B_GPIO, BOARD_PWMB_1B_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG1 (pin 79) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_1A_GPIO, BOARD_PWMB_1A_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG2 (pin 70) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_0B_GPIO, BOARD_PWMB_0B_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG3 (pin 71) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_0A_GPIO, BOARD_PWMB_0A_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG4 (pin 80) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_3B_GPIO, BOARD_PWMB_3B_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOG5 (pin 81) */
    GPIO_PinSetPeripheralMode(BOARD_PWMB_3A_GPIO, BOARD_PWMB_3A_PIN_MASK, kGPIO_ModeGpio);

    /* Pin configuration on GPIOD5 (pin 10) */
    /* Pull-Pull/Open drain output mode configuration: Push pull output mode */
    GPIO_PinSetOutputMode(BOARD_RXD_BDM_GPIO, BOARD_RXD_BDM_PIN_MASK, kGPIO_OutputPushPull);

    GPIOD->PUR = ((GPIOD->PUR &
                   /* Mask bits to zero which are setting */
                   (~(GPIO_PUR_PU_5_MASK)))

                  /* Pull Resistor Enable Bits: Pull resistor is disabled. */
                  | GPIO_PUR_PU(PUR_PU_5_DISABLED));

    GPIOE->PUR = ((GPIOE->PUR &
                   /* Mask bits to zero which are setting */
                   (~(GPIO_PUR_PU_9_MASK)))

                  /* Pull Resistor Enable Bits: Pull resistor is enabled. */
                  | GPIO_PUR_PU(PUR_PU_9_ENABLED));

    GPIOG->PUR = ((GPIOG->PUR &
                   /* Mask bits to zero which are setting */
                   (~(GPIO_PUR_PU_1_MASK | GPIO_PUR_PU_5_MASK)))

                  /* Pull Resistor Enable Bits: Pull resistor is enabled. */
                  | GPIO_PUR_PU(PUR_PU_1_ENABLED)

                  /* Pull Resistor Enable Bits: Pull resistor is enabled. */
                  | GPIO_PUR_PU(PUR_PU_5_ENABLED));
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_InitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '87', peripheral: I2C0, signal: SDA, pin_signal: GPIOC14/SDA0/XB_OUT4/PWMA_FAULT4, open_drain: enable, drive_strength: high, pull_enable: enable}
  - {pin_num: '88', peripheral: I2C0, signal: SCL, pin_signal: GPIOC15/SCL0/XB_OUT5/PWMA_FAULT5, open_drain: enable, drive_strength: high, pull_enable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C0_InitPins(void)
{
    /* GPIOC IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOC);
    /* Enable peripheral functionality on pin GPIOC14 (pin 87) */
    GPIO_PinSetPeripheralMode(I2C0_INITPINS_I2C_SDA0_GPIO, I2C0_INITPINS_I2C_SDA0_PIN_MASK, kGPIO_ModePeripheral);
    /* Peripheral functionality on pin GPIOC14 (pin 87) */
    GPIO_PinSetPeripheralMux(kGPIO_Peri_C14_SDA0);
    /* Enable peripheral functionality on pin GPIOC15 (pin 88) */
    GPIO_PinSetPeripheralMode(I2C0_INITPINS_I2C_SCL0_GPIO, I2C0_INITPINS_I2C_SCL0_PIN_MASK, kGPIO_ModePeripheral);
    /* Peripheral functionality on pin GPIOC15 (pin 88) */
    GPIO_PinSetPeripheralMux(kGPIO_Peri_C15_SCL0);

    /* Pin configuration on GPIOC14 (pin 87) */
    /* Pull-Pull/Open drain output mode configuration: Open drain output mode */
    GPIO_PinSetOutputMode(I2C0_INITPINS_I2C_SDA0_GPIO, I2C0_INITPINS_I2C_SDA0_PIN_MASK, kGPIO_OutputOpenDrain);
    /* Strength control configuration: High drive strength is configured */
    GPIO_PinSetDriveStrength(I2C0_INITPINS_I2C_SDA0_GPIO, I2C0_INITPINS_I2C_SDA0_PIN_MASK, kGPIO_DriveStrengthHigh);

    /* Pin configuration on GPIOC15 (pin 88) */
    /* Pull-Pull/Open drain output mode configuration: Open drain output mode */
    GPIO_PinSetOutputMode(I2C0_INITPINS_I2C_SCL0_GPIO, I2C0_INITPINS_I2C_SCL0_PIN_MASK, kGPIO_OutputOpenDrain);
    /* Strength control configuration: High drive strength is configured */
    GPIO_PinSetDriveStrength(I2C0_INITPINS_I2C_SCL0_GPIO, I2C0_INITPINS_I2C_SCL0_PIN_MASK, kGPIO_DriveStrengthHigh);

    GPIOC->PUR = ((GPIOC->PUR &
                   /* Mask bits to zero which are setting */
                   (~(GPIO_PUR_PU_14_MASK | GPIO_PUR_PU_15_MASK)))

                  /* Pull Resistor Enable Bits: Pull resistor is enabled. */
                  | GPIO_PUR_PU(PUR_PU_14_ENABLED)

                  /* Pull Resistor Enable Bits: Pull resistor is enabled. */
                  | GPIO_PUR_PU(PUR_PU_15_ENABLED));
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_DeinitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '87', peripheral: GPIOC, signal: 'GPIO, 14', pin_signal: GPIOC14/SDA0/XB_OUT4/PWMA_FAULT4, open_drain: enable, pull_enable: disable}
  - {pin_num: '88', peripheral: GPIOC, signal: 'GPIO, 15', pin_signal: GPIOC15/SCL0/XB_OUT5/PWMA_FAULT5, open_drain: enable, pull_enable: disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C0_DeinitPins(void)
{
    /* GPIOC IPBus Clock Enable: The peripheral is clocked. */
    CLOCK_EnableClock(kCLOCK_GPIOC);
    /* GPIO functionality on pin GPIOC14 (pin 87) */
    GPIO_PinSetPeripheralMode(I2C0_DEINITPINS_I2C_SDA0_GPIO, I2C0_DEINITPINS_I2C_SDA0_PIN_MASK, kGPIO_ModeGpio);
    /* GPIO functionality on pin GPIOC15 (pin 88) */
    GPIO_PinSetPeripheralMode(I2C0_DEINITPINS_I2C_SCL0_GPIO, I2C0_DEINITPINS_I2C_SCL0_PIN_MASK, kGPIO_ModeGpio);

    /* Pin configuration on GPIOC14 (pin 87) */
    /* Pull-Pull/Open drain output mode configuration: Open drain output mode */
    GPIO_PinSetOutputMode(I2C0_DEINITPINS_I2C_SDA0_GPIO, I2C0_DEINITPINS_I2C_SDA0_PIN_MASK, kGPIO_OutputOpenDrain);

    /* Pin configuration on GPIOC15 (pin 88) */
    /* Pull-Pull/Open drain output mode configuration: Open drain output mode */
    GPIO_PinSetOutputMode(I2C0_DEINITPINS_I2C_SCL0_GPIO, I2C0_DEINITPINS_I2C_SCL0_PIN_MASK, kGPIO_OutputOpenDrain);

    GPIOC->PUR = ((GPIOC->PUR &
                   /* Mask bits to zero which are setting */
                   (~(GPIO_PUR_PU_14_MASK | GPIO_PUR_PU_15_MASK)))

                  /* Pull Resistor Enable Bits: Pull resistor is disabled. */
                  | GPIO_PUR_PU(PUR_PU_14_DISABLED)

                  /* Pull Resistor Enable Bits: Pull resistor is disabled. */
                  | GPIO_PUR_PU(PUR_PU_15_DISABLED));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/