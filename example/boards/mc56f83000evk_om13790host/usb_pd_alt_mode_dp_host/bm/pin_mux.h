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

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define GPIO_PUR_PU_1_MASK 0x02u   /*!<@brief Pull Resistor Enable Bits Mask for item 1. */
#define GPIO_PUR_PU_5_MASK 0x20u   /*!<@brief Pull Resistor Enable Bits Mask for item 5. */
#define GPIO_PUR_PU_9_MASK 0x0200u /*!<@brief Pull Resistor Enable Bits Mask for item 9. */
#define PUR_PU_1_ENABLED 0x02u     /*!<@brief Pull Resistor Enable Bits: Pull resistor is enabled */
#define PUR_PU_5_DISABLED 0x00u    /*!<@brief Pull Resistor Enable Bits: Pull resistor is disabled */
#define PUR_PU_5_ENABLED 0x20u     /*!<@brief Pull Resistor Enable Bits: Pull resistor is enabled */
#define PUR_PU_9_ENABLED 0x0200u   /*!<@brief Pull Resistor Enable Bits: Pull resistor is enabled */

/*! @name GPIOC0 (number 3), Y1[1]/EXTAL
  @{ */
#define BOARD_EXTAL_GPIO GPIOC          /*!<@brief GPIO device name: GPIOC */
#define BOARD_EXTAL_PIN 0U              /*!<@brief GPIOC pin index: 0 */
#define BOARD_EXTAL_PIN_MASK kGPIO_Pin0 /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name GPIOC1 (number 4), Y1[3]/XTAL
  @{ */
#define BOARD_XTAL_GPIO GPIOC          /*!<@brief GPIO device name: GPIOC */
#define BOARD_XTAL_PIN 1U              /*!<@brief GPIOC pin index: 1 */
#define BOARD_XTAL_PIN_MASK kGPIO_Pin1 /*!<@brief PORT pin mask */
                                       /* @} */

/*! @name GPIOD6 (number 9), U14C[9]/TXD_BDM
  @{ */
#define BOARD_TXD_BDM_GPIO GPIOD          /*!<@brief GPIO device name: GPIOD */
#define BOARD_TXD_BDM_PIN 6U              /*!<@brief GPIOD pin index: 6 */
#define BOARD_TXD_BDM_PIN_MASK kGPIO_Pin6 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOD5 (number 10), U13[3]/RXD_BDM
  @{ */
#define BOARD_RXD_BDM_GPIO GPIOD          /*!<@brief GPIO device name: GPIOD */
#define BOARD_RXD_BDM_PIN 5U              /*!<@brief GPIOD pin index: 5 */
#define BOARD_RXD_BDM_PIN_MASK kGPIO_Pin5 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOE9 (number 73), J1[14]/GE9/PWMB_2A
  @{ */
#define BOARD_PWMB_2A_GPIO GPIOE          /*!<@brief GPIO device name: GPIOE */
#define BOARD_PWMB_2A_PIN 9U              /*!<@brief GPIOE pin index: 9 */
#define BOARD_PWMB_2A_PIN_MASK kGPIO_Pin9 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG0 (number 78), J1[12]/GG0/PWMB_1B
  @{ */
#define BOARD_PWMB_1B_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_1B_PIN 0U              /*!<@brief GPIOG pin index: 0 */
#define BOARD_PWMB_1B_PIN_MASK kGPIO_Pin0 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG1 (number 79), J1[10]/GG1/PWMB_1A
  @{ */
#define BOARD_PWMB_1A_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_1A_PIN 1U              /*!<@brief GPIOG pin index: 1 */
#define BOARD_PWMB_1A_PIN_MASK kGPIO_Pin1 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG2 (number 70), J1[8]/GG2/PWMB_0B
  @{ */
#define BOARD_PWMB_0B_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_0B_PIN 2U              /*!<@brief GPIOG pin index: 2 */
#define BOARD_PWMB_0B_PIN_MASK kGPIO_Pin2 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG3 (number 71), J1[6]/GG3/PWMB_0A
  @{ */
#define BOARD_PWMB_0A_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_0A_PIN 3U              /*!<@brief GPIOG pin index: 3 */
#define BOARD_PWMB_0A_PIN_MASK kGPIO_Pin3 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG4 (number 80), J2[4]/GG4/PWMB_3B
  @{ */
#define BOARD_PWMB_3B_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_3B_PIN 4U              /*!<@brief GPIOG pin index: 4 */
#define BOARD_PWMB_3B_PIN_MASK kGPIO_Pin4 /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name GPIOG5 (number 81), J2[2]/GG5/PWMB_3A
  @{ */
#define BOARD_PWMB_3A_GPIO GPIOG          /*!<@brief GPIO device name: GPIOG */
#define BOARD_PWMB_3A_PIN 5U              /*!<@brief GPIOG pin index: 5 */
#define BOARD_PWMB_3A_PIN_MASK kGPIO_Pin5 /*!<@brief PORT pin mask */
                                          /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#define GPIO_PUR_PU_14_MASK 0x4000u /*!<@brief Pull Resistor Enable Bits Mask for item 14. */
#define GPIO_PUR_PU_15_MASK 0x8000u /*!<@brief Pull Resistor Enable Bits Mask for item 15. */
#define PUR_PU_14_ENABLED 0x4000u   /*!<@brief Pull Resistor Enable Bits: Pull resistor is enabled */
#define PUR_PU_15_ENABLED 0x8000u   /*!<@brief Pull Resistor Enable Bits: Pull resistor is enabled */

/*! @name GPIOC14 (number 87), J2[18]/I2C_SDA0
  @{ */
#define I2C0_INITPINS_I2C_SDA0_GPIO GPIOC           /*!<@brief GPIO device name: GPIOC */
#define I2C0_INITPINS_I2C_SDA0_PIN 14U              /*!<@brief GPIOC pin index: 14 */
#define I2C0_INITPINS_I2C_SDA0_PIN_MASK kGPIO_Pin14 /*!<@brief PORT pin mask */
                                                    /* @} */

/*! @name GPIOC15 (number 88), J2[20]/I2C_SCL0
  @{ */
#define I2C0_INITPINS_I2C_SCL0_GPIO GPIOC           /*!<@brief GPIO device name: GPIOC */
#define I2C0_INITPINS_I2C_SCL0_PIN 15U              /*!<@brief GPIOC pin index: 15 */
#define I2C0_INITPINS_I2C_SCL0_PIN_MASK kGPIO_Pin15 /*!<@brief PORT pin mask */
                                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void I2C0_InitPins(void);

#define GPIO_PUR_PU_14_MASK 0x4000u /*!<@brief Pull Resistor Enable Bits Mask for item 14. */
#define GPIO_PUR_PU_15_MASK 0x8000u /*!<@brief Pull Resistor Enable Bits Mask for item 15. */
#define PUR_PU_14_DISABLED 0x00u    /*!<@brief Pull Resistor Enable Bits: Pull resistor is disabled */
#define PUR_PU_15_DISABLED 0x00u    /*!<@brief Pull Resistor Enable Bits: Pull resistor is disabled */

/*! @name GPIOC14 (number 87), J2[18]/I2C_SDA0
  @{ */
#define I2C0_DEINITPINS_I2C_SDA0_GPIO GPIOC           /*!<@brief GPIO device name: GPIOC */
#define I2C0_DEINITPINS_I2C_SDA0_PIN 14U              /*!<@brief GPIOC pin index: 14 */
#define I2C0_DEINITPINS_I2C_SDA0_PIN_MASK kGPIO_Pin14 /*!<@brief PORT pin mask */
                                                      /* @} */

/*! @name GPIOC15 (number 88), J2[20]/I2C_SCL0
  @{ */
#define I2C0_DEINITPINS_I2C_SCL0_GPIO GPIOC           /*!<@brief GPIO device name: GPIOC */
#define I2C0_DEINITPINS_I2C_SCL0_PIN 15U              /*!<@brief GPIOC pin index: 15 */
#define I2C0_DEINITPINS_I2C_SCL0_PIN_MASK kGPIO_Pin15 /*!<@brief PORT pin mask */
                                                      /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void I2C0_DeinitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/