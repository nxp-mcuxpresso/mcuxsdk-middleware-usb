/*
 * Copyright 2020 ,2021 NXP
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

#define IOPCTL_PIO_ANAMUX_DI 0x00u        /*!<@brief Analog mux is disabled */
#define IOPCTL_PIO_FULLDRIVE_DI 0x00u     /*!<@brief Normal drive */
#define IOPCTL_PIO_FULLDRIVE_EN 0x0100u   /*!<@brief Full drive enable */
#define IOPCTL_PIO_FUNC0 0x00u            /*!<@brief Selects pin function 0 */
#define IOPCTL_PIO_FUNC1 0x01u            /*!<@brief Selects pin function 1 */
#define IOPCTL_PIO_INBUF_DI 0x00u         /*!<@brief Disable input buffer function */
#define IOPCTL_PIO_INBUF_EN 0x40u         /*!<@brief Enables input buffer function */
#define IOPCTL_PIO_INV_DI 0x00u           /*!<@brief Input function is not inverted */
#define IOPCTL_PIO_PSEDRAIN_DI 0x00u      /*!<@brief Pseudo Output Drain is disabled */
#define IOPCTL_PIO_PULLDOWN_EN 0x00u      /*!<@brief Enable pull-down function */
#define IOPCTL_PIO_PUPD_DI 0x00u          /*!<@brief Disable pull-up / pull-down function */
#define IOPCTL_PIO_SLEW_RATE_NORMAL 0x00u /*!<@brief Normal mode */

/*! @name FC0_RXD_SDA_MOSI_DATA (coord G4), JP21[2]/U28[3]/U9[13]
  @{ */
/* Routed pin properties */
#define BOARD_INITPINS_DEBUG_UART_RXD_PERIPHERAL FLEXCOMM0           /*!<@brief Peripheral name */
#define BOARD_INITPINS_DEBUG_UART_RXD_SIGNAL RXD_SDA_MOSI_DATA       /*!<@brief Signal name */
                                                                     /* @} */

/*! @name FC0_TXD_SCL_MISO_WS (coord G2), J16[1]/U27[3]/U9[12]
  @{ */
/* Routed pin properties */
#define BOARD_INITPINS_DEBUG_UART_TXD_PERIPHERAL FLEXCOMM0         /*!<@brief Peripheral name */
#define BOARD_INITPINS_DEBUG_UART_TXD_SIGNAL TXD_SCL_MISO_WS       /*!<@brief Signal name */
                                                                   /* @} */

/*! @name PIO1_1 (coord G15), SW1
  @{ */
/* Routed pin properties */
#define BOARD_INITPINS_SW1_PERIPHERAL GPIO          /*!<@brief Peripheral name */
#define BOARD_INITPINS_SW1_SIGNAL PIO1              /*!<@brief Signal name */
#define BOARD_INITPINS_SW1_CHANNEL 1                /*!<@brief Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SW1_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SW1_GPIO_PIN_MASK (1U << 1U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_SW1_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SW1_PIN 1U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SW1_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void); /* Function assigned for the Cortex-M33 */

#define IOPCTL_PIO_ANAMUX_DI 0x00u        /*!<@brief Analog mux is disabled */
#define IOPCTL_PIO_FULLDRIVE_DI 0x00u     /*!<@brief Normal drive */
#define IOPCTL_PIO_FUNC0 0x00u            /*!<@brief Selects pin function 0 */
#define IOPCTL_PIO_INBUF_EN 0x40u         /*!<@brief Enables input buffer function */
#define IOPCTL_PIO_INV_DI 0x00u           /*!<@brief Input function is not inverted */
#define IOPCTL_PIO_PSEDRAIN_DI 0x00u      /*!<@brief Pseudo Output Drain is disabled */
#define IOPCTL_PIO_PULLUP_EN 0x20u        /*!<@brief Enable pull-up function */
#define IOPCTL_PIO_PUPD_EN 0x10u          /*!<@brief Enable pull-up / pull-down function */
#define IOPCTL_PIO_SLEW_RATE_NORMAL 0x00u /*!<@brief Normal mode */

/*! @name PIO2_30 (coord P16), J18[2]/U8[3]/U17[B6]/SDA_CODEC
  @{ */
/* Routed pin properties */
/*!
 * @brief Peripheral name */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_PERIPHERAL GPIO
/*!
 * @brief Signal name */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_SIGNAL PIO2
/*!
 * @brief Signal channel */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_CHANNEL 30

/* Symbols to be used with GPIO driver */
/*!
 * @brief GPIO peripheral base pointer */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_GPIO GPIO
/*!
 * @brief GPIO pin mask */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_GPIO_PIN_MASK (1U << 30U)
/*!
 * @brief PORT peripheral base pointer */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_PORT 2U
/*!
 * @brief PORT pin number */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_PIN 30U
/*!
 * @brief PORT pin mask */
#define BOARD_INITI3CPINSASGPIO_I3C0_SDA_PIN_MASK (1U << 30U)
/* @} */

/*! @name PIO2_29 (coord N17), J18[1]/U8[2]/U17[A6]/SCL_CODEC
  @{ */
/* Routed pin properties */
/*!
 * @brief Peripheral name */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_PERIPHERAL GPIO
/*!
 * @brief Signal name */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_SIGNAL PIO2
/*!
 * @brief Signal channel */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_CHANNEL 29

/* Symbols to be used with GPIO driver */
/*!
 * @brief GPIO peripheral base pointer */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_GPIO GPIO
/*!
 * @brief GPIO pin mask */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_GPIO_PIN_MASK (1U << 29U)
/*!
 * @brief PORT peripheral base pointer */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_PORT 2U
/*!
 * @brief PORT pin number */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_PIN 29U
/*!
 * @brief PORT pin mask */
#define BOARD_INITI3CPINSASGPIO_I3C0_SCL_PIN_MASK (1U << 29U)
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitI3CPinsAsGPIO(void); /* Function assigned for the Cortex-M33 */

#define IOPCTL_PIO_ANAMUX_DI 0x00u        /*!<@brief Analog mux is disabled */
#define IOPCTL_PIO_FULLDRIVE_DI 0x00u     /*!<@brief Normal drive */
#define IOPCTL_PIO_FUNC1 0x01u            /*!<@brief Selects pin function 1 */
#define IOPCTL_PIO_INBUF_DI 0x00u         /*!<@brief Disable input buffer function */
#define IOPCTL_PIO_INBUF_EN 0x40u         /*!<@brief Enables input buffer function */
#define IOPCTL_PIO_INV_DI 0x00u           /*!<@brief Input function is not inverted */
#define IOPCTL_PIO_PSEDRAIN_DI 0x00u      /*!<@brief Pseudo Output Drain is disabled */
#define IOPCTL_PIO_PULLDOWN_EN 0x00u      /*!<@brief Enable pull-down function */
#define IOPCTL_PIO_PULLUP_EN 0x20u        /*!<@brief Enable pull-up function */
#define IOPCTL_PIO_PUPD_DI 0x00u          /*!<@brief Disable pull-up / pull-down function */
#define IOPCTL_PIO_PUPD_EN 0x10u          /*!<@brief Enable pull-up / pull-down function */
#define IOPCTL_PIO_SLEW_RATE_NORMAL 0x00u /*!<@brief Normal mode */

/*! @name I3C0_PUR (coord B6), J18[3]
  @{ */
/* Routed pin properties */
#define BOARD_INITI3CPINS_I3C0_PUR_PERIPHERAL I3C    /*!<@brief Peripheral name */
#define BOARD_INITI3CPINS_I3C0_PUR_SIGNAL PUR        /*!<@brief Signal name */
                                                     /* @} */

/*! @name I3C0_SCL (coord N17), J18[1]/U8[2]/U17[A6]/SCL_CODEC
  @{ */
/* Routed pin properties */
/*!
 * @brief Peripheral name */
#define BOARD_INITI3CPINS_I3C0_SCL_PERIPHERAL I3C
/*!
 * @brief Signal name */
#define BOARD_INITI3CPINS_I3C0_SCL_SIGNAL SCL
/* @} */

/*! @name I3C0_SDA (coord P16), J18[2]/U8[3]/U17[B6]/SDA_CODEC
  @{ */
/* Routed pin properties */
/*!
 * @brief Peripheral name */
#define BOARD_INITI3CPINS_I3C0_SDA_PERIPHERAL I3C
/*!
 * @brief Signal name */
#define BOARD_INITI3CPINS_I3C0_SDA_SIGNAL SDA
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitI3CPins(void); /* Function assigned for the Cortex-M33 */

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