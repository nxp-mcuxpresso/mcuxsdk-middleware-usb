/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*${header:start}*/
#include "fsl_wm8962.h"
/*${header:end}*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define DEMO_SAI_INSTANCE_INDEX (1U)
#define DEMO_SAI_TX_SOURCE      kDmaRequestMuxSai1Tx
#define DEMO_SAI_RX_SOURCE      kDmaRequestMuxSai1Rx
#define DEMO_DMA_INDEX          (0U)
#define DEMO_DMAMUX_INDEX       (0U)
#define DEMO_DMA_TX_CHANNEL     (0U)
#define DEMO_DMA_RX_CHANNEL     (1U)
#define DEMO_SAI                (SAI1)

/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Sai1)

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)

#define BOARD_SW_GPIO        BOARD_USER_BUTTON_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_USER_BUTTON_GPIO_PIN
#define BOARD_SW_IRQ         BOARD_USER_BUTTON_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_USER_BUTTON_IRQ_HANDLER
#define BOARD_SW_NAME        BOARD_USER_BUTTON_NAME
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */