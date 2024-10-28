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

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v14.0
processor: MIMXRT1189xxxxx
package_id: MIMXRT1189CVM8B
mcu_data: ksdk2_0
processor_version: 0.14.8
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list:
  - {pin_num: C5, peripheral: LPI2C2, signal: SCL, pin_signal: GPIO_AON_16, software_input_on: Enable, pull_up_down_config: Pull_Up, open_drain: Enable}
  - {pin_num: B3, peripheral: LPI2C2, signal: SDA, pin_signal: GPIO_AON_15, software_input_on: Enable, pull_up_down_config: Pull_Up, open_drain: Enable}
  - {pin_num: F6, peripheral: SAI1, signal: sai_tx_data0, pin_signal: GPIO_AON_21, software_input_on: Enable, pull_up_down_config: Pull_Down}
  - {pin_num: C3, peripheral: SAI1, signal: sai_tx_sync, pin_signal: GPIO_AON_22, software_input_on: Enable, pull_up_down_config: Pull_Down}
  - {pin_num: C2, peripheral: SAI1, signal: sai_tx_bclk, pin_signal: GPIO_AON_23, software_input_on: Enable, pull_up_down_config: Pull_Down}
  - {pin_num: C1, peripheral: SAI1, signal: sai_mclk, pin_signal: GPIO_AON_24, software_input_on: Enable, pull_up_down_config: Pull_Down}
  - {pin_num: A5, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AON_09, software_input_on: Enable, pull_up_down_config: Pull_Down}
  - {pin_num: B1, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AON_08, pull_up_down_config: Pull_Down}
  - {pin_num: B4, peripheral: RGPIO1, signal: 'gpio_io, 04', pin_signal: GPIO_AON_04, pull_up_down_config: Pull_Up, pull_keeper_select: Pull}
  - {pin_num: D2, peripheral: SAI1, signal: sai_rx_data0, pin_signal: GPIO_AON_25, software_input_on: Enable, pull_up_down_config: Pull_Down}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins, assigned for the Cortex-M33 core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc2);          /* Turn on LPCG: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_04_GPIO1_IO04,          /* GPIO_AON_04 is configured as GPIO1_IO04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_08_LPUART1_TX,          /* GPIO_AON_08 is configured as LPUART1_TX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_09_LPUART1_RX,          /* GPIO_AON_09 is configured as LPUART1_RX */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_09 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_15_LPI2C2_SDA,          /* GPIO_AON_15 is configured as LPI2C2_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_15 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_16_LPI2C2_SCL,          /* GPIO_AON_16 is configured as LPI2C2_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_16 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_21_SAI1_TX_DATA00,      /* GPIO_AON_21 is configured as SAI1_TX_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_21 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_22_SAI1_TX_SYNC,        /* GPIO_AON_22 is configured as SAI1_TX_SYNC */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_22 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_23_SAI1_TX_BCLK,        /* GPIO_AON_23 is configured as SAI1_TX_BCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_23 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_24_SAI1_MCLK,           /* GPIO_AON_24 is configured as SAI1_MCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_24 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_25_SAI1_RX_DATA00,      /* GPIO_AON_25 is configured as SAI1_RX_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_25 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_04_GPIO1_IO04,          /* GPIO_AON_04 PAD functional properties : */
      0x0EU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_08_LPUART1_TX,          /* GPIO_AON_08 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_09_LPUART1_RX,          /* GPIO_AON_09 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_15_LPI2C2_SDA,          /* GPIO_AON_15 PAD functional properties : */
      0x1EU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_16_LPI2C2_SCL,          /* GPIO_AON_16 PAD functional properties : */
      0x1EU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_21_SAI1_TX_DATA00,      /* GPIO_AON_21 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_22_SAI1_TX_SYNC,        /* GPIO_AON_22 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_23_SAI1_TX_BCLK,        /* GPIO_AON_23 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_24_SAI1_MCLK,           /* GPIO_AON_24 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_25_SAI1_RX_DATA00,      /* GPIO_AON_25 PAD functional properties : */
      0x06U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
