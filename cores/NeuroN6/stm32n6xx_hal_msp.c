/**
  ******************************************************************************
  * @file         stm32n6xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  HAL_PWREx_EnableVddIO3();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO3, PWR_VDDIO_RANGE_1V8);

  HAL_PWREx_EnableVddIO4();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO4, PWR_VDDIO_RANGE_3V3);
}
void HAL_DCMIPP_MspInit(DCMIPP_HandleTypeDef* hdcmipp)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  RIMC_MasterConfig_t RIMC_master = {0};

  if (hdcmipp->Instance == DCMIPP)
  {
    __HAL_RCC_DCMIPP_CLK_ENABLE();
    __HAL_RCC_DCMIPP_FORCE_RESET();
    __HAL_RCC_DCMIPP_RELEASE_RESET();

    __HAL_RCC_CSI_CLK_ENABLE();
    __HAL_RCC_CSI_FORCE_RESET();
    __HAL_RCC_CSI_RELEASE_RESET();

    /* Same priority as the USB IRQ, so the USB ISR cannot preempt the camera and
     * encoder path. */
    HAL_NVIC_SetPriority(DCMIPP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DCMIPP_IRQn);

    HAL_NVIC_SetPriority(CSI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CSI_IRQn);

    PeriphClkInitStruct.PeriphClockSelection       = RCC_PERIPHCLK_DCMIPP;
    PeriphClkInitStruct.DcmippClockSelection       = RCC_DCMIPPCLKSOURCE_IC17;
    PeriphClkInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    PeriphClkInitStruct.ICSelection[RCC_IC17].ClockDivider   = 4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection       = RCC_PERIPHCLK_CSI;
    PeriphClkInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    PeriphClkInitStruct.ICSelection[RCC_IC18].ClockDivider   = 50;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_RIFSC_CLK_ENABLE();

    /* Configure DCMIPP as a secure, privileged master (CID1) */
    RIMC_master.MasterCID = RIF_CID_1;
    RIMC_master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

  }
}
void HAL_DCMIPP_MspDeInit(DCMIPP_HandleTypeDef* hdcmipp)
{
  if(hdcmipp->Instance==DCMIPP)
  {
    /* Read back first: a posted write in flight never retires once the clock is
     * gated, and the next APB5 access then stalls. */
    (void)READ_REG(hdcmipp->Instance->CMCR);
    (void)READ_REG(CSI->PRCR);
    __DSB();
    /* Peripheral clock disable */
    __HAL_RCC_CSI_CLK_DISABLE();
    __HAL_RCC_CSI_FORCE_RESET();
    __HAL_RCC_CSI_RELEASE_RESET();
  }

}
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PE6     ------> I2C1_SDA
    PH9     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* Blocking transfers only, so no interrupts. */
  }
  else if(hi2c->Instance==I2C2)
  {
    /* Wire1 on the header pins. Blocking transfers only, so no interrupts. */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PD15     ------> I2C2_SDA (header SDA, 4.7k pull-up on the board)
    PD14     ------> I2C2_SCL (header SCL, 4.7k pull-up on the board)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    __HAL_RCC_I2C2_CLK_ENABLE();
  }

}
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  /* Read back first, so the last write retires before the clock stops. */
  (void)READ_REG(hi2c->Instance->CR1);
  __DSB();
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PE6     ------> I2C1_SDA
    PH9     ------> I2C1_SCL
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_9);

    /* I2C1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  }
  else if(hi2c->Instance==I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_14 | GPIO_PIN_15);
  }

}

extern volatile uint32_t g_ltdc_clk_divider;   /* set by n6_lcd_set_fps() */

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hltdc->Instance==LTDC)
  {
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.LtdcClockSelection = RCC_LTDCCLKSOURCE_IC16;
    PeriphClkInitStruct.ICSelection[RCC_IC16].ClockSelection = RCC_ICCLKSOURCE_PLL4;
    /* The pixel clock is PLL4, 400 MHz, over the divider. The panel refresh is
     * 400e6 / (divider x 976 x 528). */
    PeriphClkInitStruct.ICSelection[RCC_IC16].ClockDivider = g_ltdc_clk_divider;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**LTDC GPIO Configuration
    PD8     ------> LTDC_R7
    PD13     ------> LTDC_R6
    PF4     ------> LTDC_R3
    PF7     ------> LTDC_VSYNC
    PA6     ------> LTDC_HSYNC
    PA1     ------> LTDC_G2
    PB11     ------> LTDC_G6
    PA11     ------> LTDC_B3
    PA9     ------> LTDC_B5
    PA5     ------> LTDC_CLK
    PB12     ------> LTDC_G5
    PA10     ------> LTDC_B4
    PA2     ------> LTDC_B7
    PG10     ------> LTDC_G4
    PA15(JTDI)     ------> LTDC_R5
    PA8     ------> LTDC_B6
    PG13     ------> LTDC_DE
    PA0     ------> LTDC_G3
    PB10     ------> LTDC_G7
    PA7     ------> LTDC_R4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_9
                          |GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_15
                          |GPIO_PIN_8|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_LCD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}

/**
  * @brief LTDC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc)
{
  if(hltdc->Instance==LTDC)
  {
    /* Read back first: a posted write in flight never retires once the clock is
     * gated, and the next APB5 access then stalls. */
    (void)READ_REG(hltdc->Instance->GCR);
    __DSB();
    /* Peripheral clock disable */
    __HAL_RCC_LTDC_CLK_DISABLE();

    /**LTDC GPIO Configuration
    PD8     ------> LTDC_R7
    PD13     ------> LTDC_R6
    PF4     ------> LTDC_R3
    PF7     ------> LTDC_VSYNC
    PA6     ------> LTDC_HSYNC
    PA1     ------> LTDC_G2
    PB11     ------> LTDC_G6
    PA11     ------> LTDC_B3
    PA9     ------> LTDC_B5
    PA5     ------> LTDC_CLK
    PB12     ------> LTDC_G5
    PA10     ------> LTDC_B4
    PA2     ------> LTDC_B7
    PG10     ------> LTDC_G4
    PA15(JTDI)     ------> LTDC_R5
    PA8     ------> LTDC_B6
    PG13     ------> LTDC_DE
    PA0     ------> LTDC_G3
    PB10     ------> LTDC_G7
    PA7     ------> LTDC_R4
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_13);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_4|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_9
                          |GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_15
                          |GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_10|GPIO_PIN_13);
  }

}

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  if (hpcd->Instance == USB1_OTG_HS)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* ST's example settings. Each write takes its register field from the
     * constant, not from the struct field the constant is assigned to. */
    PeriphClkInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_USBOTGHS1;
    PeriphClkInitStruct.UsbOtgHs1ClockSelection = RCC_USBPHY1REFCLKSOURCE_HSE_DIRECT;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection   = RCC_PERIPHCLK_USBPHY1;
    PeriphClkInitStruct.UsbPhy1ClockSelection  = RCC_USBOTGHS1CLKSOURCE_HSE_DIV2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();  // USB pins are on GPIOA

    // AHB5 reset mask covering the OTG_HS1 block
    LL_AHB5_GRP1_ForceReset(0x00800000);

    __HAL_RCC_USB1_OTG_HS_FORCE_RESET();
    __HAL_RCC_USB1_OTG_HS_PHY_FORCE_RESET();

    LL_RCC_HSE_SelectHSEDiv2AsDiv2Clock();

    LL_AHB5_GRP1_ReleaseReset(0x00800000);

    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    /* Required few clock cycles before accessing USB PHY Controller Registers */
    for (volatile uint32_t i = 0; i < 10; i++) {
        __NOP();
    }

    USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 4);      // clear FSEL bits
    USB1_HS_PHYC->USBPHYC_CR |=
          (0x1U << 16)  // ENUTMILEVEL2
        | (0x2U << 4)   // FSEL = 24 MHz
        | (0x1U << 2)   // ENHSE
        | 0x1U;         // PHYEN

    __HAL_RCC_USB1_OTG_HS_PHY_RELEASE_RESET();

    /* Required few clock cycles before releasing reset */
    for (volatile uint32_t i = 0; i < 10; i++) {
        __NOP();
    }

    __HAL_RCC_USB1_OTG_HS_RELEASE_RESET();

    __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

    HAL_PWREx_EnableVddUSB();

    /* The ISR calls vTaskNotifyGiveFromISR, which FreeRTOS permits only at or
     * below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 5. */
    HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
  }
}

/**
  * @brief PCD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB1_OTG_HS)
  {
    /* Read back first, so the last write retires before the clock stops. */
    (void)READ_REG(hpcd->Instance->GUSBCFG);
    __DSB();
    /* Peripheral clock disable */
    __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_DISABLE();

    /* Disable VDDUSB */
      HAL_PWREx_DisableVddUSB();

  }

}
/**
  * @brief CACHEAXI MSP Initialization
  * @param hcacheaxi: CACHEAXI handle pointer
  * @retval None
  */
void HAL_CACHEAXI_MspInit(CACHEAXI_HandleTypeDef *hcacheaxi)
{
  (void)hcacheaxi;
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
  __HAL_RCC_CACHEAXI_RELEASE_RESET();
}

void HAL_CACHEAXI_MspDeInit(CACHEAXI_HandleTypeDef *hcacheaxi)
{
  (void)hcacheaxi;
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
}
