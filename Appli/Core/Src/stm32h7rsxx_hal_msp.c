/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32h7rsxx_hal_msp.c
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef handle_HPDMA1_Channel1;

extern DMA_HandleTypeDef handle_HPDMA1_Channel0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /* Enable USB Voltage detector */
    if (HAL_PWREx_EnableUSBVoltageDetector() != HAL_OK) {
        /* Initialization error */
        Error_Handler();
    }

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
 * @brief CRC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc) {
    if (hcrc->Instance == CRC) {
        /* USER CODE BEGIN CRC_MspInit 0 */

        /* USER CODE END CRC_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_CRC_CLK_ENABLE();
        /* USER CODE BEGIN CRC_MspInit 1 */

        /* USER CODE END CRC_MspInit 1 */
    }
}

/**
 * @brief CRC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc) {
    if (hcrc->Instance == CRC) {
        /* USER CODE BEGIN CRC_MspDeInit 0 */

        /* USER CODE END CRC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CRC_CLK_DISABLE();
        /* USER CODE BEGIN CRC_MspDeInit 1 */

        /* USER CODE END CRC_MspDeInit 1 */
    }
}

/**
 * @brief DMA2D MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hdma2d: DMA2D handle pointer
 * @retval None
 */
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d) {
    if (hdma2d->Instance == DMA2D) {
        /* USER CODE BEGIN DMA2D_MspInit 0 */

        /* USER CODE END DMA2D_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_DMA2D_CLK_ENABLE();
        /* DMA2D interrupt Init */
        HAL_NVIC_SetPriority(DMA2D_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2D_IRQn);
        /* USER CODE BEGIN DMA2D_MspInit 1 */

        /* USER CODE END DMA2D_MspInit 1 */
    }
}

/**
 * @brief DMA2D MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hdma2d: DMA2D handle pointer
 * @retval None
 */
void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d) {
    if (hdma2d->Instance == DMA2D) {
        /* USER CODE BEGIN DMA2D_MspDeInit 0 */

        /* USER CODE END DMA2D_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DMA2D_CLK_DISABLE();

        /* DMA2D interrupt DeInit */
        HAL_NVIC_DisableIRQ(DMA2D_IRQn);
        /* USER CODE BEGIN DMA2D_MspDeInit 1 */

        /* USER CODE END DMA2D_MspDeInit 1 */
    }
}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

/**
 * @brief FDCAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hfdcan->Instance == FDCAN1) {
        /* USER CODE BEGIN FDCAN1_MspInit 0 */

        /* USER CODE END FDCAN1_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**FDCAN1 GPIO Configuration
        PA12     ------> FDCAN1_TX
        PD0     ------> FDCAN1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* FDCAN1 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
        /* USER CODE BEGIN FDCAN1_MspInit 1 */

        /* USER CODE END FDCAN1_MspInit 1 */
    } else if (hfdcan->Instance == FDCAN2) {
        /* USER CODE BEGIN FDCAN2_MspInit 0 */

        /* USER CODE END FDCAN2_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* FDCAN2 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
        /* USER CODE BEGIN FDCAN2_MspInit 1 */

        /* USER CODE END FDCAN2_MspInit 1 */
    }
}

/**
 * @brief FDCAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan) {
    if (hfdcan->Instance == FDCAN1) {
        /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

        /* USER CODE END FDCAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN1 GPIO Configuration
        PA12     ------> FDCAN1_TX
        PD0     ------> FDCAN1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0);

        /* FDCAN1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
        /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

        /* USER CODE END FDCAN1_MspDeInit 1 */
    } else if (hfdcan->Instance == FDCAN2) {
        /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

        /* USER CODE END FDCAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

        /* FDCAN2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
        /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

        /* USER CODE END FDCAN2_MspDeInit 1 */
    }
}

/**
 * @brief GFXMMU MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hgfxmmu: GFXMMU handle pointer
 * @retval None
 */
void HAL_GFXMMU_MspInit(GFXMMU_HandleTypeDef* hgfxmmu) {
    if (hgfxmmu->Instance == GFXMMU) {
        /* USER CODE BEGIN GFXMMU_MspInit 0 */

        /* USER CODE END GFXMMU_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_GFXMMU_CLK_ENABLE();
        /* USER CODE BEGIN GFXMMU_MspInit 1 */

        /* USER CODE END GFXMMU_MspInit 1 */
    }
}

/**
 * @brief GFXMMU MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hgfxmmu: GFXMMU handle pointer
 * @retval None
 */
void HAL_GFXMMU_MspDeInit(GFXMMU_HandleTypeDef* hgfxmmu) {
    if (hgfxmmu->Instance == GFXMMU) {
        /* USER CODE BEGIN GFXMMU_MspDeInit 0 */

        /* USER CODE END GFXMMU_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_GFXMMU_CLK_DISABLE();
        /* USER CODE BEGIN GFXMMU_MspDeInit 1 */

        /* USER CODE END GFXMMU_MspDeInit 1 */
    }
}

/**
 * @brief GPU2D MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hgpu2d: GPU2D handle pointer
 * @retval None
 */
void HAL_GPU2D_MspInit(GPU2D_HandleTypeDef* hgpu2d) {
    if (hgpu2d->Instance == GPU2D) {
        /* USER CODE BEGIN GPU2D_MspInit 0 */

        /* USER CODE END GPU2D_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_GPU2D_CLK_ENABLE();
        /* GPU2D interrupt Init */
        HAL_NVIC_SetPriority(GPU2D_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(GPU2D_IRQn);
        HAL_NVIC_SetPriority(GPU2D_ER_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(GPU2D_ER_IRQn);
        /* USER CODE BEGIN GPU2D_MspInit 1 */

        /* USER CODE END GPU2D_MspInit 1 */
    }
}

/**
 * @brief GPU2D MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hgpu2d: GPU2D handle pointer
 * @retval None
 */
void HAL_GPU2D_MspDeInit(GPU2D_HandleTypeDef* hgpu2d) {
    if (hgpu2d->Instance == GPU2D) {
        /* USER CODE BEGIN GPU2D_MspDeInit 0 */

        /* USER CODE END GPU2D_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_GPU2D_CLK_DISABLE();

        /* GPU2D interrupt DeInit */
        HAL_NVIC_DisableIRQ(GPU2D_IRQn);
        HAL_NVIC_DisableIRQ(GPU2D_ER_IRQn);
        /* USER CODE BEGIN GPU2D_MspDeInit 1 */

        /* USER CODE END GPU2D_MspDeInit 1 */
    }
}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hi2c->Instance == I2C1) {
        /* USER CODE BEGIN I2C1_MspInit 0 */

        /* USER CODE END I2C1_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1_I3C1;
        PeriphClkInit.I2c1_I3c1ClockSelection = RCC_I2C1_I3C1CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C1 GPIO Configuration
        PB7     ------> I2C1_SDA
        PB8     ------> I2C1_SCL
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();
        /* USER CODE BEGIN I2C1_MspInit 1 */

        /* USER CODE END I2C1_MspInit 1 */
    }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        /* USER CODE BEGIN I2C1_MspDeInit 0 */

        /* USER CODE END I2C1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C1_CLK_DISABLE();

        /**I2C1 GPIO Configuration
        PB7     ------> I2C1_SDA
        PB8     ------> I2C1_SCL
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

        /* USER CODE BEGIN I2C1_MspDeInit 1 */

        /* USER CODE END I2C1_MspDeInit 1 */
    }
}

/**
 * @brief JPEG MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hjpeg: JPEG handle pointer
 * @retval None
 */
void HAL_JPEG_MspInit(JPEG_HandleTypeDef* hjpeg) {
    DMA_DataHandlingConfTypeDef DataHandlingConfig;
    if (hjpeg->Instance == JPEG) {
        /* USER CODE BEGIN JPEG_MspInit 0 */

        /* USER CODE END JPEG_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_JPEG_CLK_ENABLE();

        /* JPEG DMA Init */
        /* HPDMA1_REQUEST_JPEG_TX Init */
        handle_HPDMA1_Channel1.Instance = HPDMA1_Channel1;
        handle_HPDMA1_Channel1.Init.Request = HPDMA1_REQUEST_JPEG_TX;
        handle_HPDMA1_Channel1.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
        handle_HPDMA1_Channel1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        handle_HPDMA1_Channel1.Init.SrcInc = DMA_SINC_FIXED;
        handle_HPDMA1_Channel1.Init.DestInc = DMA_DINC_INCREMENTED;
        handle_HPDMA1_Channel1.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
        handle_HPDMA1_Channel1.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
        handle_HPDMA1_Channel1.Init.Priority = DMA_HIGH_PRIORITY;
        handle_HPDMA1_Channel1.Init.SrcBurstLength = 8;
        handle_HPDMA1_Channel1.Init.DestBurstLength = 8;
        handle_HPDMA1_Channel1.Init.TransferAllocatedPort =
            DMA_SRC_ALLOCATED_PORT1 | DMA_DEST_ALLOCATED_PORT0;
        handle_HPDMA1_Channel1.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_HPDMA1_Channel1.Init.Mode = DMA_NORMAL;
        if (HAL_DMA_Init(&handle_HPDMA1_Channel1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hjpeg, hdmaout, handle_HPDMA1_Channel1);

        if (HAL_DMA_ConfigChannelAttributes(&handle_HPDMA1_Channel1, DMA_CHANNEL_NPRIV) !=
            HAL_OK) {
            Error_Handler();
        }

        /* HPDMA1_REQUEST_JPEG_RX Init */
        handle_HPDMA1_Channel0.Instance = HPDMA1_Channel0;
        handle_HPDMA1_Channel0.Init.Request = HPDMA1_REQUEST_JPEG_RX;
        handle_HPDMA1_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
        handle_HPDMA1_Channel0.Init.Direction = DMA_MEMORY_TO_PERIPH;
        handle_HPDMA1_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
        handle_HPDMA1_Channel0.Init.DestInc = DMA_DINC_FIXED;
        handle_HPDMA1_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
        handle_HPDMA1_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
        handle_HPDMA1_Channel0.Init.Priority = DMA_HIGH_PRIORITY;
        handle_HPDMA1_Channel0.Init.SrcBurstLength = 8;
        handle_HPDMA1_Channel0.Init.DestBurstLength = 8;
        handle_HPDMA1_Channel0.Init.TransferAllocatedPort =
            DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
        handle_HPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_HPDMA1_Channel0.Init.Mode = DMA_NORMAL;
        if (HAL_DMA_Init(&handle_HPDMA1_Channel0) != HAL_OK) {
            Error_Handler();
        }

        DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
        DataHandlingConfig.DataAlignment = DMA_DATA_PACK;
        if (HAL_DMAEx_ConfigDataHandling(&handle_HPDMA1_Channel0, &DataHandlingConfig) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hjpeg, hdmain, handle_HPDMA1_Channel0);

        if (HAL_DMA_ConfigChannelAttributes(&handle_HPDMA1_Channel0, DMA_CHANNEL_NPRIV) !=
            HAL_OK) {
            Error_Handler();
        }

        /* JPEG interrupt Init */
        HAL_NVIC_SetPriority(JPEG_IRQn, 8, 0);
        HAL_NVIC_EnableIRQ(JPEG_IRQn);
        /* USER CODE BEGIN JPEG_MspInit 1 */

        /* USER CODE END JPEG_MspInit 1 */
    }
}

/**
 * @brief JPEG MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hjpeg: JPEG handle pointer
 * @retval None
 */
void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef* hjpeg) {
    if (hjpeg->Instance == JPEG) {
        /* USER CODE BEGIN JPEG_MspDeInit 0 */

        /* USER CODE END JPEG_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_JPEG_CLK_DISABLE();

        /* JPEG DMA DeInit */
        HAL_DMA_DeInit(hjpeg->hdmaout);
        HAL_DMA_DeInit(hjpeg->hdmain);

        /* JPEG interrupt DeInit */
        HAL_NVIC_DisableIRQ(JPEG_IRQn);
        /* USER CODE BEGIN JPEG_MspDeInit 1 */

        /* USER CODE END JPEG_MspDeInit 1 */
    }
}

/**
 * @brief LTDC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hltdc: LTDC handle pointer
 * @retval None
 */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hltdc->Instance == LTDC) {
        /* USER CODE BEGIN LTDC_MspInit 0 */

        /* USER CODE END LTDC_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
        PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLL3R;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_LTDC_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        /**LTDC GPIO Configuration
        PF7     ------> LTDC_G0
        PF8     ------> LTDC_G1
        PF9     ------> LTDC_R0
        PF10     ------> LTDC_R1
        PA0     ------> LTDC_G3
        PA2     ------> LTDC_B7
        PA3     ------> LTDC_DE
        PA5     ------> LTDC_CLK
        PA7     ------> LTDC_R4
        PB1     ------> LTDC_G2
        PB2     ------> LTDC_B2
        PF11     ------> LTDC_B0
        PB10     ------> LTDC_G7
        PB11     ------> LTDC_G6
        PB12     ------> LTDC_G5
        PB13     ------> LTDC_G4
        PA8     ------> LTDC_B6
        PA9     ------> LTDC_B5
        PA10     ------> LTDC_B4
        PA11     ------> LTDC_B3
        PA15(JTDI)     ------> LTDC_R5
        PE11     ------> LTDC_VSYNC
        PD3     ------> LTDC_B1
        PG0     ------> LTDC_R7
        PG1     ------> LTDC_R6
        PG2     ------> LTDC_HSYNC
        PF0     ------> LTDC_R2
        PB4(NJTRST)     ------> LTDC_R3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_LTDC;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        GPIO_InitStruct.Pin =
            GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_LTDC;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_LTDC;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_LTDC;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_LTDC;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_LTDC;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LTDC;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* LTDC interrupt Init */
        HAL_NVIC_SetPriority(LTDC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(LTDC_IRQn);
        /* USER CODE BEGIN LTDC_MspInit 1 */

        /* USER CODE END LTDC_MspInit 1 */
    }
}

/**
 * @brief LTDC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hltdc: LTDC handle pointer
 * @retval None
 */
void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc) {
    if (hltdc->Instance == LTDC) {
        /* USER CODE BEGIN LTDC_MspDeInit 0 */

        /* USER CODE END LTDC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_LTDC_CLK_DISABLE();

        /**LTDC GPIO Configuration
        PF7     ------> LTDC_G0
        PF8     ------> LTDC_G1
        PF9     ------> LTDC_R0
        PF10     ------> LTDC_R1
        PA0     ------> LTDC_G3
        PA2     ------> LTDC_B7
        PA3     ------> LTDC_DE
        PA5     ------> LTDC_CLK
        PA7     ------> LTDC_R4
        PB1     ------> LTDC_G2
        PB2     ------> LTDC_B2
        PF11     ------> LTDC_B0
        PB10     ------> LTDC_G7
        PB11     ------> LTDC_G6
        PB12     ------> LTDC_G5
        PB13     ------> LTDC_G4
        PA8     ------> LTDC_B6
        PA9     ------> LTDC_B5
        PA10     ------> LTDC_B4
        PA11     ------> LTDC_B3
        PA15(JTDI)     ------> LTDC_R5
        PE11     ------> LTDC_VSYNC
        PD3     ------> LTDC_B1
        PG0     ------> LTDC_R7
        PG1     ------> LTDC_R6
        PG2     ------> LTDC_HSYNC
        PF0     ------> LTDC_R2
        PB4(NJTRST)     ------> LTDC_R3
        */
        HAL_GPIO_DeInit(
            GPIOF, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_0);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7 |
                                   GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                                   GPIO_PIN_15);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                                   GPIO_PIN_13 | GPIO_PIN_4);

        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3);

        HAL_GPIO_DeInit(GPIOG, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

        /* LTDC interrupt DeInit */
        HAL_NVIC_DisableIRQ(LTDC_IRQn);
        /* USER CODE BEGIN LTDC_MspDeInit 1 */

        /* USER CODE END LTDC_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
