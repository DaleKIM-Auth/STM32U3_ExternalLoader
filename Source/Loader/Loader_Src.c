/**
******************************************************************************
* @file    Loader_Src.c
* @author  MCD Application Team
* @brief   This file defines the operations of the external loader for
*          MT25QL128A QSPI memory of STM32F469I-DISCO.
*           
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

#include "Loader_Src.h"
#include <string.h>

#pragma section=".bss"

/* Private variables ---------------------------------------------------------*/
XSPI_HandleTypeDef hxspi1;

/* Private functions ---------------------------------------------------------*/
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef* hxspi);
static void MX_GPIO_Init(void);
static int MX_OCTOSPI1_Init(void);
static void MX_OCTOSPI1_DeInit(void);
static void MX_ICACHE_Init(void);

/* USER CODE END PFP */
#if 0
KeepInCompilation HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{ 
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return 1;
}
#else
KeepInCompilation HAL_StatusTypeDef HAL_InitTick(uint32_t HAL_InitTick)
{ 
  (void)HAL_InitTick;

  /* DWT initialization */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  
  return HAL_OK;
}

KeepInCompilation uint32_t HAL_GetTick(void)
{
  uint32_t cycles_per_ms = SystemCoreClock / 1000000;  
  uwTick = (DWT->CYCCNT) / cycles_per_ms;
  
  return uwTick;
}
#endif
/**
 * @brief  System initialization.
 * @param  None
 * @retval  1      : Operation succeeded
 * @retval  0      : Operation failed
 */
int Init()
{
  int32_t result=0;

  __disable_irq();
  
  /* Init structs to Zero */
  char *startadd = __section_begin(".bss");
  uint32_t size = __section_size(".bss");
  memset(startadd, 0, size);

  /* init system */
  SystemInit();   
  /* Tick init */
  HAL_InitTick(0);
  
  /* Configure the system clock  */
  result = SystemClockInit();  
  if(result != 1){
    return 0;
  }
  
  /* Initialize peripherals */
  MX_ICACHE_Init();
  
  /* QuadSPI De-Init */
  MX_OCTOSPI1_DeInit();

  /* QuadSPI Init */
  MX_OCTOSPI1_Init();

  if(PY25Q64_Init() != PY25Q64_OK){
      return 0;
  }

  /* Set Memory Mapped Mode */
  result = PY25Q64_MemoryMappedMode();

  if(result != PY25Q64_OK){
    return 0;
  }
  
  __enable_irq();
  
  return 1;
}

/**
 * @brief   Program memory.
 * @param   Address: page address
 * @param   Size   : size of data
 * @param   buffer : pointer to data buffer
 * @retval  1      : Operation succeeded
 * @retval  0      : Operation failed
 */
KeepInCompilation int Write (uint32_t Address, uint32_t Size, uint8_t* buffer)
{
  __disable_irq();

  Address = Address & 0x0FFFFFFF;

  /* QuadSPI De-Init */
  MX_OCTOSPI1_DeInit();

  /* QuadSPI Init */
  MX_OCTOSPI1_Init();

  /* Writes an amount of data to the QSPI memory */
  if(PY25Q64_QPI_Program(buffer, Size, Address) != PY25Q64_OK){
    return 0;
  }

  __enable_irq();

  return 1;
}

/**
 * @brief 	 Full erase of the device 						
 * @param 	 Parallelism : 0 																		
 * @retval  1           : Operation succeeded
 * @retval  0           : Operation failed											
 */
KeepInCompilation int MassErase (uint32_t Parallelism )
{ 

  /* QuadSPI De-Init */
  MX_OCTOSPI1_DeInit();

  /* QuadSPI Init */
  MX_OCTOSPI1_Init();

  /* Erase the entire QSPI memory */
  if (PY25Q64_MassErase() != PY25Q64_OK){
      return 0;
  }

   return 1;
}

/**
 * @brief   Sector erase.
 * @param   EraseStartAddress :  erase start address
 * @param   EraseEndAddress   :  erase end address
 * @retval  None
 */
KeepInCompilation int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress)
{
  __disable_irq();

  uint32_t BlockAddr = 0;
  EraseStartAddress &= 0x0FFFFFFF;
  EraseEndAddress &= 0x0FFFFFFF;
  EraseStartAddress = EraseStartAddress - EraseStartAddress % 0x10000;
    
  /* QuadSPI De-Init */
  MX_OCTOSPI1_DeInit();

  /* QuadSPI Init */
  MX_OCTOSPI1_Init();

  while(EraseEndAddress >= EraseStartAddress){
    BlockAddr = EraseStartAddress;

    /* Erases the specified block of the QSPI memory */
    PY25Q64_QPI_BlockErase(BlockAddr);

    /* Reads current status of the QSPI memory */
    EraseStartAddress += 0x10000;
  }
  
  __enable_irq();

  return 1;	
}


/**
 * Description :
 * Calculates checksum value of the memory zone
 * Inputs    :
 *      StartAddress  : Flash start address
 *      Size          : Size (in WORD)  
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Checksum value
 * Note: Optional for all types of device
 */
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal)
{
  return 1;
  
  __disable_irq();

  uint8_t missalignementAddress = StartAddress%4;
  uint8_t missalignementSize = Size ;
  int cnt;
  uint32_t Val;
	
  StartAddress-=StartAddress%4;
  Size += (Size%4==0)?0:4-(Size%4);
  
  for(cnt=0; cnt<Size ; cnt+=4)
  {
    Val = *(uint32_t*)StartAddress;
    if(missalignementAddress)
    {
      switch (missalignementAddress)
      {
      case 1:
        InitVal += (uint8_t) (Val>>8 & 0xff);
        InitVal += (uint8_t) (Val>>16 & 0xff);
        InitVal += (uint8_t) (Val>>24 & 0xff);
        missalignementAddress-=1;
        break;
      case 2:
        InitVal += (uint8_t) (Val>>16 & 0xff);
        InitVal += (uint8_t) (Val>>24 & 0xff);
        missalignementAddress-=2;
        break;
      case 3:   
        InitVal += (uint8_t) (Val>>24 & 0xff);
        missalignementAddress-=3;
        break;
      }  
    }
    else if((Size-missalignementSize)%4 && (Size-cnt) <=4)
    {
      switch (Size-missalignementSize)
      {
      case 1:
        InitVal += (uint8_t) Val;
        InitVal += (uint8_t) (Val>>8 & 0xff);
        InitVal += (uint8_t) (Val>>16 & 0xff);
        missalignementSize-=1;
        break;
      case 2:
        InitVal += (uint8_t) Val;
        InitVal += (uint8_t) (Val>>8 & 0xff);
        missalignementSize-=2;
        break;
      case 3:   
        InitVal += (uint8_t) Val;
        missalignementSize-=3;
        break;
      } 
    }
    else
    {
      InitVal += (uint8_t) Val;
      InitVal += (uint8_t) (Val>>8 & 0xff);
      InitVal += (uint8_t) (Val>>16 & 0xff);
      InitVal += (uint8_t) (Val>>24 & 0xff);
    }
    StartAddress+=4;
  }
  
  __enable_irq();

  return (InitVal);

}


/**
 * Description :
 * Verify flash memory with RAM buffer and calculates checksum value of
 * the programmed memory
 * Inputs    :
 *      FlashAddr     : Flash address
 *      RAMBufferAddr : RAM buffer address
 *      Size          : Size (in WORD)  
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Operation failed (address of failure)
 *     R1             : Checksum value
 * Note: Optional for all types of device
 */
KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement)
{  
  __disable_irq();

  uint32_t VerifiedData = 0, InitVal = 0;
  uint64_t checksum = 0;
  Size*=4;
  
  checksum = CheckSum((uint32_t)MemoryAddr + (missalignement & 0xf), Size - ((missalignement >> 16) & 0xF), InitVal);
  while (Size>VerifiedData)
  {
    if ( *(uint8_t*)MemoryAddr++ != *((uint8_t*)RAMBufferAddr + VerifiedData))
      return ((checksum<<32) + (MemoryAddr + VerifiedData));  
   
    VerifiedData++;  
  }
  
  __enable_irq();

  return (checksum<<32);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
int SystemClockInit(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  uint32_t msiclk = 0x0U;

  __HAL_RCC_PWR_CLK_ENABLE();

  /** Enable Epod Booster
  */
  if (HAL_RCCEx_EpodBoosterClkConfig(RCC_EPODBOOSTER_SOURCE_MSIS, RCC_EPODBOOSTER_DIV1) != HAL_OK)
  {
    return 0;
  }
  if (HAL_PWREx_EnableEpodBooster() != HAL_OK)
  {
    return 0;
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    return 0;
  }

  /** Set Flash latency before increasing MSIS
  */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSIS;
  RCC_OscInitStruct.MSISState = RCC_MSI_ON;
  RCC_OscInitStruct.MSISSource = RCC_MSI_RC0;
  RCC_OscInitStruct.MSISDiv = RCC_MSI_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return 0;
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSIS;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    return 0;
  }

  if (RCC_OscInitStruct.MSISSource == RCC_MSI_RC0){
    msiclk = 96000000U;
  } else {
    msiclk = 24000000U;
  }
  
  switch (RCC_OscInitStruct.MSISDiv){
  case RCC_MSI_DIV1:
    SystemCoreClock = msiclk;
    break;
  case RCC_MSI_DIV2:
    SystemCoreClock = msiclk / 2;
    break;
  case RCC_MSI_DIV4:
    SystemCoreClock = msiclk / 4;
    break;
  case RCC_MSI_DIV8:
    SystemCoreClock = msiclk / 8;
    break;
  default:
    SystemCoreClock = 24000000U;
  }

  return 1;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
 static int MX_OCTOSPI1_Init(void)
 {
 
   /* USER CODE BEGIN OCTOSPI1_Init 0 */
 
   /* USER CODE END OCTOSPI1_Init 0 */
 
   /* USER CODE BEGIN OCTOSPI1_Init 1 */
 
   /* USER CODE END OCTOSPI1_Init 1 */
   /* OCTOSPI1 parameter configuration*/
  hxspi1.Instance = OCTOSPI1;
  hxspi1.Init.FifoThresholdByte = 1;
  hxspi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi1.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_8MB;
  hxspi1.Init.ChipSelectHighTimeCycle = 1;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_3;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi1.Init.ClockPrescaler = 1;
  hxspi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi1.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hxspi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_8MB;
  hxspi1.Init.DelayBlockBypass = HAL_XSPI_DELAY_BLOCK_BYPASS;
  hxspi1.Init.MaxTran = 255;
  hxspi1.Init.Refresh = 0;
   if (HAL_XSPI_Init(&hxspi1) != HAL_OK)
   {
     return 0;
   }
   
  return 1; 
 }

static void MX_OCTOSPI1_DeInit(void)
{  

  if(hxspi1.State != HAL_XSPI_STATE_RESET){
    HAL_XSPI_DeInit(&hxspi1);
  }    
    
  HAL_XSPI_MspDeInit(&hxspi1);
    
  __HAL_RCC_OCTOSPI1_FORCE_RESET();
  __HAL_RCC_OCTOSPI1_RELEASE_RESET();
  __HAL_RCC_OCTOSPI1_CLK_DISABLE();  
}
/**
  * @brief XSPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspInit(XSPI_HandleTypeDef* hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
    /* USER CODE BEGIN OCTOSPI1_MspInit 0 */

    /* USER CODE END OCTOSPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OCTOSPI1;
    PeriphClkInit.Octospi1ClockSelection = RCC_OCTOSPICLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      while(1);
    }

    /* Peripheral clock enable */
    __HAL_RCC_OCTOSPI1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OCTOSPI1 GPIO Configuration
    PC12     ------> OCTOSPI1_IO1
    PC11     ------> OCTOSPI1_NCS
    PD2     ------> OCTOSPI1_IO2
    PC10     ------> OCTOSPI1_IO0
    PC6     ------> OCTOSPI1_IO3
    PB10     ------> OCTOSPI1_CLK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_10|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief XSPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef* hxspi)
{

    /* Peripheral clock disable */
    __HAL_RCC_OCTOSPI1_CLK_DISABLE();

    /**OCTOSPI1 GPIO Configuration
    PC12     ------> OCTOSPI1_IO1
    PC11     ------> OCTOSPI1_NCS
    PD2     ------> OCTOSPI1_IO2
    PC10     ------> OCTOSPI1_IO0
    PC6     ------> OCTOSPI1_IO3
    PB10     ------> OCTOSPI1_CLK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  HAL_ICACHE_Enable();
  
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
