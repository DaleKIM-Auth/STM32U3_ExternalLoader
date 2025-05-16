#include "py25q64ha.h"

extern XSPI_HandleTypeDef hxspi1;

PY25Q64_STATE PY25Q64_Init(void)
{
  uint8_t qe = 0;
 
  if (PY25Q64_QPIDisable() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }  
  
  /* Memory reset */
  if (PY25Q64_ResetMemory() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  PY25Q64_AutoPollingMemReady();

  /* Check memory activation */
  if(PY25Q64_ReadID() != PY25Q64_DEVICE_ID){
    return PY25Q64_CHIP_ERR;
  }

  /* Memory Write Enable */
  if(PY25Q64_WriteEnable() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  /* Quad mode enable */
  PY25Q64_ReadStatus1Register(&qe);
  
  if((qe&PY25Q_QE_MASK) != PY25Q_QE_MASK){
    PY25Q64_QuadModeEnable();
  }
 
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPIInitConfiguration(void)
{
  uint8_t qe = 0;
  uint8_t wel = 0;
  
  if (PY25Q64_QPIDisable() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  PY25Q64_ReadStatus0Register(&wel);
  PY25Q64_ReadStatus1Register(&qe);
  
  if((qe&PY25Q_QE_MASK) != PY25Q_QE_MASK){
    PY25Q64_QuadModeEnable();
  }
  
  PY25Q64_QPIEnable();
  
  
  if((wel&PY25Q_WEL_MASK) != PY25Q_WEL_MASK){
    PY25Q64_QPI_WriteEnable();
  }
  
  while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_Quad_MemoryMappedMode(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_MemoryMappedTypeDef MemMappedCfg = { 0 };
  uint8_t wel = 0;
  
  PY25Q64_ReadStatus0Register(&wel);
  if((wel&PY25Q_WEL_MASK) != PY25Q_WEL_MASK){
      PY25Q64_QPI_WriteEnable();
  }

  sCommands.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QPP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return PY25Q64_SPI_ERR;
  }

  sCommands.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QREAD;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 8U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  MemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
  MemMappedCfg.TimeoutPeriodClock = 0x50U;

  if (HAL_XSPI_MemoryMapped(&hxspi1, &MemMappedCfg) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_MemoryMappedMode(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_MemoryMappedTypeDef MemMappedCfg = { 0 };

  if (PY25Q64_QPIInitConfiguration() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_PP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return PY25Q64_SPI_ERR;
  }

  sCommands.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_FREAD;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 10U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  MemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
  MemMappedCfg.TimeoutPeriodClock = 0x50U;

  if (HAL_XSPI_MemoryMapped(&hxspi1, &MemMappedCfg) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_Program(uint8_t* pData, uint32_t len, uint32_t rawAddr)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint32_t endAddr = 0U;
  uint32_t curAddr = 0U;
  uint32_t curSize = 0U;  

  if (PY25Q64_QPIInitConfiguration() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  /* Calculation of the size between the write address and the end of the page */
  curSize = MEM_PAGE_SIZE - (rawAddr % MEM_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (curSize > len){
    curSize = len;
  }
  
  /* Initialize the address variables */
  curAddr = rawAddr;
  endAddr = rawAddr + len;
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_PP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  do{

   sCommands.Address = curAddr;
   sCommands.DataLength = curSize;  
    
    while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
    }    
    
    PY25Q64_QPI_WriteEnable();
    
    if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
      return PY25Q64_SPI_ERR;
    }

    if (HAL_XSPI_Transmit(&hxspi1, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
      return PY25Q64_SPI_ERR;
    }

    while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
    }
    
    curAddr += curSize;
    pData += curSize;
    curSize = ((curAddr + MEM_PAGE_SIZE) > endAddr) ? (endAddr - curAddr) : MEM_PAGE_SIZE;
    
  }while(curAddr < endAddr);

  return PY25Q64_OK;
}
    
PY25Q64_STATE PY25Q64_QPI_MassErase(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };  

  if (PY25Q64_QPIInitConfiguration() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_CE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;  
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_BlockErase(uint32_t BlockAddr)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint32_t RawAddr = 0;

  if (PY25Q64_QPIInitConfiguration() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  if(BlockAddr >= MEM_BLOCK_COUNT){
    return PY25Q64_PARAM_ERROR;
  }

  RawAddr = BlockAddr * MEM_BLOCK_SIZE * 1024U;
  
  while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
  }
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_BE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.Address = RawAddr;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  while(PY25Q63_QPI_IsBusy() == PY25Q64_BUSY){
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_SetReadParameter(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint8_t value = 32; // P5,P4 = (1, 0) -> 6 dummy cycles
                            
  /* Set Read parameters for read(EBh) dummy cycles */
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_SRP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Transmit(&hxspi1, &value, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  return PY25Q64_OK;
}


PY25Q64_STATE PY25Q64_QPI_WriteEnable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_WREN;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode =  HAL_XSPI_DATA_4_LINES;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = PY25Q_WEL_MASK;
  sConfig.MatchMask = PY25Q_WEL_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_WriteDisable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_WRDI;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode =  HAL_XSPI_DATA_4_LINES;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = 0x00;
  sConfig.MatchMask = PY25Q_WEL_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_ReadStatus0Register(uint8_t *reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Receive(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_ReadStatus1Register(uint8_t *reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_RDSR1;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Receive(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPI_AutoPollingMemReady(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.Instruction = PY25Q_QPI_RDSR;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  sCommands.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = 0U;
  sConfig.MatchMask = PY25Q_WIP_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10U;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_BUSY;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_WriteEnable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_WREN;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode =  HAL_XSPI_DATA_1_LINE;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = PY25Q_WEL_MASK;
  sConfig.MatchMask = PY25Q_WEL_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_WriteDisable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_WRDI;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode =  HAL_XSPI_DATA_1_LINE;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = 0x00;
  sConfig.MatchMask = PY25Q_WEL_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QuadModeEnable(void)
{  
  uint8_t value = 0x00;

  PY25Q64_ReadStatus1Register(&value);

  value |= PY25Q_QE_MASK;
  if (PY25Q64_WriteStatus1Register(&value) != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  if (PY25Q64_WriteDisable() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  /* QE bit verification */
  PY25Q64_ReadStatus1Register(&value);
   
  if((value & PY25Q_QE_MASK) != PY25Q_QE_MASK){
    return PY25Q64_CHIP_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPIEnable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QPIEN;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_QPIDisable(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.Instruction = PY25Q_QPI_QPIDI;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q63_QPI_IsBusy(void)
{
  uint8_t wip = 0;
  uint8_t busyReg  = 0;
  
  if(PY25Q64_QPI_ReadStatus0Register(&wip) != PY25Q64_OK){
    return PY25Q64_SPI_ERR;
  }

  busyReg = (wip & PY25Q_WIP_MASK);

  return busyReg ? PY25Q64_BUSY : PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_MemoryMappedMode(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_MemoryMappedTypeDef MemMappedCfg = { 0 };

  if(PY25Q64_WriteEnable() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }

  sCommands.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_PP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return PY25Q64_SPI_ERR;
  }

  sCommands.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QREAD;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DummyCycles = 8U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  MemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
  MemMappedCfg.TimeoutPeriodClock = 0x50U;

  if (HAL_XSPI_MemoryMapped(&hxspi1, &MemMappedCfg) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_Program(uint8_t* pData, uint32_t len, uint32_t rawAddr)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint32_t endAddr = 0U;
  uint32_t curAddr = 0U;
  uint32_t curSize = 0U;  

  /* Calculation of the size between the write address and the end of the page */
  curSize = MEM_PAGE_SIZE - (rawAddr % MEM_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (curSize > len){
    curSize = len;
  }
  
  /* Initialize the address variables */
  curAddr = rawAddr;
  endAddr = rawAddr + len;
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QPP;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  do{

   sCommands.Address = curAddr;
   sCommands.DataLength = curSize;  
    
    while(PY25Q63_IsBusy() == PY25Q64_BUSY){
    }

    PY25Q64_WriteEnable();

    if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
      return PY25Q64_SPI_ERR;
    }

    if (HAL_XSPI_Transmit(&hxspi1, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
      return PY25Q64_SPI_ERR;
    }

    while(PY25Q63_IsBusy() == PY25Q64_BUSY){
    }

    curAddr += curSize;
    pData += curSize;
    curSize = ((curAddr + MEM_PAGE_SIZE) > endAddr) ? (endAddr - curAddr) : MEM_PAGE_SIZE;

  }while(curAddr < endAddr);

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_AutoPollingMemReady(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  XSPI_AutoPollingTypeDef sConfig = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.Instruction = PY25Q_RDSR;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  sCommands.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  sConfig.MatchValue = 0x00;
  sConfig.MatchMask = PY25Q_WIP_MASK;
  sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
  sConfig.IntervalTime = 0x10;
  sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(&hxspi1, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_BUSY;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_MassErase(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };  

  while(PY25Q63_IsBusy() == PY25Q64_BUSY){
  }

  PY25Q64_WriteEnable(); 

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_CE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;  
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  while(PY25Q63_IsBusy() == PY25Q64_BUSY){
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_BlockErase(uint32_t BlockAddr)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint32_t RawAddr = 0;


  if(BlockAddr >= MEM_BLOCK_COUNT){
    return PY25Q64_PARAM_ERROR;
  }
    
  while(PY25Q63_IsBusy() == PY25Q64_BUSY){
  }

  RawAddr = BlockAddr * MEM_BLOCK_SIZE * 1024U;

  PY25Q64_WriteEnable();  

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_QPI_BE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.Address = RawAddr;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DataLength = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  while(PY25Q63_IsBusy() == PY25Q64_BUSY){
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_ResetMemory(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.Instruction = PY25Q_RSTEN;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DataMode = HAL_XSPI_DATA_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
      return PY25Q64_SPI_ERR;
  }    
  
  sCommands.Instruction = PY25Q_RST;
  
  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  while(PY25Q63_IsBusy() == PY25Q64_BUSY){
  }
  
  PY25Q64_Delay(12);
  
  return PY25Q64_OK;
}


PY25Q64_STATE PY25Q64_ReadStatus0Register(uint8_t *reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_RDSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Receive(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_WriteStatus0Register(uint8_t* reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_WRSR;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Transmit(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  if (PY25Q64_AutoPollingMemReady() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_ReadStatus1Register(uint8_t *reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_RDSR1;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Receive(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  return PY25Q64_OK;
}

PY25Q64_STATE PY25Q64_WriteStatus1Register(uint8_t* reg)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };

  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.Instruction = PY25Q_WRSR1;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_NONE;
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DummyCycles = 0U;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DataLength = 1U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }

  if (HAL_XSPI_Transmit(&hxspi1, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  if (PY25Q64_AutoPollingMemReady() != PY25Q64_OK){
    return PY25Q64_CHIP_ERR;
  }
  
  return PY25Q64_OK;
}

uint8_t PY25Q64_ReadID(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint8_t rems[2] = { 0x00 };
  uint8_t id  =  0x00;
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.Instruction = PY25Q_REMS;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_8_BITS;
  sCommands.Address = 0U;  
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DataMode = HAL_XSPI_DATA_1_LINE;
  sCommands.DataLength = 2U;
  sCommands.DummyCycles = 16U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  sCommands.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  if (HAL_XSPI_Receive(&hxspi1, rems, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  id = rems[1]; // first = manufacturer Id, second = Device Id
  
  return id;
}

uint8_t PY25Q64_QPI_ReadID(void)
{
  XSPI_RegularCmdTypeDef sCommands = { 0 };
  uint8_t rems[2] = { 0x00 };
  uint8_t id  =  0x00;
  
  sCommands.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommands.Instruction = PY25Q_REMS;
  sCommands.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  sCommands.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  sCommands.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
  sCommands.Address = 0U;  
  sCommands.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommands.DataMode = HAL_XSPI_DATA_4_LINES;
  sCommands.DataLength = 2U;
  sCommands.DummyCycles = 0U;
  sCommands.DQSMode = HAL_XSPI_DQS_DISABLE;
  sCommands.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(&hxspi1, &sCommands, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  if (HAL_XSPI_Receive(&hxspi1, rems, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
    return PY25Q64_SPI_ERR;
  }
  
  id = rems[1]; // first = manufacturer Id, second = Device Id
  
  return id;
}

PY25Q64_STATE PY25Q63_IsBusy(void)
{
  uint8_t wip = 0;
  uint8_t busyReg  = 0;
  
  if(PY25Q64_ReadStatus0Register(&wip) != PY25Q64_OK){
    return PY25Q64_SPI_ERR;
  }

  busyReg = (wip & PY25Q_WIP_MASK);

  return busyReg ? PY25Q64_BUSY : PY25Q64_OK;
}

void PY25Q64_Delay(uint32_t ms)
{
  uint32_t cycles_per_ms = SystemCoreClock / 1000000;
  uint32_t start = DWT->CYCCNT;
  uint32_t delay_cycles = ms * cycles_per_ms;

  while((DWT->CYCCNT - start) < delay_cycles){
    
  }
}

