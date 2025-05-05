#ifndef __PY25Q64HA_DRIVER__
#define __PY25Q64HA_DRIVER__

#include "stm32u3xx_hal.h"

/*
 * **********************
 * PY25Q64HA Commands List
 * ***********************
 */

/* Command set (Standard/Dual/Quad SPI) ---------------------------------- */
/* Read */
#define PY25Q_FREAD    0x0BU /* n bytes read out until CS# goes high */
#define PY25Q_READ     0x03U /* n bytes read out until CS# goes high */
#define PY25Q_DREAD    0x3BU /* n bytes read out by Dual output */
#define PY25Q_2READ    0xBBU /* n bytes read out by 2IO */
#define PY25Q_QREAD    0x6BU /* n bytes read out by Quad output */
#define PY25Q_4READ    0xEBU /* n bytes read out by 4IO */
#define PY25Q_WREAD    0xE7U /* n bytes word read out by 4IO */
/* Program and Erase */
#define PY25Q_SE       0x20U /* erase selected sector */
#define PY25Q_BE32     0x52U /* erase selected 32K block */
#define PY25Q_BE       0xD8U /* erase selected 64K block */
#define PY25Q_CE       0x60U /* erase whole chip */
#define PY25Q_PP       0x02U /* program selected page */
#define PY25Q_QPP      0x32U /* quad input to program selecte page */
#define PY25Q_PES      0x75U /* suspend program/erase operation */
#define PY25Q_PER      0x7AU /* continue program/erase operation */
/* Protection */
#define PY25Q_WREN     0x06U /* Sets the write enable latch bit */
#define PY25Q_WRDI     0x04U /* Resets the write enable latch bit */
#define PY25Q_VWREN    0x50U /* Write enable for volatile SR/CR */
#define PY25Q_SBLK     0x36U /* Individual block lock */
#define PY25Q_SBULK    0x39U /* Individual block unlock */
#define PY25Q_RDBLOCK  0x3DU /* Read individual block lock register */
#define PY25Q_GBLK     0x7EU /* Whole chip block protect */
#define PY25Q_GBULK    0x98U /* Whole chip block unprotect */
/* Security */
#define PY25Q_ERSCUR   0x44U /* Erase security registers */
#define PY25Q_PRSCUR   0x42U /* Program security registers */
#define PY25Q_RDSCUR   0x48U /* Read value of security register */
/* Status Register */
#define PY25Q_RDSR     0x05U /* read out status register */
#define PY25Q_RDSR1    0x35U /* Read out status register-1 */
#define PY25Q_RDCR     0x15U /* Read out configure register */
#define PY25Q_WRSR     0x01U /* Write data to status registers and status register-1 */
#define PY25Q_WRSR1    0x31U /* Write data to status register-1 */
#define PY25Q_WRCR     0x11U /* Write data to configuration register */
/* Other Commands */
#define PY25Q_RSTEN    0x66U /* Enable reset */
#define PY25Q_RST      0x99U /* Reset */
#define PY25Q_QPIEN    0x38U /* Enable QPI mode */
#define PY25Q_RDID     0x9FU /* output JEDEC ID: 1-byte manufacturer ID & 2-byte device ID */
#define PY25Q_REMS     0x90U /* Read manufacturer ID/device ID data */
#define PY25Q_DREMS    0x92U /* Dual output read manufacture/device ID */
#define PY25Q_QREMS    0x94U /* Quad output read manufacture/device ID */
#define PY25Q_DP       0xB9U /* enters deep power-down mode */
#define PY25Q_RDP_RES  0xABU /* Read electronics ID data */
#define PY25Q_SBL      0x77U /* Set burst length */
#define PY25Q_RDSFDP   0x5AU /* Read SFDP parameter */
#define PY25Q_RRE      0xFFU /* Release from read enhanced */
#define PY25Q_RUID     0x4BU /* Read unique ID */

/* Command set (DTR Instructions) ---------------------------------- */
#define PY25Q_DTRFRD   0x0DU /* DTR n byte fast read out */
#define PY25Q_2DTRD    0xBDU /* DTR n byte read out by 2IO */
#define PY25Q_4DTRD    0xEDU /* DTR n byte read out by 4IO */

/* Command set (QPI Instructions) ---------------------------------- */
/* Read */
#define PY25Q_QPI_FREAD         0x0BU /* n bytes read out until CS# goes high */
#define PY25Q_QPI_BREAD_WRAP    0x0CU /* n bytes burst read with wrap by 4IO */
#define PY25Q_QPI_4READ         0xEBU /* n bytes read out by 4IO */
/* Program and Erase */
#define PY25Q_QPI_PP            0x02U /* program selected page */
#define PY25Q_QPI_SE            0x20U /* erase selected sector */
#define PY25Q_QPI_BE32          0x52U /* erase seelcted 32K block */
#define PY25Q_QPI_BE            0xD8U /* erase selected 64K block */
#define PY25Q_QPI_CE            0xC7U /* erase whole chip */
#define PY25Q_QPI_PES           0x75U /* suspend program/erase operation */
#define PY25Q_QPI_PER           0x7AU /* continue program/erase operation */
/* Protection */
#define PY25Q_QPI_WREN          0x06U /* sets the write enable latch bit */
#define PY25Q_QPI_VWREN         0x50U /* Write enable for volatile SR/CR */
#define PY25Q_QPI_WRDI          0x04U /* resets the write enable latch bit */
#define PY25Q_QPI_SBLK          0x36U /* Individual block lock */
#define PY25Q_QPI_SBULK         0x39U /* Individual block unlick */
#define PY25Q_QPI_RDBLOCK       0x3DU /* Read indivisual block lock register */
#define PY25Q_QPI_GBLK          0x7EU /* Whole chip block protect */
#define PY25Q_QPI_GBULK         0x98U /* Whole chip block unprotect */
/* Status Register */
#define PY25Q_QPI_RDSR          0x05U /* read out status register */
#define PY25Q_QPI_RDSR1         0x35U /* read out status register-1 */
#define PY25Q_QPI_RDCR          0x15U /* Read out configure register */
#define PY25Q_QPI_WRSR          0x01U /* Write data to status registers-0 and status register-1 */
#define PY25Q_QPI_WRSR1         0x31U /* Write data to status registers-1 */
#define PY25Q_QPI_WRCR          0x11U /* Wrtie data to configuration register */
/* Other Commands */
#define PY25Q_QPI_DP            0xB9U /* Enters deep power-down mode */
#define PY25Q_QPI_RDP_RES       0xABU /* Read electronic ID data */
#define PY25Q_QPI_SRP           0xC0U /* Set read dummy and wrap */
#define PY25Q_QPI_REMS          0x90U /* Read manufacturer ID/device ID data */
#define PY25Q_QPI_RDID          0x9FU /* output JEDEC ID: 1-byte manufacturer ID & 2-byte device ID */
#define PY25Q_QPI_RDSFDP        0x5AU /* Read SFDP parameter */
#define PY25Q_QPI_QPIDI         0xFFU /* Release from read enhanced */
#define PY25Q_QPI_RSTEN         0x66U /* Enable reset */
#define PY25Q_QPI_RST           0x99U /* Reset */

/* Command set (DTR QPI Instructions) ---------------------------------- */
#define PY25Q_QPI_DTRBREAD_WRAP    0x0EU /* DTR n bytes burst read with wrap by 4IO */
#define PY25Q_QPI_DTRFRD           0x0DU /* DTR n byte fast read out */
#define PY25Q_QPI_4DTRD            0xEDU /* DTR n byte fast read out */

/* Status regsiter mask */
#define PY25Q_RDSR_MASK    0x01U
#define PY25Q_WIP_MASK     (PY25Q_RDSR_MASK)
#define PY25Q_WEL_MASK     (PY25Q_RDSR_MASK << 1)
#define PY25Q_BP0_MASK     (PY25Q_RDSR_MASK << 2)
#define PY25Q_BP1_MASK     (PY25Q_RDSR_MASK << 3)
#define PY25Q_BP2_MASK     (PY25Q_RDSR_MASK << 4)
#define PY25Q_BP3_MASK     (PY25Q_RDSR_MASK << 5)
#define PY25Q_BP4_MASK     (PY25Q_RDSR_MASK << 6)
#define PY25Q_SRP0_MASK    (PY25Q_RDSR_MASK << 7)

#define PY25Q_RDSR1_MASK   0x01U
#define PY25Q_SRP1_MASK    (PY25Q_RDSR1_MASK)
#define PY25Q_QE_MASK      (PY25Q_RDSR1_MASK << 1)
#define PY25Q_EP_FAIL_MASK (PY25Q_RDSR1_MASK << 2)
#define PY25Q_LB1_MASK     (PY25Q_RDSR1_MASK << 3)
#define PY25Q_LB2_MASK     (PY25Q_RDSR1_MASK << 4)
#define PY25Q_LB3_MASK     (PY25Q_RDSR1_MASK << 5)
#define PY25Q_CMP_MASK     (PY25Q_RDSR1_MASK << 6)
#define PY25Q_SUS_MASK     (PY25Q_RDSR1_MASK << 7)

/* PY25Q64HA Device ID */
#define PY25Q64_DEVICE_ID    0x16
/* PY25Q64HA Manufacturer ID */
#define PY25Q64_MANUFACTURER_ID    0x85

/* PY25Q64HA Chip's parameters */
#define MEM_FLASH_SIZE     64U
#define MEM_BLOCK_SIZE     64U
#define MEM_SECTOR_SIZE    4U
#define MEM_PAGE_SIZE      256U
#define MEM_BLOCK_COUNT    127U
#define MEM_SECTOR_COUNT   (BLOCK_COUNT * 16U)
#define MEM_PAGE_COUNT     (SECTOR_COUNT * 16U)

typedef enum{
  PY25Q64_OK          = 0x0,
  PY25Q64_BUSY        = 0x01,
  PY25Q64_PARAM_ERROR = 0x02,
  PY25Q64_CHIP_ERR    = 0x03,
  PY25Q64_SPI_ERR     = 0x04,
  PY25Q64_CHIP_IGNORE = 0x05
}PY25Q64_STATE;

typedef enum{
  QUAD_RESET = 0U,
  QUAD_SET = !QUAD_RESET
}QuadFlag;

#define UPDATE_STATUS_REGISTER(val, obj, bit) \
  ((obj) = ((val) & (1U << (bit)))) ? QUAD_SET : QUAD_RESET)

PY25Q64_STATE PY25Q64_Init(void);
PY25Q64_STATE PY25Q64_QPIInitConfiguration(void);
PY25Q64_STATE PY25Q64_MemoryMappedMode(void);
PY25Q64_STATE PY25Q64_QPI_MemoryMappedMode(void);
PY25Q64_STATE PY25Q64_QPI_Program(uint8_t* pData, uint16_t len, uint32_t rawAddr);
PY25Q64_STATE PY25Q64_QPI_MassErase(void);
PY25Q64_STATE PY25Q64_QPI_BlockErase(uint32_t BlockAddr);
PY25Q64_STATE PY25Q64_QPI_SetReadParameter(void);
PY25Q64_STATE PY25Q64_QPI_WriteEnable(void);
PY25Q64_STATE PY25Q64_QPI_WriteDisable(void);
PY25Q64_STATE PY25Q64_QPI_ReadStatus0Register(uint8_t *reg);
PY25Q64_STATE PY25Q64_QPI_ReadStatus1Register(uint8_t *reg);
PY25Q64_STATE PY25Q64_QPI_AutoPollingMemReady(void);
PY25Q64_STATE PY25Q63_QPI_IsBusy(void);
uint8_t PY25Q64_QPI_ReadID(void);

PY25Q64_STATE PY25Q64_MemoryMappedMode(void);
PY25Q64_STATE PY25Q64_Program(uint8_t* pData, uint16_t len, uint32_t rawAddr);
PY25Q64_STATE PY25Q64_QPIEnable(void);
PY25Q64_STATE PY25Q64_QPIDisable(void);
PY25Q64_STATE PY25Q64_WriteEnable(void);
PY25Q64_STATE PY25Q64_WriteDisable(void);
PY25Q64_STATE PY25Q64_QuadModeEnable(void);
PY25Q64_STATE PY25Q64_MassErase(void);
PY25Q64_STATE PY25Q64_WriteStatus1Register(uint8_t* pData);
PY25Q64_STATE PY25Q64_AutoPollingMemReady(void);
PY25Q64_STATE PY25Q64_ResetMemory(void);
PY25Q64_STATE PY25Q64_ReadStatus0Register(uint8_t *reg);
PY25Q64_STATE PY25Q64_ReadStatus1Register(uint8_t *reg);
PY25Q64_STATE PY25Q63_IsBusy(void);
uint8_t PY25Q64_ReadID(void);


#endif /* __PY25Q64HA_DRIVER__ */
