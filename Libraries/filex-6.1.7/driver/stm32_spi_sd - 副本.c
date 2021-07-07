/**
 * stm32_spi_sd.c
 *
 * Version:     V1.0.0
 * Created on:  2021-06-19
 * Author:      Qiu Chengwei
 * Description: 参照stm3210c_eval_spi_sd.c(V5.0.2)修改
 *
 *
 * Copyright (c) 2021 Nanjing Zitai Xinghe Electronics Co., Ltd.
 * All rights reserved.
 *
 * 1 tab == 2 spaces!
 */

/**
 ******************************************************************************
 * @file    stm3210c_eval_spi_sd.c
 * @author  MCD Application Team
 * @version V5.0.2
 * @date    22-September-2016
 * @brief   This file provides a set of functions needed to manage the SPI SD
 *          Card memory mounted on STM3210C-EVAL board.
 *          It implements a high level communication layer for read and write
 *          from/to this memory. The needed STM32F1 hardware resources (SPI and
 *          GPIO) are defined in stm3210c_eval.h file, and the initialization is
 *          performed in SD_LowLevel_Init() function declared in stm3210c_eval.c
 *          file.
 *          You can easily tailor this driver to any other development board,
 *          by just adapting the defines for hardware resources and
 *          SD_LowLevel_Init() function.
 *
 *          ===================================================================
 *          Note:
 *           - This driver doesn't support SD High Capacity cards.
 *          ===================================================================
 *
 *          +-------------------------------------------------------+
 *          |                     Pin assignment                    |
 *          +-------------------------+---------------+-------------+
 *          |  STM32 SPI Pins         |     SD        |    Pin      |
 *          +-------------------------+---------------+-------------+
 *          | SD_SPI_CS_PIN           |   ChipSelect  |    1        |
 *          | SD_SPI_MOSI_PIN / MOSI  |   DataIn      |    2        |
 *          |                         |   GND         |    3 (0 V)  |
 *          |                         |   VDD         |    4 (3.3 V)|
 *          | SD_SPI_SCK_PIN / SCLK   |   Clock       |    5        |
 *          |                         |   GND         |    6 (0 V)  |
 *          | SD_SPI_MISO_PIN / MISO  |   DataOut     |    7        |
 *          +-------------------------+---------------+-------------+
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/** 头文件包含顺序(以空行分隔): 关联.h, C库, C++库(无扩展), 其它库, 本项目.h
 * Includes ------------------------------------------------------------------*/
#include "stm32_spi_sd.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Types, Constants, Macros, Variables ---------------------------------------*/
typedef struct {
  uint8_t r1;
  uint8_t r2;
  uint8_t r3;
  uint8_t r4;
  uint8_t r5;
} SD_CmdAnswer_typedef;

#define SD_DUMMY_BYTE            0xFF

#define SD_MAX_FRAME_LENGTH        17    /* Lenght = 16 + 1 */
#define SD_CMD_LENGTH               6

#define SD_MAX_TRY                100    /* Number of try */

#define SD_CSD_STRUCT_V1          0x2    /* CSD struct version V1 */
#define SD_CSD_STRUCT_V2          0x1    /* CSD struct version V2 */


/**
  * @brief  SD ansewer format
  */
typedef enum {
 SD_ANSWER_R1_EXPECTED,
 SD_ANSWER_R1B_EXPECTED,
 SD_ANSWER_R2_EXPECTED,
 SD_ANSWER_R3_EXPECTED,
 SD_ANSWER_R4R5_EXPECTED,
 SD_ANSWER_R7_EXPECTED,
}SD_Answer_type;

/**
  * @brief  Start Data tokens:
  *         Tokens (necessary because at nop/idle (and CS active) only 0xff is
  *         on the data/command line)
  */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_READ    0xFE  /* Data token start byte, Start Single Block Read */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_READ  0xFE  /* Data token start byte, Start Multiple Block Read */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE   0xFE  /* Data token start byte, Start Single Block Write */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE 0xFD  /* Data token start byte, Start Multiple Block Write */
#define SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE  0xFD  /* Data toke stop byte, Stop Multiple Block Write */

/**
  * @brief  SD reponses and error flags
  */
typedef enum
{
/* R1 answer value */
  SD_R1_NO_ERROR            = (0x00),
  SD_R1_IN_IDLE_STATE       = (0x01),
  SD_R1_ERASE_RESET         = (0x02),
  SD_R1_ILLEGAL_COMMAND     = (0x04),
  SD_R1_COM_CRC_ERROR       = (0x08),
  SD_R1_ERASE_SEQUENCE_ERROR= (0x10),
  SD_R1_ADDRESS_ERROR       = (0x20),
  SD_R1_PARAMETER_ERROR     = (0x40),

/* R2 answer value */
  SD_R2_NO_ERROR            = 0x00,
  SD_R2_CARD_LOCKED         = 0x01,
  SD_R2_LOCKUNLOCK_ERROR    = 0x02,
  SD_R2_ERROR               = 0x04,
  SD_R2_CC_ERROR            = 0x08,
  SD_R2_CARD_ECC_FAILED     = 0x10,
  SD_R2_WP_VIOLATION        = 0x20,
  SD_R2_ERASE_PARAM         = 0x40,
  SD_R2_OUTOFRANGE          = 0x80,

/**
  * @brief  Data response error
  */
  SD_DATA_OK                = (0x05),
  SD_DATA_CRC_ERROR         = (0x0B),
  SD_DATA_WRITE_ERROR       = (0x0D),
  SD_DATA_OTHER_ERROR       = (0xFF)
} SD_Error;

__IO uint8_t SdStatus = SD_NOT_PRESENT;

/* flag_SDHC :
      0 : Standard capacity
      1 : High capacity
*/
uint16_t flag_SDHC = 0;

// static uint32_t CardType = SD_STD_CAPACITY_SD_CARD_V1_1;
static uint32_t CardType = SD_HIGH_CAPACITY_SD_CARD;
/* Prototypes of private function --------------------------------------------*/

/* Definition of functions ---------------------------------------------------*/

/**
 * @brief  DeInitializes the SD/SD communication.
 * @param  None
 * @retval None
 */
void SD_DeInit(void) { SD_LowLevel_DeInit(); }

/**
 * @brief  Initializes the SD/SD communication.
 * @param  None
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_Init(void) {
  uint32_t i = 0;

  /*!< Initialize SD_SPI */
  SD_LowLevel_Init();

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte 0xFF, 10 times with CS high */
  /*!< Rise CS and MOSI for 80 clocks cycles */
  for (i = 0; i <= 9; i++) {
    /*!< Send dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);
  }

  /*------------Put SD in SPI mode--------------*/
  /*!< SD initialized and set to SPI mode properly */
  return (SD_GoIdleState());
}

/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
uint8_t SD_Detect(void) {
  __IO uint8_t status = SD_PRESENT;

  /*!< Check GPIO to detect SD */
  if (GPIO_ReadInputData(SD_DETECT_GPIO_PORT) & SD_DETECT_PIN) {
    status = SD_NOT_PRESENT;
  }
  return status;
}

/**
 * @brief  Returns information about specific card.
 * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD
 *         card information.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCardInfo(SD_CardInfo* cardinfo) {
  SD_Error status = SD_RESPONSE_FAILURE;

  status                 = SD_GetCSDRegister(&(cardinfo->SD_csd));
  status                 = SD_GetCIDRegister(&(cardinfo->SD_cid));
  cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);
  cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
  cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
  cardinfo->CardCapacity *= cardinfo->CardBlockSize;

  /*!< Returns the reponse */
  return status;
}

/**
 * @brief  Reads block(s) from a specified address in the SD card, in polling mode.
 * @param  pBuffer: pointer to the buffer that receives the data read from the
 *                  SD.
 * @param  ReadAddr: SD's internal address to read from.
 * @param  BlockSize: the SD card Data block size.
 * @param  NumberOfBlocks: number of blocks to be read.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_ReadBlocks(uint8_t* pBuffer, uint64_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks) {
  uint32_t i = 0, Offset = 0;
  SD_Error rvalue      = SD_RESPONSE_FAILURE;
  uint32_t offset_unit = SD_BLOCK_SIZE;

  /*!< SD chip select low */
  SD_CS_LOW();

  /* Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
     Check if the SD acknowledged the set block length command: R1 response (0x00: no errors) */
  SD_SendCmd(SD_CMD_SET_BLOCKLEN, SD_BLOCK_SIZE, 0xFF);
  SD_WriteByte(SD_DUMMY_BYTE);
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
    return SD_RESPONSE_FAILURE;
  }

  if (CardType == SD_HIGH_CAPACITY_SD_CARD) {
    ReadAddr >>= 9; /* Convert it to sector address, divided by 512 */
    offset_unit = 1;
  }

  /*!< Data transfer */
  while (NumberOfBlocks--) {
    /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block and
      Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
    SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, ReadAddr, 0xFF);
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
      return SD_RESPONSE_FAILURE;
    }

    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ)) {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < SD_BLOCK_SIZE; i++) {
        /*!< Read the pointed data */
        *pBuffer = SD_ReadByte();
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }

      /*!< Set next read address*/
      Offset += offset_unit;

      /*!< get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
      SD_ReadByte();

      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    } else {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Returns the reponse */
  return rvalue;
}

/**
 * @brief  Writes block(s) from a specified address in the SD card, in polling mode.
 * @param  pBuffer: pointer to the buffer containing the data to be written on
 *                  the SD.
 * @param  WriteAddr: address to write on.
 * @param  BlockSize: the SD card Data block size.
 * @param  NumberOfBlocks: number of blocks to be written.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_WriteBlocks(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks) {
  uint32_t i = 0, Offset = 0;
  SD_Error rvalue      = SD_RESPONSE_FAILURE;
  uint32_t offset_unit = SD_BLOCK_SIZE;

  /*!< SD chip select low */
  SD_CS_LOW();

  /* Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
     Check if the SD acknowledged the set block length command: R1 response (0x00: no errors) */
  SD_SendCmd(SD_CMD_SET_BLOCKLEN, SD_BLOCK_SIZE, 0xFF);
  SD_WriteByte(SD_DUMMY_BYTE);
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
    return SD_RESPONSE_FAILURE;
  }

  if (CardType == SD_HIGH_CAPACITY_SD_CARD) {
    WriteAddr >>= 9; /* Convert it to sector address, divided by 512 */
    offset_unit = 1;
  }

  /*!< Data transfer */
  while (NumberOfBlocks--) {
    /*!< Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks and
         Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
    SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr, 0xFF);
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
      return SD_RESPONSE_FAILURE;
    }

    /* Send dummy byte for NWR timing : one byte between CMDWRITE and TOKEN */
    SD_WriteByte(SD_DUMMY_BYTE);

    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(SD_START_DATA_SINGLE_BLOCK_WRITE);

    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++) {
      /*!< Send the pointed byte */
      SD_WriteByte(*pBuffer);
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
    }

    /*!< Set next write address */
    Offset += offset_unit;

    /*!< Put CRC bytes (not really needed by us, but required by SD) */
    SD_ReadByte();
    SD_ReadByte();

    /*!< Read data response */
    if (SD_GetDataResponse() == SD_DATA_OK) {
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    } else {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Returns the reponse */
  return rvalue;
}

/**
 * @brief  Read the CSD card register.
 *         Reading the contents of the CSD register in SPI mode is a simple
 *         read-block transaction.
 * @param  SD_csd: pointer on an SCD register structure
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCSDRegister(SD_CSD* SD_csd) {
  uint32_t i      = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t  CSD_Tab[16];

  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Send CMD9 (CSD register) or CMD10(CSD register) */
  SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF);
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ)) {
      for (i = 0; i < 16; i++) {
        /*!< Store CSD register value on CSD_Tab */
        CSD_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Byte 0 */
  SD_csd->CSDStruct      = (CSD_Tab[0] & 0xC0) >> 6;
  SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  SD_csd->Reserved1      = CSD_Tab[0] & 0x03;

  /*!< Byte 1 */
  SD_csd->TAAC = CSD_Tab[1];

  /*!< Byte 2 */
  SD_csd->NSAC = CSD_Tab[2];

  /*!< Byte 3 */
  SD_csd->MaxBusClkFrec = CSD_Tab[3];

  /*!< Byte 4 */
  SD_csd->CardComdClasses = CSD_Tab[4] << 4;

  /*!< Byte 5 */
  SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;

  /*!< Byte 6 */
  SD_csd->PartBlockRead   = (CSD_Tab[6] & 0x80) >> 7;
  SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  SD_csd->DSRImpl         = (CSD_Tab[6] & 0x10) >> 4;
  SD_csd->Reserved2       = 0; /*!< Reserved */

  SD_csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;

  /*!< Byte 7 */
  SD_csd->DeviceSize |= (CSD_Tab[7]) << 2;

  /*!< Byte 8 */
  SD_csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;

  SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);

  /*!< Byte 9 */
  SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  SD_csd->DeviceSizeMul      = (CSD_Tab[9] & 0x03) << 1;
  /*!< Byte 10 */
  SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;

  SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
  SD_csd->EraseGrMul  = (CSD_Tab[10] & 0x3F) << 1;

  /*!< Byte 11 */
  SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
  SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);

  /*!< Byte 12 */
  SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  SD_csd->ManDeflECC        = (CSD_Tab[12] & 0x60) >> 5;
  SD_csd->WrSpeedFact       = (CSD_Tab[12] & 0x1C) >> 2;
  SD_csd->MaxWrBlockLen     = (CSD_Tab[12] & 0x03) << 2;

  /*!< Byte 13 */
  SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
  SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  SD_csd->Reserved3           = 0;
  SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);

  /*!< Byte 14 */
  SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  SD_csd->CopyFlag         = (CSD_Tab[14] & 0x40) >> 6;
  SD_csd->PermWrProtect    = (CSD_Tab[14] & 0x20) >> 5;
  SD_csd->TempWrProtect    = (CSD_Tab[14] & 0x10) >> 4;
  SD_csd->FileFormat       = (CSD_Tab[14] & 0x0C) >> 2;
  SD_csd->ECC              = (CSD_Tab[14] & 0x03);

  /*!< Byte 15 */
  SD_csd->CSD_CRC   = (CSD_Tab[15] & 0xFE) >> 1;
  SD_csd->Reserved4 = 1;

  /*!< Return the reponse */
  return rvalue;
}

/**
 * @brief  Read the CID card register.
 *         Reading the contents of the CID register in SPI mode is a simple
 *         read-block transaction.
 * @param  SD_cid: pointer on an CID register structure
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCIDRegister(SD_CID* SD_cid) {
  uint32_t i      = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t  CID_Tab[16];

  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD10 (CID register) */
  SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF);

  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR)) {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ)) {
      /*!< Store CID register value on CID_Tab */
      for (i = 0; i < 16; i++) {
        CID_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Byte 0 */
  SD_cid->ManufacturerID = CID_Tab[0];

  /*!< Byte 1 */
  SD_cid->OEM_AppliID = CID_Tab[1] << 8;

  /*!< Byte 2 */
  SD_cid->OEM_AppliID |= CID_Tab[2];

  /*!< Byte 3 */
  SD_cid->ProdName1 = CID_Tab[3] << 24;

  /*!< Byte 4 */
  SD_cid->ProdName1 |= CID_Tab[4] << 16;

  /*!< Byte 5 */
  SD_cid->ProdName1 |= CID_Tab[5] << 8;

  /*!< Byte 6 */
  SD_cid->ProdName1 |= CID_Tab[6];

  /*!< Byte 7 */
  SD_cid->ProdName2 = CID_Tab[7];

  /*!< Byte 8 */
  SD_cid->ProdRev = CID_Tab[8];

  /*!< Byte 9 */
  SD_cid->ProdSN = CID_Tab[9] << 24;

  /*!< Byte 10 */
  SD_cid->ProdSN |= CID_Tab[10] << 16;

  /*!< Byte 11 */
  SD_cid->ProdSN |= CID_Tab[11] << 8;

  /*!< Byte 12 */
  SD_cid->ProdSN |= CID_Tab[12];

  /*!< Byte 13 */
  SD_cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  SD_cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

  /*!< Byte 14 */
  SD_cid->ManufactDate |= CID_Tab[14];

  /*!< Byte 15 */
  SD_cid->CID_CRC   = (CID_Tab[15] & 0xFE) >> 1;
  SD_cid->Reserved2 = 1;

  /*!< Return the reponse */
  return rvalue;
}

/**
  * @brief  Send 5 bytes command to the SD card and get response
  * @param  Cmd The user expected command to send to SD card.
  * @param  Arg The command argument.
  * @param  Crc The CRC.
  * @param  Answer SD_ANSWER_NOT_EXPECTED or SD_ANSWER_EXPECTED
  * @retval SD status
  */
SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer)
{
  uint8_t frame[SD_CMD_LENGTH], frameout[SD_CMD_LENGTH];
  SD_CmdAnswer_typedef retr = {0xFF, 0xFF , 0xFF, 0xFF, 0xFF};

  /* R1 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 1 Bytes answer + NEC(0) = 15bytes */
  /* R1b identical to R1 + Busy information                                                   */
  /* R2 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 2 Bytes answer + NEC(0) = 16bytes */

  /* Prepare Frame to send */
  frame[0] = (Cmd | 0x40);         /* Construct byte 1 */
  frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
  frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
  frame[3] = (uint8_t)(Arg >> 8);  /* Construct byte 4 */
  frame[4] = (uint8_t)(Arg);       /* Construct byte 5 */
  frame[5] = (Crc | 0x01);         /* Construct byte 6 */

  /* Send the command */
  SD_IO_CSState(0);
  SD_IO_WriteReadData(frame, frameout, SD_CMD_LENGTH); /* Send the Cmd bytes */

  switch(Answer)
  {
  case SD_ANSWER_R1_EXPECTED :
    retr.r1 = SD_ReadData();
    break;
  case SD_ANSWER_R1B_EXPECTED :
    retr.r1 = SD_ReadData();
    retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    /* Set CS High */
    SD_IO_CSState(1);
    HAL_Delay(1);
    /* Set CS Low */
    SD_IO_CSState(0);

    /* Wait IO line return 0xFF */
    while (SD_IO_WriteByte(SD_DUMMY_BYTE) != 0xFF);
    break;
  case SD_ANSWER_R2_EXPECTED :
    retr.r1 = SD_ReadData();
    retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    break;
  case SD_ANSWER_R3_EXPECTED :
  case SD_ANSWER_R7_EXPECTED :
    retr.r1 = SD_ReadData();
    retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    retr.r3 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    retr.r4 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    retr.r5 = SD_IO_WriteByte(SD_DUMMY_BYTE);
    break;
  default :
    break;
  }
  return retr;
}

/**
  * @brief  Gets the SD card data response and check the busy flag.
  * @retval The SD status: Read data response xxx0<status>1
  *         - status 010: Data accecpted
  *         - status 101: Data rejected due to a crc error
  *         - status 110: Data rejected due to a Write error.
  *         - status 111: Data rejected due to other error.
  */
uint8_t SD_GetDataResponse(void)
{
  uint8_t dataresponse;
  uint8_t rvalue = SD_DATA_OTHER_ERROR;

  dataresponse = SD_IO_WriteByte(SD_DUMMY_BYTE);
  SD_IO_WriteByte(SD_DUMMY_BYTE); /* read the busy response byte*/

  /* Mask unused bits */
  switch (dataresponse & 0x1F)
  {
  case SD_DATA_OK:
    rvalue = SD_DATA_OK;

    /* Set CS High */
    SD_IO_CSState(1);
    /* Set CS Low */
    SD_IO_CSState(0);

    /* Wait IO line return 0xFF */
    while (SD_IO_WriteByte(SD_DUMMY_BYTE) != 0xFF);
    break;
  case SD_DATA_CRC_ERROR:
    rvalue =  SD_DATA_CRC_ERROR;
    break;
  case SD_DATA_WRITE_ERROR:
    rvalue = SD_DATA_WRITE_ERROR;
    break;
  default:
    break;
  }

  /* Return response */
  return rvalue;
}

/**
 * @brief  Returns the SD response.
 * @param  None
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetResponse(uint8_t Response) {
  uint32_t Count = 0xFFF;

  /*!< Check if response is got or a timeout is happen */
  while ((SD_ReadByte() != Response) && Count) {
    Count--;
  }
  if (Count == 0) {
    /*!< After time out */
    return SD_RESPONSE_FAILURE;
  } else {
    /*!< Right response got */
    return SD_RESPONSE_NO_ERROR;
  }
}

/**
  * @brief  Returns the SD status.
  * @retval The SD status.
  */
uint8_t SD_GetStatus(void)
{
  SD_CmdAnswer_typedef retr;

  /* Send CMD13 (SD_SEND_STATUS) to get SD status */
  retr = SD_SendCmd(SD_CMD_SEND_STATUS, 0, 0xFF, SD_ANSWER_R2_EXPECTED);
  SD_IO_CSState(1);
  SD_IO_WriteByte(SD_DUMMY_BYTE);

  /* Find SD status according to card state */
  if(( retr.r1 == SD_R1_NO_ERROR) && ( retr.r2 == SD_R2_NO_ERROR))
  {
    return BSP_SD_OK;
  }

  return BSP_SD_ERROR;
}
/**
  * @brief  Put the SD in Idle state.
  * @retval SD status
  */
uint8_t SD_GoIdleState(void)
{
  SD_CmdAnswer_typedef response;
  __IO uint8_t counter = 0;
  /* Send CMD0 (SD_CMD_GO_IDLE_STATE) to put SD in SPI mode and
     wait for In Idle State Response (R1 Format) equal to 0x01 */
  do{
    counter++;
    response = SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95, SD_ANSWER_R1_EXPECTED);
    SD_IO_CSState(1);
    SD_IO_WriteByte(SD_DUMMY_BYTE);
    if(counter >= SD_MAX_TRY)
    {
      return BSP_SD_ERROR;
    }
  }
  while(response.r1 != SD_R1_IN_IDLE_STATE);


  /* Send CMD8 (SD_CMD_SEND_IF_COND) to check the power supply status
     and wait until response (R7 Format) equal to 0xAA and */
  response = SD_SendCmd(SD_CMD_SEND_IF_COND, 0x1AA, 0x87, SD_ANSWER_R7_EXPECTED);
  SD_IO_CSState(1);
  SD_IO_WriteByte(SD_DUMMY_BYTE);
  if((response.r1  & SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND)
  {
    /* initialise card V1 */
    do
    {
      /* initialise card V1 */
      /* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
      response = SD_SendCmd(SD_CMD_APP_CMD, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
      SD_IO_CSState(1);
      SD_IO_WriteByte(SD_DUMMY_BYTE);

      /* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
      response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
      SD_IO_CSState(1);
      SD_IO_WriteByte(SD_DUMMY_BYTE);
    }
    while(response.r1 == SD_R1_IN_IDLE_STATE);
    flag_SDHC = 0;
  }
  else if(response.r1 == SD_R1_IN_IDLE_STATE)
  {
      /* initialise card V2 */
    do {

      /* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
      response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
      SD_IO_CSState(1);
      SD_IO_WriteByte(SD_DUMMY_BYTE);

      /* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
      response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x40000000, 0xFF, SD_ANSWER_R1_EXPECTED);
      SD_IO_CSState(1);
      SD_IO_WriteByte(SD_DUMMY_BYTE);
    }
    while(response.r1 == SD_R1_IN_IDLE_STATE);

    if((response.r1 & SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND)
    {
      do {
        /* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
        response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
        SD_IO_CSState(1);
        SD_IO_WriteByte(SD_DUMMY_BYTE);
        if(response.r1 != SD_R1_IN_IDLE_STATE)
        {
          return BSP_SD_ERROR;
        }
        /* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
        response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
        SD_IO_CSState(1);
        SD_IO_WriteByte(SD_DUMMY_BYTE);
      }
      while(response.r1 == SD_R1_IN_IDLE_STATE);
    }

    /* Send CMD58 (SD_CMD_READ_OCR) to initialize SDHC or SDXC cards: R3 response (0x00: no errors) */
    response = SD_SendCmd(SD_CMD_READ_OCR, 0x00000000, 0xFF, SD_ANSWER_R3_EXPECTED);
    SD_IO_CSState(1);
    SD_IO_WriteByte(SD_DUMMY_BYTE);
    if(response.r1 != SD_R1_NO_ERROR)
    {
      return BSP_SD_ERROR;
    }
    flag_SDHC = (response.r2 & 0x40) >> 6;
  }
  else
  {
    return BSP_SD_ERROR;
  }

  return BSP_SD_OK;
}

/**
 * @brief  Write a byte on the SD.
 * @param  Data: byte to send.
 * @retval None
 */
uint8_t SD_WriteByte(uint8_t Data) {
  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_TXE) == RESET) {
  }

  /*!< Send the byte */
  SPI_I2S_SendData(SD_SPI, Data);

  /*!< Wait to receive a byte*/
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
  }

  /*!< Return the byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(SD_SPI);
}

/**
 * @brief  Read a byte from the SD.
 * @param  None
 * @retval The received byte.
 */
uint8_t SD_ReadByte(void) {
  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_TXE) == RESET) {
  }
  /*!< Send the byte */
  SPI_I2S_SendData(SD_SPI, SD_DUMMY_BYTE);

  /*!< Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
  }

  /*!< Return the received data */
  return (uint8_t)SPI_I2S_ReceiveData(SD_SPI);
}

/**
  * @brief  Waits a data until a value different from SD_DUMMY_BITE
  * @retval the value read
  */
uint8_t SD_ReadData(void)
{
  uint8_t timeout = 0x08;
  uint8_t readvalue;

  /* Check if response is got or a timeout is happen */
  do {
    readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
    timeout--;

  }while ((readvalue == SD_DUMMY_BYTE) && timeout);

  /* Right response got */
  return readvalue;
}

/**
  * @brief  Waits a data from the SD card
  * @param  data  Expected data from the SD card
  * @retval BSP_SD_OK or BSP_SD_TIMEOUT
  */
uint8_t SD_WaitData(uint8_t data)
{
  uint16_t timeout = 0xFFFF;
  uint8_t readvalue;

  /* Check if response is got or a timeout is happen */

  do {
    readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
    timeout--;
  }while ((readvalue != data) && timeout);

  if (timeout == 0)
  {
    /* After time out */
    return BSP_SD_TIMEOUT;
  }

  /* Right response got */
  return BSP_SD_OK;
}

void SD_IO_CSState(uint8_t val)
{
  if(val == 1)
  {
    SD_CS_HIGH();
  }
  else
  {
    SD_CS_LOW();
  }
}

/**
  * @brief  Write a byte on the SD.
  * @param  DataIn byte to send.
  * @param  DataOut read byte.
  * @param  DataLength data length.
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
  {
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Writes a byte on the SD.
  * @param  Data byte to send.
  * @retval None
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;

  /* Send the byte */
  SPIx_WriteReadData(&Data,&tmp,1);
  return tmp;
}
