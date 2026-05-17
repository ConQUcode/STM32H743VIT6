#ifndef FLASH_H
#define FLASH_H

#include "main.h"

// CS GPIO Port and Pin
#define W25Q128_CS_PORT GPIOA
#define W25Q128_CS_PIN  GPIO_PIN_4

// W25Q128 commands
#define W25X_WriteEnable      0x06
#define W25X_WriteDisable     0x04
#define W25X_ReadStatusReg1   0x05
#define W25X_ReadStatusReg2   0x35
#define W25X_ReadStatusReg3   0x15
#define W25X_WriteStatusReg1  0x01
#define W25X_WriteStatusReg2  0x31
#define W25X_WriteStatusReg3  0x11
#define W25X_ReadData         0x03
#define W25X_FastReadData     0x0B
#define W25X_FastReadDual     0x3B
#define W25X_PageProgram      0x02
#define W25X_BlockErase       0xD8
#define W25X_SectorErase      0x20
#define W25X_ChipErase        0xC7
#define W25X_PowerDown        0xB9
#define W25X_ReleasePowerDown 0xAB
#define W25X_DeviceID         0xAB
#define W25X_ManufactDeviceID 0x90
#define W25X_JedecDeviceID    0x9F

#define W25Q_CS_ENABLE()  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET)
#define W25Q_CS_DISABLE() HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET)

void W25Q128_Init(void);
uint16_t W25Q128_ReadID(void);
void W25Q128_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void W25Q128_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25Q128_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25Q128_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25Q128_Erase_Sector(uint32_t Dst_Addr);
void W25Q128_Wait_Busy(void);

void W25Q128_Test(void);

#endif
