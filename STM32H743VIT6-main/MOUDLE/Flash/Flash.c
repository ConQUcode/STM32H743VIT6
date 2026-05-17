#include "Flash.h"
#include "spi.h"

uint16_t W25QXX_TYPE = 0xEF40;

uint8_t W25Q128_SPI_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
    return Rxdata;
}

void W25Q128_Init(void)
{
    W25Q_CS_DISABLE();
}

uint16_t W25Q128_ReadID(void)
{
    uint16_t Temp = 0;
    W25Q_CS_ENABLE();
    W25Q128_SPI_ReadWriteByte(W25X_ManufactDeviceID);
    W25Q128_SPI_ReadWriteByte(0x00);
    W25Q128_SPI_ReadWriteByte(0x00);
    W25Q128_SPI_ReadWriteByte(0x00);
    Temp |= W25Q128_SPI_ReadWriteByte(0xFF)<<8;
    Temp |= W25Q128_SPI_ReadWriteByte(0xFF);
    W25Q_CS_DISABLE();
    return Temp;
}

void W25Q128_Write_Enable(void)
{
    W25Q_CS_ENABLE();
    W25Q128_SPI_ReadWriteByte(W25X_WriteEnable);
    W25Q_CS_DISABLE();
}

void W25Q128_Wait_Busy(void)
{
    uint8_t status = 0;
    while(1)
    {
        W25Q_CS_ENABLE();
        W25Q128_SPI_ReadWriteByte(W25X_ReadStatusReg1);
        status = W25Q128_SPI_ReadWriteByte(0XFF);
        W25Q_CS_DISABLE();
        if ((status & 0x01) == 0)
            break;
    }
}

void W25Q128_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint16_t i;
    W25Q_CS_ENABLE();
    W25Q128_SPI_ReadWriteByte(W25X_ReadData);
    W25Q128_SPI_ReadWriteByte((uint8_t)((ReadAddr)>>16));
    W25Q128_SPI_ReadWriteByte((uint8_t)((ReadAddr)>>8));
    W25Q128_SPI_ReadWriteByte((uint8_t)ReadAddr);
    for(i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = W25Q128_SPI_ReadWriteByte(0XFF);
    }
    W25Q_CS_DISABLE();
}

void W25Q128_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;
    W25Q128_Write_Enable();
    W25Q_CS_ENABLE();
    W25Q128_SPI_ReadWriteByte(W25X_PageProgram);
    W25Q128_SPI_ReadWriteByte((uint8_t)((WriteAddr)>>16));
    W25Q128_SPI_ReadWriteByte((uint8_t)((WriteAddr)>>8));
    W25Q128_SPI_ReadWriteByte((uint8_t)WriteAddr);
    for(i = 0; i < NumByteToWrite; i++)
    {
        W25Q128_SPI_ReadWriteByte(pBuffer[i]);
    }
    W25Q_CS_DISABLE();
    W25Q128_Wait_Busy();
}

void W25Q128_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256;
    if(NumByteToWrite <= pageremain) pageremain = NumByteToWrite;
    while(1)
    {
        W25Q128_Write_Page(pBuffer, WriteAddr, pageremain);
        if(NumByteToWrite == pageremain) break;
        else 
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;

            NumByteToWrite -= pageremain;
            if(NumByteToWrite > 256) pageremain = 256;
            else pageremain = NumByteToWrite;
        }
    }
}

void W25Q128_Erase_Sector(uint32_t Dst_Addr)
{
    Dst_Addr *= 4096;
    W25Q128_Write_Enable();
    W25Q128_Wait_Busy();
    W25Q_CS_ENABLE();
    W25Q128_SPI_ReadWriteByte(W25X_SectorErase);
    W25Q128_SPI_ReadWriteByte((uint8_t)((Dst_Addr)>>16));
    W25Q128_SPI_ReadWriteByte((uint8_t)((Dst_Addr)>>8));
    W25Q128_SPI_ReadWriteByte((uint8_t)Dst_Addr);
    W25Q_CS_DISABLE();
    W25Q128_Wait_Busy();
}

uint8_t w25q128_buffer[4096];
void W25Q128_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t* w25q128_buf;
    w25q128_buf = w25q128_buffer;
    secpos = WriteAddr / 4096;
    secoff = WriteAddr % 4096;
    secremain = 4096 - secoff;
    if(NumByteToWrite <= secremain) secremain = NumByteToWrite;
    while(1)
    {
        W25Q128_Read(w25q128_buf, secpos * 4096, 4096);
        for(i = 0; i < secremain; i++)
        {
            if(w25q128_buf[secoff + i] != 0XFF) break;
        }
        if(i < secremain)
        {
            W25Q128_Erase_Sector(secpos);
            for(i = 0; i < secremain; i++)
            {
                w25q128_buf[i + secoff] = pBuffer[i];
            }
            W25Q128_Write_NoCheck(w25q128_buf, secpos * 4096, 4096);
        }
        else
        {
            W25Q128_Write_NoCheck(pBuffer, WriteAddr, secremain);
        }
        if(NumByteToWrite == secremain) break;
        else 
        {
            secpos++;
            secoff = 0;

            pBuffer += secremain;
            WriteAddr += secremain;
            NumByteToWrite -= secremain;
            if(NumByteToWrite > 4096) secremain = 4096;
            else secremain = NumByteToWrite;
        }
    }
}

// ==== 下面是为您准备的测试代码 ====

// 定义全局变量，防止在 Debug 时被编译器优化掉，方便你在 Watch 窗口查看
volatile uint16_t g_flash_id = 0;
uint8_t g_flash_test_write[32] = "RoboMaster W25Q128 Test!!";
uint8_t g_flash_test_read[32] = {0};

/**
 * @brief  测试 W25Q128 的基础功能 (读ID、擦除、写入、读取)
 * @note   你可以在 main.c 的 while(1) 之前调用一次该函数，并在最后一行代码处打上断点
 */
void W25Q128_Test(void)
{
    // 1. 读取 Flash ID
    // 正常情况下，W25Q128 的 Manufacturer/Device ID 是 0xEF17 (如果是 JEDEC ID 可能是 0xEF4018)
    // 根据宏定义 W25X_ManufactDeviceID 0x90 读取出来的应为 0xEF17 左右
    g_flash_id = W25Q128_ReadID();
    
    // 如果 g_flash_id 不是 0xFF 或 0x00，且读到了正确的值 (比如 0xEF17)
    if(g_flash_id == 0xEF17)
    {
        uint32_t test_addr = 0x000000;
        
        // 2. 擦除扇区 (Sector 0)
        W25Q128_Erase_Sector(test_addr); 
        
        // 3. 写入测试字符串
        W25Q128_Write(g_flash_test_write, test_addr, sizeof(g_flash_test_write));
        
        // 4. 将刚才写进去的数据读出来
        W25Q128_Read(g_flash_test_read, test_addr, sizeof(g_flash_test_read));
    }
    
    // --> 请在这行后面打一个断点 (Breakpoint)。
    // 运行至此处停止后，将 `g_flash_id`、`g_flash_test_read` 添加到 Watch 窗口查看。
    // 如果 `g_flash_test_read` 里的字符串和 `g_flash_test_write` 完全一致，说明板载 Flash 的 SPI 通信和驱动工作完美！
    while(0){} 
}



