/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    /* Lock protected registers */
    SYS_LockReg();
}
#define DRVFMC_PAGE_SIZE       512
#define DRVFMC_SECTOR_SIZE     2048
uint32_t g_sectorBuf[DRVFMC_SECTOR_SIZE]; /* 2096 bytes */
void  FMC_ReadPage(uint32_t u32StartAddr, uint32_t *u32Pattern)
{
    uint32_t    u32Addr = 0;
    uint32_t u32Data = 0;
    uint32_t u32LoopCounter = 0;

    for (u32Addr = u32StartAddr; u32Addr < u32StartAddr + DRVFMC_PAGE_SIZE; u32Addr += 4)
    {
        u32Data = FMC_Read(u32Addr);
        u32Pattern[u32LoopCounter] = u32Data;
        u32LoopCounter++;
    }

}

void FMC_WARP_Read(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level read function of USB Mass Storage */
    int32_t len;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();
    len = (int32_t)size;

    do
    {
        FMC_ReadPage(addr, (uint32_t *)buffer);
        addr   += DRVFMC_PAGE_SIZE;
        buffer += DRVFMC_PAGE_SIZE;
        len    -= DRVFMC_PAGE_SIZE;
    } while (len >= DRVFMC_PAGE_SIZE);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
}


void FMC_ProgramPage(uint32_t u32StartPage, uint8_t *pu8Data)
{
    uint32_t u32StartAddr;
    uint32_t u32LoopCounter;
    uint32_t u32Data;
    uint8_t  u8Status;

    u32StartAddr = u32StartPage & 0x0FFFFF00;

    for (u32LoopCounter = 0; u32LoopCounter < DRVFMC_PAGE_SIZE; u32LoopCounter += 4)
    {
        u32Data = ((((pu8Data[u32LoopCounter] << 0) | (pu8Data[u32LoopCounter + 1] << 8)) |
                    (pu8Data[u32LoopCounter + 2] << 16)) + (pu8Data[u32LoopCounter + 3] << 24));
        FMC_Write(u32StartAddr + u32LoopCounter, u32Data);
    }
}

void FMC_WARP_Write(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;
    uint32_t BLACK_TARGET = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();


    FMC_ENABLE_AP_UPDATE();

    FMC_ENABLE_LD_UPDATE();


    len = (int32_t)size;

    if (len == DRVFMC_SECTOR_SIZE && ((addr & (DRVFMC_SECTOR_SIZE - 1)) == 0))
    {
        /* one-sector length & the start address is sector-alignment */
        FMC_Erase(addr);


        while (len >= DRVFMC_PAGE_SIZE)
        {
            FMC_ReadPage(addr, (uint32_t *)buffer);
            addr   += DRVFMC_PAGE_SIZE;
            buffer += DRVFMC_PAGE_SIZE;
            len    -= DRVFMC_PAGE_SIZE;
        }
    }
    else
    {
        do
        {
            /* alignAddr: sector address */
            alignAddr = addr & 0x1FFFF800;

            /* Get the sector offset*/
            offset = (addr & (DRVFMC_SECTOR_SIZE - 1));

            if (offset || (size < DRVFMC_SECTOR_SIZE))
            {
                /* if the start address is not sector-alignment or the size is less than one sector, */
                /* read back the data of the destination sector to g_sectorBuf[].                    */
                FMC_WARP_Read(alignAddr, DRVFMC_SECTOR_SIZE, (uint32_t)&g_sectorBuf[0]);
                SYS_UnlockReg();
                FMC_Open();
            }

            /* Update the data */
            pu32 = (uint32_t *)buffer;
            len = DRVFMC_SECTOR_SIZE - offset; /* len: the byte count between the start address and the end of a sector. */

            if (size < len) /* check if the range of data arrive at the end of a sector. */
                len = size; /* Not arrive at the end of a sector. "len" indicate the actual byte count of data. */

            for (i = 0; i < len / 4; i++)
            {
                g_sectorBuf[offset / 4 + i] = pu32[i];
            }


            FMC_Erase(alignAddr);

            for (i = 0; i < 4; i++) /* one sector (4 pages) */
            {
                FMC_ProgramPage(alignAddr + (i << 9), (uint8_t *)g_sectorBuf + (i << 9));
            }

            size -= len;
            addr += len;
            buffer += len;

        } while (size > 0);
    }


    FMC_DISABLE_AP_UPDATE();
    FMC_DISABLE_LD_UPDATE();


    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
}

unsigned char w_buffer[4096];
unsigned char r_buffer[4096];

int main()
{
    uint32_t    i, u32Data;

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Checking if target device supports the feature */
    if ((GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G))
    {
        /* Checking if flash size matches with target device */
        if (FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please enable the compiler option - PAGE_SIZE_2048 in fmc.h\n");

            while (SYS->PDID);
        }
    }
    else
    {
        if (FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please disable the compiler option - PAGE_SIZE_2048 in fmc.h\n");

            while (SYS->PDID);
        }
    }


    for (i = 0; i < 4096; i++)
    {
        w_buffer[i] = i;
        r_buffer[i] = 0;
    }

    FMC_WARP_Write(0x40000, 4096, (uint32_t)w_buffer);
    FMC_WARP_Read(0x40000, 4096, (uint32_t)r_buffer);

    //compare
    for (i = 0; i < 4096; i++)
    {
        if (w_buffer[i] != r_buffer[i])
        {
            while (1);
        }
    }

    for (i = 0; i < 64; i++)
    {
        w_buffer[i] = 64 - i;
        r_buffer[i] = 0;
    }

    FMC_WARP_Write(0x40900, 64, (uint32_t)w_buffer);
    FMC_WARP_Read(0x40900, 64, (uint32_t)r_buffer);

    for (i = 0; i < 64; i++)
    {

        if (w_buffer[i] != r_buffer[i])
        {
            while (1);
        }
    }

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
