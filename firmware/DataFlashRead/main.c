/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 20/05/28 5:40p $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define DATA_FLASH_BASE     0x7E00
#define DATA_FLASH_END      0x7FFF      /* 512B */
#define TEST_PATTERN        0x5A5A5A5A
/* CONFIG0: Enable data flash. */
/* This value need to be change into 0xFDFFFDFE in final product to disable reset pin and extern POR to 26.6ms */
#define CONFIG0             0xFFFFFFFE
/* CONFIG1: Set Data Flash Base Address to 0x7E00 (size 512 bytes) */
#define CONFIG1             DATA_FLASH_BASE

int IsDebugFifoEmpty(void);

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

    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |        \
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int check_config_bits(uint32_t u32Cfg0, uint32_t u32Cfg1)
{
    uint32_t au32Config[2];

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if (((au32Config[0] == u32Cfg0)) && (au32Config[1] == u32Cfg1))
        return 0;

    return -1;
}

int set_config_bits(uint32_t u32Cfg0, uint32_t u32Cfg1)
{
    uint32_t au32Config[2];

    FMC_ENABLE_CFG_UPDATE();

    /* Erase User Configuration */
    FMC_Erase(FMC_CONFIG_BASE);

    au32Config[0] = u32Cfg0;         /* CONFIG0[0] = 0 (Enabled) / 1 (Disabled) */
    au32Config[1] = u32Cfg1;

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;
    
    printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_BASE);

    /* To check if all the debug messages are finished */
    while(!IsDebugFifoEmpty());

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;

    return 0;
}


void run_crc32_checksum()
{
    uint32_t    chksum;

    chksum = FMC_GetChkSum(FMC_APROM_BASE, DATA_FLASH_BASE - FMC_APROM_BASE);

    printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC_Write(u32Addr, u32Pattern);
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);
        if (u32data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        FMC_Erase(u32Addr);

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page */
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }
    }
    printf("\r    Flash Test Passed.          \n");
    return 0;
}


int main()
{
    uint32_t    i, u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          M0A21 FMC Sample Code         |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable Data Flash and set base address. */
    if (check_config_bits(CONFIG0, CONFIG1) < 0)
    {
        if(set_config_bits(CONFIG0, CONFIG1))
        {
            printf("Failed to set Data Flash base address!\n");
            goto lexit;
        }
    }

    printf("Config bits check passed.\r\n");

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    u32Data = FMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        u32Data = FMC_ReadUID(i);
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        u32Data = FMC_ReadUCID(i);
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    /* Read Data Flash base address */
    u32Data = FMC_ReadDataFlashBaseAddr();
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nData Flash test =>\n");
    if (flash_test(DATA_FLASH_BASE, DATA_FLASH_END, TEST_PATTERN) < 0)
    {
        printf("\n\nData flash read/write test failed!\n");
        goto lexit;
    }

    run_crc32_checksum();

lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
