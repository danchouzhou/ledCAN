/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M0A21 MCU.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "delay.h"

void UART_Open(UART_T *uart, uint32_t u32baudrate);

volatile int64_t g_i64EncoderCnt = 0;

void GPAB_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PA.0 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT0))
    {
        GPIO_CLR_INT_FLAG(PA, BIT0);
        //printf("PA.0 INT occurred.\n");
        if (PA1 == 0) // CW
            g_i64EncoderCnt++;
        else // CCW
            g_i64EncoderCnt--;
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA interrupts */
        temp = PA->INTSRC;
        PA->INTSRC = temp;
        //printf("Un-expected interrupts.\n");
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
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

/*
 * This is a template project for M0A21 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    int64_t i64EncoderCntTmp;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");

    printf("Press any key to start.\n");
    getchar();

    /* Set PA multi-function pins for GPIO */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));

    /* Set GPIO mode as Quasi-bidirectional mode */
    GPIO_SetMode(PA, BIT0 | BIT1, GPIO_MODE_QUASI); // PA0, PA1

    /* Select de-bounce sampling cycle */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_64);

    /* Enable interrupt de-bounce function */
    GPIO_ENABLE_DEBOUNCE(PA, BIT0);

    /* Enable interrupt by rising edge trigger */
    GPIO_EnableInt(PA, 0, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

    /* Got no where to go, just loop forever */
    while(1)
    {
        i64EncoderCntTmp = g_i64EncoderCnt;
        g_i64EncoderCnt = 0;
        printf("Count: %d\r\n", (int)i64EncoderCntTmp);
        delay(2000);
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
