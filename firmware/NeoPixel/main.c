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
#include "NeoPixel.h"

void UART_Open(UART_T *uart, uint32_t u32baudrate);

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

    /* Set PA0, PA1 multi-function pins as GPIO */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));

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
    /* Create a NeoPixel objects */
    STR_NEOPIXEL_T pixels;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\r\n");

    /* Initialize NeoPixel */
    NeoPixel_begin(&pixels, 20, &PD6, NEO_GRB);

    /* Clear the buffer */
    NeoPixel_clear(&pixels);

    /* Set pixel color to purple */
    for(int i=0; i<pixels.u16numLEDs; i++)
    {
        NeoPixel_setPixelColor(&pixels, i, 30, 5, 40);
    }

    /* Send the updated pixel colors to the NeoPixel strip */
    NeoPixel_show(&pixels);

    printf("End.\r\n\n");

    /* Got no where to go, just loop forever */
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
