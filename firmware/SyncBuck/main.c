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
#include "pid.h"

void UART_Open(UART_T *uart, uint32_t u32baudrate);

STR_PID_T PFM_PID = {0};
STR_PID_T PWM_PID = {0};

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

	/* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select Timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable Hardware Divider Clock */
    CLK_EnableModuleClock(HDIV_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |        \
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);
	
	/* Set PC multi-function pins for PWM0 Channel 2/3 */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC0MFP_Msk | SYS_GPC_MFP0_PC1MFP_Msk)) |
                    (SYS_GPC_MFP0_PC0MFP_PWM0_CH2 | SYS_GPC_MFP0_PC1MFP_PWM0_CH3);

    /* Set PC.0, PC.1 to push-pull mode */
    GPIO_SetMode(PC, BIT0 | BIT1, GPIO_MODE_OUTPUT);

    /* Set PC.3, PC.4 to input mode */
    GPIO_SetMode(PC, BIT3 | BIT4, GPIO_MODE_INPUT);

    /* Configure the PC.3, PC.4 ADC analog input pins.  */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC3MFP_Msk)) |
                    (SYS_GPC_MFP0_PC3MFP_ADC0_CH12);
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(SYS_GPC_MFP1_PC4MFP_Msk)) |
                    (SYS_GPC_MFP1_PC4MFP_ADC0_CH13);

    /* Disable the PC.3, PC.4 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PC, BIT3 | BIT4);

    /* Lock protected registers */
    SYS_LockReg();
}

void PWM0_Init()
{
    /* Set Pwm mode as complementary mode */
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /* PWM0 channel 2 frequency is 480000 Hz, duty 0%, */
    //PWM_ConfigOutputChannel(PWM0, 2, 480000, 0);

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 0);

    /* Set up counter type */
    PWM0->CTL1 = (PWM0->CTL1 & ~(PWM_CTL1_CNTTYPE2_Msk)) | (PWM_DOWN_COUNTER << PWM_CTL1_CNTTYPE2_Pos);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 2, 0);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 2, 2400); // PFM
    //PWM_SET_CNR(PWM0, 2, 200); // PWM

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set PWM0 dead-time */
    PWM_EnableDeadZone(PWM0, 2, 6);

    /* Lock protected registers */
    SYS_LockReg();

    /* PWM period point trigger ADC enable */
    //PWM_EnableADCTrigger(PWM0, 2, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT2, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW, PWM_OUTPUT_HIGH);

    /* Enable output of PWM0 channel 2 and 3 */
    PWM_EnableOutput(PWM0, 0xc);
}

void ADC_Init()
{
    /* Enable ADC converter */
    ADC_POWER_ON(ADC);
    
    /* Set input mode as single-end, Single mode, and select channel 12 and 13 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT12 | BIT13);

    /* Configure the sample module and enable PWM0 trigger source */
    //ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_PWM, 0);

    /* Configure the sample module and enable TIMER trigger source */
    ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_TIMER, 0);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the sample module interrupt */
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
    NVIC_EnableIRQ(ADC_IRQn);
}

void TIMER0_Init()
{
    /* Set timer0 periodic time-out frequency is 200KHz */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 200000);

    /* Enable timer interrupt trigger ADC */
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSRC_TIMEOUT_EVENT);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_ADC);
}

void PID_Init()
{
    PID_SetPoint(&PFM_PID, 1365);

    PID_SetGain(&PFM_PID, -308, 0, 0);

    PID_SetPoint(&PWM_PID, 1365);

    PID_SetGain(&PWM_PID, 312, 0, 0);
}

void ADC_IRQHandler(void)
{
    int32_t i32ConversionData, i32Comp, i32PFM, i32PWM;

    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
#if 1
    if(ADC_IS_DATA_VALID(ADC, 12))
        i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 12);


    i32Comp = HDIV_Div(PID_GetCompValue(&PFM_PID, i32ConversionData), 10000);

    //printf("Comp: %d\r\n", i32Comp);

    i32PFM = PWM_GET_CNR(PWM0, 2); // Get current period value
    if ((i32PFM + i32Comp) > 2400)
        i32PFM = 2400; // Min freq=20KHz
    else if ((i32PFM + i32Comp) < 100)
        i32PFM = 100; // Max freq=480KHz
    else
        i32PFM += i32Comp;

    //printf("PFM: %d\r\n", i32PFM);

    PWM_SET_CNR(PWM0, 2, i32PFM);
/*
    i32Comp = HDIV_Div(PID_GetCompValue(&PWM_PID, i32ConversionData), 10000);

    //printf("Comp: %d\r\n", i32Comp);

    i32PWM = PWM_GET_CMR(PWM0, 2);
    if ((i32PWM + i32Comp) > 200)
        i32PWM = 200;
    else if ((i32PWM + i32Comp) < 0)
        i32PWM = 0;
    else
        i32PWM += i32Comp;

    //printf("PFM: %d\r\n", i32PFM);

    PWM_SET_CMR(PWM0, 2, i32PWM);
*/
#else
    int32_t u32ChannelCount, i32ConversionData;

    printf("ADC Result: \r\n");

    for(u32ChannelCount = 12; u32ChannelCount <= 13; u32ChannelCount++)
    {
        if(ADC_IS_DATA_VALID(ADC, u32ChannelCount))
        {
            i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
            printf("ADC0_CH%d: %d\r\n", u32ChannelCount, i32ConversionData);
        }
    }
    printf("\r\n");
#endif
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
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");

    /* Initial PWM0 */
    PWM0_Init();

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 2, 26);

    /* Initial PID controller */
    PID_Init();

    /* Init TIMER0 for ADC */
    TIMER0_Init();

    /* Initial ADC */
    ADC_Init();

    /* Enable Timer0 counter */
    TIMER_Start(TIMER0);

    /* Enable PWM0 channel 2 counter */
    PWM_Start(PWM0, 0x4); // CNTEN2

    /* Got no where to go, just loop forever */
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
