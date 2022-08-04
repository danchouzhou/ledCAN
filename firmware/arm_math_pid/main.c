#include <stdio.h>
#include "NuMicro.h"
#include "delay.h"
#include "arm_math.h"

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

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |        \
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

    /* Set PC multi-function pins for PWM0 Channel 2 */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_PWM0_CH2;

    /* Configure the PC.3 ADC analog input pins. */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_ADC0_CH12;

    /* Lock protected registers */
    SYS_LockReg();
}

void PWM_Init()
{
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 47); // 48MHz/(47+1)=1MHz

    /* Set counter to up counting */
    PWM0->CTL1 = (PWM0->CTL1 & ~(PWM_CTL1_CNTTYPE2_Msk)) | (PWM_UP_COUNTER << PWM_CTL1_CNTTYPE2_Pos);

    /* Set PWM0 duty */
    PWM_SET_CMR(PWM0, 2, 0);

    /* Set PWM0 period */
    PWM_SET_CNR(PWM0, 2, 99); // 1MHz/(99+1)=10KHz

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_2_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 2 */
    PWM_EnableOutput(PWM0, PWM_CH_2_MASK);

    /* Enable PWM0 channel 2 counter (PWM_CNT2) */
    PWM_Start(PWM0, PWM_CH_2_MASK);
}

void ADC_Init()
{
    /* Disable the PC.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PC, BIT3);

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);
    
    /* Set input mode as single-end, Single mode, and select channel 12 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT12);
}

void Note_Configure()
{
    printf("\n\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|  About the ARM DSP_Lib pid example code configure                      |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|   This example perform a simple way to test the DSP_Lib PID library.   |\n");
    printf("|   We will use PWM feed into a low-pass filter and read the output      |\n");
    printf("|   voltage by ADC.                                                      |\n");
    printf("|   1.Make a RC low-pass filter circuit on your bread board              |\n");
    printf("|   2.Feed PWM output into the filter use Vout as ADC input              |\n");
    printf("|   3.Use a scope or multimeter to measure the output voltage            |\n");
    printf("|                                                                        |\n");
    printf("|  |--------|       _____  R=1K                                          |\n");
    printf("|  |        |------[_____]-----------@ Vout                              |\n");
    printf("|  |        |PWM               |                                         |\n");
    printf("|  |  M0A21 |     _____________|                                         |\n");
    printf("|  |        |     |            |                                         |\n");
    printf("|  |        |------          -----                                       |\n");
    printf("|  |--------|ADC             ----- C=10uF                                |\n");
    printf("|       |                      |                                         |\n");
    printf("|       |                      |                                         |\n");
    printf("|       |                      |                                         |\n");
    printf("|     -----                  -----                                       |\n");
    printf("|      ---                    ---                                        |\n");
    printf("|       -                      -                                         |\n");
    printf("+------------------------------------------------------------------------+\n");
}

int main()
{
    const int16_t i16Target = 1024;
    arm_pid_instance_q15 PID;
    int16_t i16Current, i16Error, i16Duty;

    uint32_t u32Timestamp = 0;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    PWM_Init();

    ADC_Init();

    PID.Kp = 60;
    PID.Ki = 500;
    PID.Kd = 0;

    arm_pid_init_q15(&PID, 1);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");
    Note_Configure();
    printf("Press any key to start.\n");
    getchar();

    /* Got no where to go, just loop forever */
    while(1)
    {
        
        ADC_START_CONV(ADC);
        while(ADC_IS_DATA_VALID(ADC, 12) == 0);

        i16Current = ADC_GET_CONVERSION_DATA(ADC, 12);
        i16Error = i16Target - i16Current;
        i16Duty = arm_pid_q15(&PID, i16Error);

        printf("\r\nTimestamp: %d\r\n", u32Timestamp++);
        printf("i16Current: %d\r\n", i16Current);
        printf("i16Error: %d\r\n", i16Error);
        printf("i16Duty: %d\r\n", i16Duty);

        if(i16Duty < 0)
            i16Duty = 0;
        if(i16Duty > 100)
            i16Duty = 100;

        printf("i16Duty_limited: %d\r\n", i16Duty);

        /* Set PWM0 timer duty */
        PWM_SET_CMR(PWM0, 2, i16Duty);

        delay(200);
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
