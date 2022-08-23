#include <stdio.h>
#include "NuMicro.h"
#include "user_init.h"

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

    /* Enable CAN0 clock */
    CLK_EnableModuleClock(CAN0_MODULE);

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

    /* Set PA0, PA1 multi-function pins as GPIO for NeoPixel */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));

	/* Set PC multi-function pins for PWM0 Channel 2/3 */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC0MFP_Msk | SYS_GPC_MFP0_PC1MFP_Msk)) |
                    (SYS_GPC_MFP0_PC0MFP_PWM0_CH2 | SYS_GPC_MFP0_PC1MFP_PWM0_CH3);

    /* Set PC.3, PC.4 to input mode */
    GPIO_SetMode(PC, BIT3 | BIT4, GPIO_MODE_INPUT);

    /* Configure the PC.3, PC.4 ADC analog input pins.  */
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(SYS_GPC_MFP0_PC3MFP_Msk)) |
                    (SYS_GPC_MFP0_PC3MFP_ADC0_CH12);
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(SYS_GPC_MFP1_PC4MFP_Msk)) |
                    (SYS_GPC_MFP1_PC4MFP_ADC0_CH13);

    /* Disable the PC.3, PC.4 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PC, BIT3 | BIT4);

    /* Set PA multi-function pins for CAN0 TXD(PA.5) and RXD(PA.4) */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_CAN0_RXD | SYS_GPA_MFP1_PA5MFP_CAN0_TXD);
}

void CAN_Init()
{
    if(CAN_Open(CAN,  500000, CAN_NORMAL_MODE) < 0)
        printf("Set CAN bit rate is fail\n");

    /* Enable CAN interrupt */
    CAN_EnableInt(CAN, CAN_CON_IE_Msk);
    NVIC_SetPriority(CAN0_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);
}

void PWM0_Init()
{
    /* Set PWM mode as complementary mode for channel 2 and 3 */
    PWM0->CTL1 |= PWM_CTL1_PWMMODE2_Msk;

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 0);

    /* Set counter to up counting */
    PWM0->CTL1 = (PWM0->CTL1 & ~(PWM_CTL1_CNTTYPE2_Msk)) | (PWM_UP_COUNTER << PWM_CTL1_CNTTYPE2_Pos);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 2, 24);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 2, 2400); // PFM

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set PWM0 dead-time */
    PWM_EnableDeadZone(PWM0, 2, 4);

    /* Lock protected registers */
    SYS_LockReg();

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_2_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 2 and 3 */
    PWM_EnableOutput(PWM0, 0xc);
}

void ADC_Init()
{
    /* Enable ADC converter */
    ADC_POWER_ON(ADC);
    
    /* Set input mode as single-end, Single mode, and select channel 12 and 13 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT12);

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
    /* Set timer0 periodic time-out frequency is 20KHz */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 20000);

    /* Enable timer interrupt trigger ADC */
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSRC_TIMEOUT_EVENT);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_ADC);
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
    //while(!IsDebugFifoEmpty());

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;

    return 0;
}

void FMC_Init()
{
    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable Data Flash and set base address. */
    if (check_config_bits(CONFIG0, CONFIG1) < 0)
    {
        if(set_config_bits(CONFIG0, CONFIG1))
        {
            printf("Failed to set Data Flash base address!\n");
            return;
        }
    }

    printf("Config bits check passed.\r\n");
}
