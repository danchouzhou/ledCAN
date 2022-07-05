#include <stdio.h>
#include "NuMicro.h"
#include "NeoPixel.h"
#include "delay.h"
#include "pid.h"
#include "servo.h"
#include "nidec.h"

/* Hardware configuration */
#define LED_PIN             PA0
#define MAX_BRIGHTNESS      0x80 // 我們店裡的正常是半糖喔

#define DATA_FLASH_BASE     0x7E00
#define MODE_ID_OFFSET      0
/* CONFIG0: Enable data flash. */
#define CONFIG0             0xFDFFF97E
/* CONFIG1: Set Data Flash Base Address to 0x7E00 (size 512 bytes) */
#define CONFIG1             DATA_FLASH_BASE

/* Create a NeoPixel object */
STR_NEOPIXEL_T pixels;

void colorWipe(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait);

/* Create a Servo object */
STR_SERVO_T servo = {0};

/* Create a Nidec motor object */
STR_NIDEC_T nidec = {0};

/* Create CAN message object */
STR_CANMSG_T rrMsg;
STR_CANMSG_T modeMsg;
STR_CANMSG_T ttMsg;

STR_PID_T PFM_PID = {0};

volatile uint32_t g_u32SyncFlag = 0;
volatile uint32_t g_u32AdCh0Data = 0;
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

void ADC_IRQHandler(void)
{
    int32_t i32ConversionData, i32Comp, i32PFM, i32PWM;

    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */

    if(ADC_IS_DATA_VALID(ADC, 12))
        i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 12);

    if(ADC_IS_DATA_VALID(ADC, 1))
        g_u32AdCh0Data = ADC_GET_CONVERSION_DATA(ADC, 1);

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
}

void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;
    printf("Read ID=%8X, Type=%s, DLC=%d,Data=",Msg->Id,Msg->IdType?"EXT":"STD",Msg->DLC);
    for(i=0; i<Msg->DLC; i++)
        printf("%02X,",Msg->Data[i]);
    printf("\n\n");
}

void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR==1)
    {
        //printf("Msg-0 INT and Callback\n");
        CAN_Receive(tCAN, 0,&rrMsg);
        //CAN_ShowMsg(&rrMsg);
        g_u32SyncFlag = 1;
    }
    if(u32IIDR==2)
    {
        //printf("Msg-1 INT and Callback\n");
        CAN_Receive(tCAN, 1,&modeMsg);
        //CAN_ShowMsg(&modeMsg);
    }
}

void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear Rx Ok status*/

            //printf("RX OK INT\n") ;
        }

        if(CAN->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear Tx Ok status*/

            //printf("TX OK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN->STATUS & CAN_STATUS_EWARN_Msk)
        {
            //printf("EWARN INT\n") ;

            /* Do Init to release busoff pin */
            CAN->CON = (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
            CAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            while(CAN->CON & CAN_CON_INIT_Msk);
        }

        if(CAN->STATUS & CAN_STATUS_BOFF_Msk)
        {
            //printf("BOFF INT\n") ;
        }
    }
    else if (u8IIDRstatus!=0)
    {
        //printf("=> Interrupt Pointer = %d\n",CAN->IIDR -1);

        CAN_MsgInterrupt(CAN, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN, ((CAN->IIDR) -1));      /* Clear Interrupt Pending */

    }
    else if(CAN->WU_STATUS == 1)
    {
        //printf("Wake up\n");

        CAN->WU_STATUS = 0;                       /* Write '0' to clear */
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
    /* Set PWM mode as complementary mode */
    //PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);
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

void PID_Init()
{
    PID_SetPoint(&PFM_PID, 1365);

    PID_SetGain(&PFM_PID, -90, 0, 0);
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

void updateLen(STR_NEOPIXEL_T *pNeoPixel, uint16_t u16NewLen)
{
    if(NeoPixel_numPixels(&pixels) != u16NewLen)
        NeoPixel_updateLength(&pixels, u16NewLen);
}

int main()
{
    /* Create a volatile pointer to modeMsg keep the value up to date */
    volatile STR_CANMSG_T *pModeMsg = &modeMsg;

    uint32_t u32ModeID = 0;
    uint32_t u32Mode = 0;

    int64_t i64EncoderCntTmp;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initial system setup */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\r\n");

    /* Initial flash configuration */
    FMC_Init();

    /* Read mode ID in data flash */
    u32ModeID = FMC_Read(DATA_FLASH_BASE + MODE_ID_OFFSET);
    printf("Mode ID is: 0x%x\r\n", u32ModeID);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    /* Initial CAN module */
    CAN_Init();

    /* Set message object 0 for synchronous arbitration = 0x200 (512) */
    CAN_SetRxMsg(CAN, MSG(0),CAN_STD_ID, 0x200);

    /* Set message object 1 for mode setup arbitration = from data flash */
    CAN_SetRxMsg(CAN, MSG(1),CAN_STD_ID, u32ModeID);

    /* Initialize NeoPixel, will update length later in main functions */
    NeoPixel_begin(&pixels, 0, &LED_PIN, NEO_GRB);

    /* Turn OFF all pixels ASAP */
    //NeoPixel_show(&pixels);

    /* Set the maxmium brightness for NeoPixels */
    NeoPixel_setBrightness(&pixels, MAX_BRIGHTNESS);

    /* Initial PWM0 */
    PWM0_Init();

    /* Initial PID controller */
    PID_Init();

    /* Init TIMER0 for ADC */
    TIMER0_Init();

    /* Initial ADC */
    ADC_Init();

    /* Enable Timer0 counter */
    TIMER_Start(TIMER0);

    /* Enable PWM0 channel 2 counter */
    PWM_Start(PWM0, PWM_CH_2_MASK); // CNTEN2

    /* Got no where to go, just loop forever */
    while(1)
    {
        if (g_u32SyncFlag == 1)
        {
            if (u32Mode != pModeMsg->Data[0])
            {
                if (pModeMsg->Data[0] != 7)
                    servo_detach(&servo);
                
                if (pModeMsg->Data[0] != 8)
                    nidec_detach(&nidec);

                u32Mode = pModeMsg->Data[0];
            }

            switch (pModeMsg->Data[0]) {
                case 0: // idel
                    updateLen(&pixels, modeMsg.Data[1]);
                    NeoPixel_clear(&pixels);
                    NeoPixel_show(&pixels);
                    break;
                case 1: // fill, numberOfLEDs, r, g, b
                    updateLen(&pixels, modeMsg.Data[1]);
                    NeoPixel_fill(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], 0, NeoPixel_numPixels(&pixels));
                    NeoPixel_show(&pixels);
                    NeoPixel_clear(&pixels);
                    break;
                case 2: // wipe, numberOfLEDs, r, g, b, interval (ms)
                    updateLen(&pixels, modeMsg.Data[1]);
                    colorWipe(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], modeMsg.Data[5]);
                    break;
                case 3: // blink, numberOfLEDs, r, g, b, delay (second*10)
                    updateLen(&pixels, modeMsg.Data[1]);
                    NeoPixel_fill(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], 0, NeoPixel_numPixels(&pixels));
                    NeoPixel_show(&pixels);
                    delay(modeMsg.Data[5]*10);
                    NeoPixel_clear(&pixels);
                    NeoPixel_show(&pixels);
                    break;
                case 4: // breath, numberOfLEDs, r, g, b, period (second)
                    updateLen(&pixels, modeMsg.Data[1]);
                    for(int i=0; i<MAX_BRIGHTNESS; i+=((modeMsg.Data[5]>3)?1:5)) // Increase 5 if period <= 3 seconds
                    {
                        NeoPixel_setBrightness(&pixels, i);
                        NeoPixel_fill(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], 0, NeoPixel_numPixels(&pixels));
                        NeoPixel_show(&pixels);
                        //delay((uint32_t)modeMsg.Data[5]*1000/512);
                        delayMicroseconds((uint32_t)modeMsg.Data[5]*((modeMsg.Data[5]>3)?3906:19530)); // 1000000/MAX_BRIGHTNESS/2
                    }
                    for(int i=MAX_BRIGHTNESS; i>=0; i-=((modeMsg.Data[5]>3)?1:5)) // Decrease 5 if period <= 3 seconds
                    {
                        NeoPixel_setBrightness(&pixels, i);
                        NeoPixel_fill(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], 0, NeoPixel_numPixels(&pixels));
                        NeoPixel_show(&pixels);
                        //delay((uint32_t)modeMsg.Data[5]*1000/512);
                        delayMicroseconds((uint32_t)modeMsg.Data[5]*((modeMsg.Data[5]>3)?3906:19530)); // 1000000/MAX_BRIGHTNESS/2
                    }
                    NeoPixel_clear(&pixels);
                    NeoPixel_setBrightness(&pixels, MAX_BRIGHTNESS);
                    break;
                case 5: // snake scroll, numberOfLEDs, r, g, b, length of snake, interval (ms)
                    if(modeMsg.Data[1] != NeoPixel_numPixels(&pixels))
                    {
                        NeoPixel_clear(&pixels);
                        NeoPixel_show(&pixels);
                    }
                    updateLen(&pixels, modeMsg.Data[1]);
                    NeoPixel_fill(&pixels, modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4], 0, NeoPixel_numPixels(&pixels));
                    for(int i=0; i<NeoPixel_numPixels(&pixels); i++)
                    {
                        if(i<modeMsg.Data[6])
                        {
                            NeoPixel_setPixelColor(&pixels, i, (((uint32_t)modeMsg.Data[2]*2>255)?255:modeMsg.Data[2]*2), (((uint32_t)modeMsg.Data[3]*2>255)?255:modeMsg.Data[3]*2), (((uint32_t)modeMsg.Data[4]*2>255)?255:modeMsg.Data[4]*2));
                        }
                        else
                        {
                            NeoPixel_setPixelColor(&pixels, i, (((uint32_t)modeMsg.Data[2]*2>255)?255:modeMsg.Data[2]*2), (((uint32_t)modeMsg.Data[3]*2>255)?255:modeMsg.Data[3]*2), (((uint32_t)modeMsg.Data[4]*2>255)?255:modeMsg.Data[4]*2));
                            NeoPixel_setPixelColor(&pixels, i-modeMsg.Data[5], modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4]);
                        }
                        NeoPixel_show(&pixels);
                        delay(modeMsg.Data[5]);
                    }
                    for(int i=NeoPixel_numPixels(&pixels)-modeMsg.Data[6]; i>=-(int)modeMsg.Data[6]; i--)
                    {
                        if(i<0)
                        {
                            NeoPixel_setPixelColor(&pixels, i+modeMsg.Data[5], modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4]);
                        }
                        else
                        {
                            NeoPixel_setPixelColor(&pixels, i, (((uint32_t)modeMsg.Data[2]*2>255)?255:modeMsg.Data[2]*2), (((uint32_t)modeMsg.Data[3]*2>255)?255:modeMsg.Data[3]*2), (((uint32_t)modeMsg.Data[4]*2>255)?255:modeMsg.Data[4]*2));
                            NeoPixel_setPixelColor(&pixels, i+modeMsg.Data[5], modeMsg.Data[2], modeMsg.Data[3], modeMsg.Data[4]);
                        }
                        NeoPixel_show(&pixels);
                        delay(modeMsg.Data[5]);
                    }
                    break;
                case 6: // Get PA1 ADC value every 10ms
                    /* Set PA.1 to input mode */
                    GPIO_SetMode(PA, BIT1, GPIO_MODE_INPUT);

                    /* Configure the PA.1 ADC analog input pins.  */
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk)) |
                                    (SYS_GPA_MFP0_PA1MFP_ADC0_CH1);

                    /* Disable the PC.3, PC.4 digital input path to avoid the leakage current. */
                    GPIO_DISABLE_DIGITAL_PATH(PA, BIT1);

                    /* Add input channel */
                    ADC->ADCHER = ADC->ADCHER | BIT1;

                    modeMsg.FrameType = CAN_DATA_FRAME;
                    modeMsg.IdType = CAN_STD_ID;
                    modeMsg.Id = u32ModeID;
                    
                    while(pModeMsg->Data[0] == 6) // leave if mode has been change
                    {
                        modeMsg.DLC = 3;
                        //modeMsg.Data[0] = 6;
                        modeMsg.Data[1] = (uint8_t)(g_u32AdCh0Data & 0xFF);
                        modeMsg.Data[2] = (uint8_t)(g_u32AdCh0Data>>8 & 0x0F);
                        CAN_Transmit(CAN, MSG(2), &modeMsg); // Use msg 2 transmit
                        delay(10);
                    }

                    /* Set the I/O mode to default */
                    ADC->ADCHER = ADC->ADCHER & (~BIT1);
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk));
                    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
                    GPIO_ENABLE_DIGITAL_PATH(PA, BIT1);

                    break;
                case 7: // servo, degree
                    servo_attach(&servo);
                    servo_write(&servo, modeMsg.Data[1]);
                    break;
                case 8: // Nidec, duty (0-100), dir (0=CCW, 1=CW)
                    nidec_attach(&nidec);
                    nidec_write(&nidec, modeMsg.Data[1], modeMsg.Data[2] & 0x1);
                    
                    // Stop the Nidec motor if there is no update
                    modeMsg.Data[1] = 0;
                    while(modeMsg.Data[1] == 0 || g_u32SyncFlag == 0)
                    {
                        if (timeout(200) == 0)
                        {
                            nidec_write(&nidec, modeMsg.Data[1], modeMsg.Data[2] & 0x1);
                        }
                    }

                    timeout(0); // Stop the timeout progress

                    break;
                case 9: // Encoder
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));
                    GPIO_SetMode(PA, BIT0 | BIT1, GPIO_MODE_QUASI); // PA0, PA1
                    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_64);
                    GPIO_ENABLE_DEBOUNCE(PA, BIT0);
                    GPIO_EnableInt(PA, 0, GPIO_INT_RISING);
                    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

                    ttMsg.FrameType = CAN_DATA_FRAME;
                    ttMsg.IdType = CAN_STD_ID;
                    ttMsg.Id = u32ModeID;
                    ttMsg.DLC = 8;

                    while(pModeMsg->Data[0] == 9) // leave if mode has been change
                    {
                        i64EncoderCntTmp = g_i64EncoderCnt;
                        g_i64EncoderCnt = 0;
                        ttMsg.Data[0] = 6;
                        ttMsg.Data[1] = (uint8_t)(g_i64EncoderCnt & 0xFF);
                        ttMsg.Data[2] = (uint8_t)(g_i64EncoderCnt>>8 & 0xFF);
                        ttMsg.Data[3] = (uint8_t)(g_i64EncoderCnt>>16 & 0xFF);
                        ttMsg.Data[4] = (uint8_t)(g_i64EncoderCnt>>32 & 0xFF);
                        ttMsg.Data[5] = (uint8_t)(g_i64EncoderCnt>>64 & 0xFF);
                        ttMsg.Data[6] = (uint8_t)(g_i64EncoderCnt>>128 & 0xFF);
                        ttMsg.Data[7] = (uint8_t)(g_i64EncoderCnt>>256 & 0xFF);
                        CAN_Transmit(CAN, MSG(2), &ttMsg); // Use msg 2 transmit
                        delay(100);
                    }

                    GPIO_DisableInt(PA, 0);
                    NVIC_DisableIRQ(GPIO_PAPB_IRQn);

                    break;                
                default: 
                    break;
            }
            
            /* Clear synchronous flag */
            g_u32SyncFlag = 0;
        }
    }
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait) {
    for(int i=0; i<NeoPixel_numPixels(pNeoPixel); i++) {  // For each pixel in strip...
        NeoPixel_setPixelColor(pNeoPixel, i, r, g, b);  //  Set pixel's color (in RAM)
        NeoPixel_show(pNeoPixel);                       //  Update strip to match
        delay(wait);                                       //  Pause for a moment
    }
}
