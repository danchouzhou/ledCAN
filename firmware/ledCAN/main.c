#include <stdio.h>
#include "NuMicro.h"
#include "NeoPixel.h"
#include "delay.h"
#include "pid.h"
#include "servo.h"
#include "nidec.h"
#include "user_init.h"

/* Hardware configuration */
#define LED_PIN             PA0
#define MAX_BRIGHTNESS      0x80 // 我們店裡的正常是半糖喔

/* CAN frame */
#define CMD_SET_SYNC_ID     0x5A
#define CMD_SYS_RESET       0xEE
#define MODE_IDEL           0x00
#define MODE_NEO_FILL       0x01
#define MODE_NEO_WIPE       0x02
#define MODE_NEO_BLINK      0x03
#define MODE_NEO_BREATH     0x04
#define MODE_NEO_SCROLL     0x05
#define MODE_ADC_PA1        0x06
#define MODE_SERVO_PA0      0x07
#define MODE_NIDEC_PA0      0x08
#define MODE_ENCODER        0x09

/* ISP */
#define V6M_AIRCR_VECTKEY_DATA            0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ             0x00000004UL

/* Create a NeoPixel object */
STR_NEOPIXEL_T pixels;

void colorWipe(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait);

/* Create a Servo object */
STR_SERVO_T servo = {0};

/* Create a Nidec motor object */
STR_NIDEC_T nidec = {0};

/* Create CAN message object */
STR_CANMSG_T modeMsg;
STR_CANMSG_T tMsg;

STR_PID_T PFM_PID = {0};

volatile uint32_t g_u32SyncFlag = 0;
volatile uint32_t g_u32ModeMsgFlag = 0;
volatile uint32_t g_u32AdCh0Data = 0;
volatile int64_t g_i64EncoderCnt = 0;

/* For encoder counting */
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
    STR_CANMSG_T rrMsg;

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
        g_u32ModeMsgFlag = 1;
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

void PID_Init()
{
    PID_SetPoint(&PFM_PID, 1365);

    PID_SetGain(&PFM_PID, -90, 0, 0);
}

void updateLen(STR_NEOPIXEL_T *pNeoPixel, uint16_t u16NewLen)
{
    if(NeoPixel_numPixels(&pixels) != u16NewLen)
    {
        NeoPixel_clear(&pixels);
        NeoPixel_show(&pixels);
        NeoPixel_updateLength(&pixels, u16NewLen);
    }
}

int main()
{
    /* Create a volatile pointer to modeMsg keep the value up to date */
    volatile STR_CANMSG_T *pModeMsg = &modeMsg;

    uint32_t u32SyncID = 0x200; // CAN sync ID
    uint32_t u32ModeID = 0; // CAN mode ID
    uint8_t u8Mode, u8LastMode = 0;
    uint8_t u8NumOfPixels, u8Red, u8Green, u8Blue; // Basic NeoPixel parameter
    uint8_t u8Delay, u8Second, u8LengthOfSnake;
    uint8_t u8Degree, u8Duty, u8Direction; // For Servo and Nidec motor

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

    /* Set message object 0 for synchronous arbitration = u32SyncID */
    CAN_SetRxMsg(CAN, MSG(0),CAN_STD_ID, u32SyncID);

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
        u8Mode = pModeMsg->Data[0];
        u8NumOfPixels = pModeMsg->Data[1];
        u8Red = pModeMsg->Data[2];
        u8Green = pModeMsg->Data[3];
        u8Blue = pModeMsg->Data[4];
        u8Delay = pModeMsg->Data[5];
        u8Second = pModeMsg->Data[5];
        u8LengthOfSnake = pModeMsg->Data[5];
        u8Degree = pModeMsg->Data[1];
        u8Duty = pModeMsg->Data[1];
        u8Direction = pModeMsg->Data[2];

        if (u8Mode == CMD_SET_SYNC_ID)
        {
            u32SyncID = pModeMsg->Data[1] & 0xFF;
            u32SyncID |= pModeMsg->Data[2] << 8;
            CAN_SetRxMsg(CAN, MSG(0),CAN_STD_ID, u32SyncID);
            pModeMsg->Data[0] = 0;
            u8Mode = 0;
        }

        if (u8Mode == CMD_SYS_RESET)
        {
            
        }

        if (g_u32SyncFlag == 1)
        {
            if (u8LastMode != u8Mode)
            {
                if (u8Mode != 7)
                    servo_detach(&servo);
                
                if (u8Mode != 8)
                    nidec_detach(&nidec);

                u8LastMode = pModeMsg->Data[0];
            }

            switch (u8Mode) {
                case MODE_IDEL: // idel
                    updateLen(&pixels, u8NumOfPixels);
                    NeoPixel_clear(&pixels);
                    NeoPixel_show(&pixels);
                    break;
                case MODE_NEO_FILL: // fill, numberOfLEDs, r, g, b
                    updateLen(&pixels, u8NumOfPixels);
                    NeoPixel_fill(&pixels, u8Red, u8Green, u8Blue, 0, u8NumOfPixels);
                    NeoPixel_show(&pixels);
                    break;
                case MODE_NEO_WIPE: // wipe, numberOfLEDs, r, g, b, interval (ms)
                    updateLen(&pixels, u8NumOfPixels);
                    colorWipe(&pixels, u8Red, u8Green, u8Blue, u8Delay);
                    break;
                case MODE_NEO_BLINK: // blink, numberOfLEDs, r, g, b, delay (second*10)
                    updateLen(&pixels, u8NumOfPixels);
                    NeoPixel_fill(&pixels, u8Red, u8Green, u8Blue, 0, NeoPixel_numPixels(&pixels));
                    NeoPixel_show(&pixels);
                    delay(u8Delay*10);
                    NeoPixel_clear(&pixels);
                    NeoPixel_show(&pixels);
                    break;
                case MODE_NEO_BREATH: // breath, numberOfLEDs, r, g, b, period (second)
                    updateLen(&pixels, u8NumOfPixels);
                    for(int i=0; i<MAX_BRIGHTNESS; i+=((u8Second>3)?1:5)) // Increase 5 if period <= 3 seconds
                    {
                        NeoPixel_setBrightness(&pixels, i);
                        NeoPixel_fill(&pixels, u8Red, u8Green, u8Blue, 0, NeoPixel_numPixels(&pixels));
                        NeoPixel_show(&pixels);
                        //delay((uint32_t)modeMsg.Data[5]*1000/512);
                        delayMicroseconds((uint32_t)u8Second*((u8Second>3)?3906:19530)); // 1000000/MAX_BRIGHTNESS/2
                    }
                    for(int i=MAX_BRIGHTNESS; i>=0; i-=((u8Second>3)?1:5)) // Decrease 5 if period <= 3 seconds
                    {
                        NeoPixel_setBrightness(&pixels, i);
                        NeoPixel_fill(&pixels, u8Red, u8Green, u8Blue, 0, NeoPixel_numPixels(&pixels));
                        NeoPixel_show(&pixels);
                        //delay((uint32_t)modeMsg.Data[5]*1000/512);
                        delayMicroseconds((uint32_t)u8Second*((u8Second>3)?3906:19530)); // 1000000/MAX_BRIGHTNESS/2
                    }
                    /* In the end, clear the persistence and set the brightness back to normal */
                    NeoPixel_clear(&pixels);
                    NeoPixel_show(&pixels);
                    NeoPixel_setBrightness(&pixels, MAX_BRIGHTNESS);
                    break;
                case MODE_NEO_SCROLL: // snake scroll, numberOfLEDs, r, g, b, length of snake, interval (ms)
                    if(modeMsg.Data[1] != NeoPixel_numPixels(&pixels))
                    {
                        NeoPixel_clear(&pixels);
                        NeoPixel_show(&pixels);
                    }
                    updateLen(&pixels, u8NumOfPixels);
                    NeoPixel_fill(&pixels, u8Red, u8Green, u8Blue, 0, u8NumOfPixels); // background color
                    for(int i=0; i < u8NumOfPixels; i++)
                    {
                        if (i > u8LengthOfSnake)
                            NeoPixel_setPixelColor(&pixels, i - u8LengthOfSnake, u8Red, u8Green, u8Blue);
                        NeoPixel_setPixelColor(&pixels, i, (((uint32_t)u8Red*2>255)?255:u8Red*2), (((uint32_t)u8Green*2>255)?255:u8Green*2), (((uint32_t)u8Blue*2>255)?255:u8Blue*2));
                        NeoPixel_show(&pixels);
                        delay(modeMsg.Data[6]);
                    }
                    for(int i=u8NumOfPixels - 1; i >= 0; i--)
                    {
                        if(i > u8LengthOfSnake)
                            NeoPixel_setPixelColor(&pixels, i - u8LengthOfSnake, (((uint32_t)u8Red*2>255)?255:u8Red*2), (((uint32_t)u8Green*2>255)?255:u8Green*2), (((uint32_t)u8Blue*2>255)?255:u8Blue*2));
                        NeoPixel_setPixelColor(&pixels, i, u8Red, u8Green, u8Blue);
                        NeoPixel_show(&pixels);
                        delay(modeMsg.Data[6]);
                    }
                    break;
                case MODE_ADC_PA1: // Get PA1 ADC value every 10ms
                    /* Set PA.1 to input mode */
                    GPIO_SetMode(PA, BIT1, GPIO_MODE_INPUT);

                    /* Configure the PA.1 ADC analog input pins.  */
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk)) |
                                    (SYS_GPA_MFP0_PA1MFP_ADC0_CH1);

                    /* Disable the PC.3, PC.4 digital input path to avoid the leakage current. */
                    GPIO_DISABLE_DIGITAL_PATH(PA, BIT1);

                    /* Add input channel */
                    ADC->ADCHER = ADC->ADCHER | BIT1;

                    tMsg.FrameType = CAN_DATA_FRAME;
                    tMsg.IdType = CAN_STD_ID;
                    tMsg.Id = u32ModeID;
                    
                    while(pModeMsg->Data[0] == 6) // leave if mode has been change
                    {
                        tMsg.DLC = 3;
                        tMsg.Data[0] = 6;
                        tMsg.Data[1] = (uint8_t)(g_u32AdCh0Data & 0xFF); // MSB
                        tMsg.Data[2] = (uint8_t)(g_u32AdCh0Data>>8 & 0x0F); // LSB
                        CAN_Transmit(CAN, MSG(2), &tMsg); // Use msg 2 transmit
                        delay(10);
                    }

                    /* Set the I/O mode to default */
                    ADC->ADCHER = ADC->ADCHER & (~BIT1);
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk));
                    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
                    GPIO_ENABLE_DIGITAL_PATH(PA, BIT1);

                    break;
                case MODE_SERVO_PA0: // servo, degree
                    servo_attach(&servo);
                    servo_write(&servo, u8Degree);
                    break;
                case MODE_NIDEC_PA0: // Nidec, duty (0-100), dir (0=CCW, 1=CW)
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
                case MODE_ENCODER: // Encoder
                    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));
                    GPIO_SetMode(PA, BIT0 | BIT1, GPIO_MODE_QUASI); // PA0, PA1
                    PA0=1; PA1=1; // Pull up the GPIO
                    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_64);
                    GPIO_ENABLE_DEBOUNCE(PA, BIT0);
                    GPIO_EnableInt(PA, 0, GPIO_INT_RISING);
                    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

                    tMsg.FrameType = CAN_DATA_FRAME;
                    tMsg.IdType = CAN_STD_ID;
                    tMsg.Id = u32ModeID;
                    tMsg.DLC = 8;

                    while(pModeMsg->Data[0] == 9) // leave if mode has been change
                    {
                        i64EncoderCntTmp = g_i64EncoderCnt;
                        g_i64EncoderCnt = 0;

                        if (pModeMsg->Data[1])
                            i64EncoderCntTmp = 0 - i64EncoderCntTmp;
                        tMsg.Data[0] = (uint8_t)(i64EncoderCntTmp & 0xFF); // MSB
                        tMsg.Data[1] = (uint8_t)(i64EncoderCntTmp>>8 & 0xFF);
                        tMsg.Data[2] = (uint8_t)(i64EncoderCntTmp>>16 & 0xFF);
                        tMsg.Data[3] = (uint8_t)(i64EncoderCntTmp>>24 & 0xFF);
                        tMsg.Data[4] = (uint8_t)(i64EncoderCntTmp>>32 & 0xFF);
                        tMsg.Data[5] = (uint8_t)(i64EncoderCntTmp>>40 & 0xFF);
                        tMsg.Data[6] = (uint8_t)(i64EncoderCntTmp>>48 & 0xFF);
                        tMsg.Data[7] = (uint8_t)(i64EncoderCntTmp>>56 & 0xFF); // LSB
                        CAN_Transmit(CAN, MSG(2), &tMsg); // Use msg 2 transmit
                        delay(100);
                    }

                    GPIO_DisableInt(PA, 0);
                    NVIC_DisableIRQ(GPIO_PAPB_IRQn);
                    PA0=0; PA1=0;

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
