#include <stdio.h>
#include "NuMicro.h"
#include "delay.h"

#define DATA_FLASH_BASE     0x7E00
#define MODE_ID_OFFSET      0
/* CONFIG0: Enable data flash. */
/* This value need to be change into 0xFDFFF97E in final product to disable reset pin and extern POR to 26.6ms */
#define CONFIG0             0xFFFFFB7E
/* CONFIG1: Set Data Flash Base Address to 0x7E00 (size 512 bytes) */
#define CONFIG1             DATA_FLASH_BASE

STR_CANMSG_T rrMsg;
STR_CANMSG_T modeMsg;

void CAN_ShowMsg(STR_CANMSG_T* Msg);

volatile uint32_t g_u32SyncFlag = 0;
volatile uint32_t g_u32AdCh0Data = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR==1)
    {
        printf("Msg-0 INT and Callback\n");
        CAN_Receive(tCAN, 0,&rrMsg);
        CAN_ShowMsg(&rrMsg);
        g_u32SyncFlag = 1;
    }
    if(u32IIDR==2)
    {
        printf("Msg-1 INT and Callback\n");
        CAN_Receive(tCAN, 1,&modeMsg);
        CAN_ShowMsg(&modeMsg);
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

            printf("RX OK INT\n") ;
        }

        if(CAN->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear Tx Ok status*/

            printf("TX OK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;

            /* Do Init to release busoff pin */
            CAN->CON = (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
            CAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            while(CAN->CON & CAN_CON_INIT_Msk);
        }

        if(CAN->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
    }
    else if (u8IIDRstatus!=0)
    {
        printf("=> Interrupt Pointer = %d\n",CAN->IIDR -1);

        CAN_MsgInterrupt(CAN, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN, ((CAN->IIDR) -1));      /* Clear Interrupt Pending */

    }
    else if(CAN->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN->WU_STATUS = 0;                       /* Write '0' to clear */
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

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

void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;
    printf("Read ID=%8X, Type=%s, DLC=%d,Data=",Msg->Id,Msg->IdType?"EXT":"STD",Msg->DLC);
    for(i=0; i<Msg->DLC; i++)
        printf("%02X,",Msg->Data[i]);
    printf("\n\n");
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

int main()
{
    /* Create a volatile pointer to modeMsg keep the value up to date */
    volatile STR_CANMSG_T *pModeMsg = &modeMsg;

    uint32_t u32ModeID = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initialize system setup */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");

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

    /* Set message object 0 for synchronous, arbitration = 0x200 (512) */
    CAN_SetRxMsg(CAN, MSG(0),CAN_STD_ID, 0x200);

    /* Set message object 1 for mode. Arbitration read from flash */
    CAN_SetRxMsg(CAN, MSG(1),CAN_STD_ID, u32ModeID);

    while(1)
    {
        if (g_u32SyncFlag == 1)
        {
            /* Clear synchronous flag */
            g_u32SyncFlag = 0;

            printf("Mode: %d\r\n", pModeMsg->Data[0]);

            switch (pModeMsg->Data[0]) {
                case 0: // idel
                    printf("Perform mode 0: idel\r\n");
                    break;
                case 1: // fill, numberOfLEDs, r, g, b
                    printf("Perform mode 1: fill\r\n");
                    break;
                case 2: // wipe, numberOfLEDs, r, g, b, interval (ms)
                    printf("Perform mode 2: wipe\r\n");
                    break;
                case 3: // blink, numberOfLEDs, r, g, b, delay (second*10)
                    printf("Perform mode 3: blink\r\n");
                    break;
                case 4: // breath, numberOfLEDs, r, g, b, period (second)
                    printf("Perform mode 4: breath\r\n");
                    break;
                case 5: // snake scroll, numberOfLEDs, r, g, b, length of snake, interval (ms)
                    printf("Perform mode 5: snake scroll\r\n");
                    break;
                case 6: // Send PA0 ADC value every 500ms
                    printf("Perform mode 6: read A/D value\r\n");
                    g_u32AdCh0Data = 1234; // transmit test value
                    modeMsg.FrameType = CAN_DATA_FRAME;
                    modeMsg.IdType = CAN_STD_ID;
                    modeMsg.Id = u32ModeID;
                    modeMsg.DLC = 3;
                    
                    while(pModeMsg->Data[0] == 6) // leave if mode has been change
                    {
                        //modeMsg.Data[0] = 6;
                        modeMsg.Data[1] = (uint8_t)(g_u32AdCh0Data & 0xFF);
                        modeMsg.Data[2] = (uint8_t)(g_u32AdCh0Data>>8 & 0x0F);
                        CAN_Transmit(CAN, MSG(1), &modeMsg);
                        delay(500);
                    }
                    break;
                
                default: 
                    break;
            }
        }
    }
}
