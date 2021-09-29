#include <stdio.h>
#include "NuMicro.h"

STR_CANMSG_T rrMsg;
STR_CANMSG_T modeMsg;

void CAN_ShowMsg(STR_CANMSG_T* Msg);

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

    /* Lock protected registers */
    SYS_LockReg();
}

void CAN_Init(CAN_T *tCAN)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;
    
    if(CAN_Open(tCAN,  500000, CAN_NORMAL_MODE) < 0)
        printf("Set CAN bit rate is fail\n");

    /* Enable CAN interrupt */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk);
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

int main()
{
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");

    /* Initial CAN module */
    CAN_Init(CAN);

    /* Set message object 0 arbitration = 0x200 (512) */
    CAN_SetRxMsg(CAN, MSG(0),CAN_STD_ID, 0x200);

    /* Set message object 0 arbitration = 0x201 (513) */
    CAN_SetRxMsg(CAN, MSG(1),CAN_STD_ID, 0x201);

    while(1) ;
}
