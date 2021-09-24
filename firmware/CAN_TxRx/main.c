/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/19 3:12p $
 * @brief    Demonstrate CAN bus receive a message with normal mode
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
STR_CANMSG_T rrMsg;

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
    if(u32IIDR==5+1)
    {
        printf("Msg-5 INT and Callback \n");
        CAN_Receive(tCAN, 5,&rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR==31+1)
    {
        printf("Msg-31 INT and Callback \n");
        CAN_Receive(tCAN, 31,&rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
}


/**
  * @brief  CAN0_IRQ Handler.
  * @param  None.
  * @return None.
  */
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

    /* Enable UI2C0 clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

    /* Set PB multi-function pins for CAN0 TXD(PB.7) and RXD(PB.5) */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk | SYS_GPB_MFP1_PB7MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_CAN0_RXD | SYS_GPB_MFP1_PB7MFP_CAN0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/**
  * @brief      Init CAN driver
  */

void CAN_Init(CAN_T  *tCAN)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;
}

/**
  * @brief      Disable CAN
  * @details    Reset and clear all CAN control and disable CAN IP
  */

void CAN_STOP(CAN_T  *tCAN)
{
    /* Disable CAN0 Clock and Reset it */
    SYS->IPRST1 |= SYS_IPRST1_CAN0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_CAN0RST_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_CAN0CKEN_Msk;
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN       |\n");
    printf("|   Before execute it, please check description as below                 |\n");
    printf("|                                                                        |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver      |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS             |\n");
    printf("|   3.Check the terminal resistor of bus is connected                    |\n");
    printf("|   4.Using UART0 as print message port                                  |\n");
    printf("|                                                                        |\n");
    printf("|  |--------|       |-----------| CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |   CAN     |  CAN_H  |   CAN     |CAN_TX |        | |\n");
    printf("|  |  M0A21 |       |Transceiver|         |Transceiver|       |  M0A21 | |\n");
    printf("|  |        |<------|           |<------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |  CAN_L  |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|         |-----------|       |--------| |\n");
    printf("|   |                                                           |        |\n");
    printf("|   |                                                           |        |\n");
    printf("|   V                                                           V        |\n");
    printf("| UART0                                                         UART0    |\n");
    printf("|(print message)                                          (print message)|\n");
    printf("+------------------------------------------------------------------------+\n");
}

void SelectCANSpeed(CAN_T  *tCAN)
{
    uint32_t unItem;
    int32_t i32Err =0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  500Kbps\n");
    printf("[2]  250Kbps\n");
    printf("[3]  125Kbps\n");
    printf("[4]  100Kbps\n");
    printf("[5]   50Kbps\n");

    unItem = getchar();
    printf("%c\n",unItem) ;
    if(unItem=='1')
        i32Err = CAN_Open(tCAN,  500000, CAN_NORMAL_MODE);
    else if(unItem=='2')
        i32Err = CAN_Open(tCAN,  250000, CAN_NORMAL_MODE);
    else if(unItem=='3')
        i32Err = CAN_Open(tCAN,  125000, CAN_NORMAL_MODE);
    else if(unItem=='4')
        i32Err = CAN_Open(tCAN,  100000, CAN_NORMAL_MODE);
    else if(unItem=='5')
        i32Err = CAN_Open(tCAN,   50000, CAN_NORMAL_MODE);
    else
        i32Err = CAN_Open(tCAN, 1000000, CAN_NORMAL_MODE);

    if(i32Err<0)
        printf("Set CAN bit rate is fail\n");
}


void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;
    printf("Read ID=%8X, Type=%s, DLC=%d,Data=",Msg->Id,Msg->IdType?"EXT":"STD",Msg->DLC);
    for(i=0; i<Msg->DLC; i++)
        printf("%02X,",Msg->Data[i]);
    printf("\n\n");
}

/*----------------------------------------------------------------------------*/
/*  Send Rx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/

void Test_NormalMode_Rx(CAN_T *tCAN)
{
    if(CAN_SetRxMsg(tCAN, MSG(0),CAN_STD_ID, 0x7FF) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

    if(CAN_SetRxMsg(tCAN, MSG(5),CAN_STD_ID, 0x7FF) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

    if(CAN_SetRxMsg(tCAN, MSG(31),CAN_STD_ID, 0x7FF) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }
    /*Choose one mode to test*/
#if 1
    /* Polling Mode */
    while(1)
    {
        while(tCAN->IIDR ==0);            /* Wait IDR is changed */
        printf("IDR = %x\n",tCAN->IIDR);
        CAN_Receive(tCAN, tCAN->IIDR -1, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
#else
    /* INT Mode */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk);
    NVIC_SetPriority(CAN0_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);
    printf("Wait Msg\n");
    printf("Enter any key to exit\n");
    getchar();
#endif
}

int main()
{
    CAN_T *tCAN;

    tCAN = (CAN_T *) CAN;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Select CAN Multi-Function */
    CAN_Init(tCAN);
    Note_Configure();
    SelectCANSpeed(tCAN);

    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|  Receive a message by normal mode                                 |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("Press any key to continue ...\n\n");
    getchar();
    Test_NormalMode_Rx(tCAN);

    while(1) ;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
