/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x34
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define V6M_AIRCR_VECTKEY_DATA            0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ             0x00000004UL
#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/

/* Declare a CAN message structure */
STR_CANMSG_T rrMsg;
volatile uint8_t u8CAN_PackageFlag = 0, u8CAN_AckFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if (u32IIDR == 1)
    {
        CAN_Receive(tCAN, 0, &rrMsg);
        u8CAN_PackageFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;
    u8IIDRstatus = CAN->IIDR;

    if (u8IIDRstatus == 0x00008000)       /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if (CAN->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            u8CAN_AckFlag = 0;
        }
    }
    else if (u8IIDRstatus != 0)
    {
        CAN_MsgInterrupt(CAN, u8IIDRstatus);
        CAN_CLR_INT_PENDING_BIT(CAN, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable CAN0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;

    /* Update System Core Clock */
    SystemCoreClock = __HIRC;
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for CAN0 TXD(PA.5) and RXD(PA.4) */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_CAN0_RXD | SYS_GPA_MFP1_PA5MFP_CAN0_TXD);
}

/*----------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                         */
/*----------------------------------------------------------------------------*/
void CAN_Package_ACK(CAN_T *tCAN)
{
    STR_CANMSG_T tMsg;
    u8CAN_AckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType    = CAN_STD_ID;
    tMsg.Id        = Device0_ISP_ID;
    tMsg.DLC       = CAN_ISP_DtatLength;
    memcpy(&tMsg.Data, &rrMsg.Data, 8);

    if (CAN_Transmit(tCAN, MSG(5), &tMsg) == FALSE)   /* Configure Msg RAM and send the Msg in the RAM */
    {
        return;
    }

    while (u8CAN_AckFlag);
}

void CAN_Init(void)
{

    /* Set CAN target baud-rate 500KHz */
    CAN_EnterInitMode(CAN, (uint8_t)0);
    CAN->BTIME = 0x1C45;
    CAN_LeaveInitMode(CAN);

    CAN_EnableInt(CAN, (CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk));
    NVIC_EnableIRQ(CAN0_IRQn);
    /* Set CAN reveive message */
    CAN_SetRxMsg(CAN, MSG(0), CAN_STD_ID, Master_ISP_ID);
}

int main(void)
{
    uint32_t  Address, Data, u32Tmp;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Enable FMC ISP function */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_CFG_UPDATE();
    /* Init CAN port */
    CAN_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (u8CAN_PackageFlag == 1)
        {
            break;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

    /* stat update program */
    while (1)
    {
        if (u8CAN_PackageFlag)
        {
            u8CAN_PackageFlag = 0;
            memcpy(&Address, &rrMsg.Data[0], 4);
            memcpy(&Data, &rrMsg.Data[4], 4);

            if (Address == CMD_GET_DEVICEID)
            {
                u32Tmp = SYS->PDID;
                memcpy(&rrMsg.Data[4], &u32Tmp, 4);
            }
            else if (Address == CMD_READ_CONFIG)
            {
                u32Tmp = FMC_Read(Data);
                memcpy(&rrMsg.Data[4], &u32Tmp, 4);
            }
            else if (Address == CMD_RUN_APROM)
            {
                break;
            }
            else
            {
                if ((Address % FMC_FLASH_PAGE_SIZE) == 0)
                {
                    FMC_Erase(Address);
                }

                FMC_Write(Address, Data);         /* program ROM */
                Data = FMC_Read(Address);
                memcpy(&rrMsg.Data[4], &Data, 4); /* update data */
            }

            CAN_Package_ACK(CAN);            /* send CAN ISP Package (ACK) */
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
