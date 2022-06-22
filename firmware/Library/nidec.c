#include <stdio.h>
#include "NuMicro.h"
#include "nidec.h"

uint32_t nidec_attached(STR_NIDEC_T *pNidec)
{
    return pNidec->u32isAttached;
}

uint32_t nidec_attach(STR_NIDEC_T *pNidec)
{
    /* Check if already configured */
    if (pNidec->u32isAttached)
        return pNidec->u32isAttached;

    /* Save the MFP settings */
    pNidec->u32gpioMFPsave = SYS->GPA_MFP0;

    /* Set PA multi-function pins for PWM0 Channel 1 */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA0MFP_PWM0_CH1);

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 1, 47); // 48MHz/(47+1)=1MHz

    /* Set counter to up counting */
    PWM0->CTL1 = (PWM0->CTL1 & ~(PWM_CTL1_CNTTYPE0_Msk)) | (PWM_UP_COUNTER << PWM_CTL1_CNTTYPE0_Pos);

    /* Set PWM0 duty */
    PWM_SET_CMR(PWM0, 1, 0);

    /* Set PWM0 period */
    PWM_SET_CNR(PWM0, 1, 99); // 1MHz/(99+1)=10KHz

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_1_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 1 */
    PWM_EnableOutput(PWM0, PWM_CH_1_MASK);

    /* Enable PWM0 channel 1 counter (PWM_CNT0) */
    PWM_Start(PWM0, PWM_CH_1_MASK);

    /* Mark as attach */
    pNidec->u32isAttached = 1;

    /* Return success */
    return pNidec->u32isAttached;
}

uint8_t nidec_write(STR_NIDEC_T *pNidec, uint8_t u8duty, uint8_t u8direction)
{
    /* Set the direction */
    PA1 = u8direction & 0x1;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 1, u8duty);

    pNidec->u8duty = u8duty;

    return pNidec->u8duty;
}

uint8_t nidec_read(STR_NIDEC_T *pNidec)
{
    return pNidec->u8duty;
}

void nidec_detach(STR_NIDEC_T *pNidec)
{
    /* Mark as detach */
    pNidec->u32isAttached = 0;

    /* Stop the PWM */
    PWM_ForceStop(PWM0, PWM_CH_1_MASK);

    /* Restore MFP settings */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) | 
                    (pNidec->u32gpioMFPsave & SYS_GPA_MFP0_PA0MFP_Msk);
}
