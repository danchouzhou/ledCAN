#include <stdio.h>
#include "NuMicro.h"
#include "servo.h"

uint32_t servo_attached(STR_SERVO_T *pServo)
{
    return pServo->u32isAttached;
}

uint32_t servo_attach(STR_SERVO_T *pServo)
{
    /* Check if already configured */
    if (pServo->u32isAttached)
        return pServo->u32isAttached;

    /* Save the MFP settings */
    pServo->u32gpioMFPsave = SYS->GPA_MFP0;

    /* Set PA multi-function pins for PWM0 Channel 1 */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA0MFP_PWM0_CH1);

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 1, 479); // 48MHz/480=100KHz

    /* Set counter to up counting */
    PWM0->CTL1 = (PWM0->CTL1 & ~(PWM_CTL1_CNTTYPE0_Msk)) | (PWM_UP_COUNTER << PWM_CTL1_CNTTYPE0_Pos);

    /* Set PWM0 timer duty */
    // 0 degree start from (60/500)/(1/200)=0.6ms, end to 2.4ms (180 degree)
    // Tested with SAVOX SB-2270SG servo motor
    PWM_SET_CMR(PWM0, 1, 60);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 1, 499); // 100KHz/500=200Hz

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_1_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 1 */
    PWM_EnableOutput(PWM0, PWM_CH_1_MASK);

    /* Enable PWM0 channel 1 counter (PWM_CNT0) */
    PWM_Start(PWM0, PWM_CH_1_MASK);

    /* Mark as attach */
    pServo->u32isAttached = 1;

    /* Return success */
    return pServo->u32isAttached;
}

uint8_t servo_write(STR_SERVO_T *pServo, uint8_t u8degree)
{
    /* Set PWM0 timer duty */
    // 0 degree start from (60/500)/(1/200)=0.6ms, end to 2.4ms (180 degree)
    // Tested with SAVOX SB-2270SG servo motor
    PWM_SET_CMR(PWM0, 1, 60 + u8degree);

    pServo->u8degree = u8degree;

    return pServo->u8degree;
}

uint8_t servo_read(STR_SERVO_T *pServo)
{
    return pServo->u8degree;
}

void servo_detach(STR_SERVO_T *pServo)
{
    /* Mark as detach */
    pServo->u32isAttached = 0;

    /* Stop the PWM */
    PWM_ForceStop(PWM0, PWM_CH_1_MASK);

    /* Restore MFP settings */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) | 
                    (pServo->u32gpioMFPsave & SYS_GPA_MFP0_PA0MFP_Msk);
}
