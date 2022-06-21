#include <stdio.h>
#include "NuMicro.h"
#include "servo.h"

uint32_t servo_attached(STR_SERVO_T *pServo)
{
    return pServo->u32isAttached;
}

uint32_t servo_attach(STR_SERVO_T *pServo, PWM_T *pwm, uint32_t u32pwmChannelMask, volatile uint32_t *pu32pdio)
{
    /* Check if already configured */
    if (pServo->u32isAttached)
        return pServo->u32isAttached;
    
    pServo->pwm = pwm;
    pServo->u32pwmChannelMask = u32pwmChannelMask;
    pServo->pu32pdio = pu32pdio;

    pServo->u32gpioMfpSave = SYS->GPA_MFP0;

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
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT1, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 1 */
    PWM_EnableOutput(PWM0, BIT1);

    /* Enable PWM0 channel 1 counter */
    PWM_Start(PWM0, BIT1); // CNTEN1

    pServo->u32isAttached = 1;

    return pServo->u32isAttached;
}

uint8_t servo_write(STR_SERVO_T *pServo, uint8_t u8degree)
{
    /* Set PWM0 timer duty */
    // 0 degree start from (60/500)/(1/200)=0.6ms, end to 2.4ms (180 degree)
    // Tested with SAVOX SB-2270SG servo motor
    PWM_SET_CMR(PWM0, 1, 60 + u8degree);

    pServo->u8degree = u8degree;
}

uint8_t servo_read(STR_SERVO_T *pServo)
{
    return pServo->u8degree;
}

void servo_detach(STR_SERVO_T *pServo)
{
    SYS->GPA_MFP0 = pServo->u32gpioMfpSave;
}
