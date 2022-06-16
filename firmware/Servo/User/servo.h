#ifndef _SERVO_H_
#define _SERVO_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    PWM_T * PWM;
    uint32_t channelmask;
} STR_SERVO_T;

void servo_attach();

#ifdef __cplusplus
}
#endif

#endif
