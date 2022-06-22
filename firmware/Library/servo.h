#ifndef _SERVO_H_
#define _SERVO_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32_t u32gpioMFPsave;
    uint8_t u8degree;
    uint32_t u32isAttached;
} STR_SERVO_T;

uint32_t servo_attach(STR_SERVO_T *pServo);
uint8_t servo_write(STR_SERVO_T *pServo, uint8_t u8degree);
uint8_t servo_read(STR_SERVO_T *pServo);
void servo_detach(STR_SERVO_T *pServo);

#ifdef __cplusplus
}
#endif

#endif
