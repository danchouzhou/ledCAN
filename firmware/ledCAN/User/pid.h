#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /* Controller parameter */
    int32_t i32SetPoint;
    int32_t i32Kp;
    int32_t i32Ki;
    int32_t i32Kd;
    /* variable */
    int32_t i32LastErr;
    int32_t i32Err;
    int32_t i32Int;
    int32_t i32Div;
    int32_t i32Comp;
} STR_PID_T;

int32_t PID_SetPoint(STR_PID_T *pPID, int32_t i32SetPoint);
void PID_SetGain(STR_PID_T *pPID, int32_t i32Kp, int32_t i32Ki, int32_t i32Kd);
int32_t PID_Add(int32_t i32InputA, int32_t i32InputB);
int32_t PID_Multiply(int32_t i32InputA, int32_t i32InputB);
int32_t PID_GetCompValue(STR_PID_T *pPID, int32_t i32Error);

#ifdef __cplusplus
}
#endif

#endif /*__PID_H__ */
