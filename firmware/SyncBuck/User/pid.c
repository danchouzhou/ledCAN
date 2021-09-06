#include <stdio.h>
#include "pid.h"

int32_t PID_SetPoint(STR_PID_T *pPID, int32_t i32SetPoint)
{
    pPID->i32SetPoint = i32SetPoint;
    return i32SetPoint;
}

void PID_SetGain(STR_PID_T *pPID, int32_t i32Kp, int32_t i32Ki, int32_t i32Kd)
{
    pPID->i32Kp = i32Kp;
    pPID->i32Ki = i32Ki;
    pPID->i32Kd = i32Kd;
}

int32_t PID_Add(int32_t i32InputA, int32_t i32InputB)
{
    int32_t i32Output;

    /* Prevent from overflow */
    if((int64_t)i32InputA + i32InputB > 2147483647)
        i32Output = 2147483647; // Set to maximum value
    else if((int64_t)i32InputA + i32InputB < -2147483648)
        i32Output = -2147483648; // Set to minimum value
    else
        i32Output = i32InputA + i32InputB;

    return i32Output;
}

int32_t PID_Multiply(int32_t i32InputA, int32_t i32InputB)
{
    int32_t i32Output;

    /* Prevent from overflow */
    if((int64_t)i32InputA * i32InputB > 2147483647)
        i32Output = 2147483647; // Set to maximum value 
    else if((int64_t)i32InputA * i32InputB < -2147483648)
        i32Output = -2147483648; // Set to minimum value
    else
        i32Output = i32InputA * i32InputB;

    return i32Output;
}

int32_t PID_GetCompValue(STR_PID_T *pPID, int32_t i32FeedbackValue)
{
    pPID->i32Err = PID_Add(pPID->i32SetPoint, -i32FeedbackValue);
    pPID->i32Int = PID_Add(pPID->i32Int, pPID->i32Err);
    pPID->i32Div = PID_Add(pPID->i32Err, -pPID->i32LastErr);

    pPID->i32LastErr = pPID->i32Err;

    pPID->i32Comp = PID_Add(
                    PID_Add(PID_Multiply(pPID->i32Err, pPID->i32Kp), PID_Multiply(pPID->i32Int, pPID->i32Ki)), 
                    PID_Multiply(pPID->i32Div, pPID->i32Kd)
                    );
    
    return pPID->i32Comp;
}
