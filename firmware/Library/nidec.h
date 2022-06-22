#ifndef _NIDEC_H_
#define _NIDEC_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32_t u32gpioMFPsave;
    uint8_t u8duty;
    uint8_t u8direction;
    uint32_t u32isAttached;
} STR_NIDEC_T;

uint32_t nidec_attach(STR_NIDEC_T *pNidec);
uint8_t nidec_write(STR_NIDEC_T *pNidec, uint8_t u8duty, uint8_t u8direction);
uint8_t nidec_read(STR_NIDEC_T *pNidec);
void nidec_detach(STR_NIDEC_T *pNidec);

#ifdef __cplusplus
}
#endif

#endif
