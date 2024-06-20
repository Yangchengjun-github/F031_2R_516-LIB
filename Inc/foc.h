#ifndef _FOC_H
#define _FOC_H

#include "main.h"

void foc_cal(void);
void get_current_offset(void);
void log_out(void);

extern int16_t ia_mea, ib_mea, ic_mea;
extern float Ialpha, Ibeta;
extern float Id_mea, Iq_mea;
extern uint16_t vadc_ia, vadc_ib, vadc_ic, vadc_vpot;
extern uint16_t pwmDuty[3] ;
#endif

