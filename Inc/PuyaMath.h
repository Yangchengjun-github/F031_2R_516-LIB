/**
  ******************************************************************************
  * @file    PuyaMath.c
  * @author  MCU Application Team
  * @Version V1.0.0
  * @Date    2022-08-01
  * @brief   BLDC fFOC Sensorless under 2R Sensing Current
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PuyaMath_H
#define __PuyaMath_H

#ifdef __cplusplus
extern "C" {
#endif

/** 
  * @brief PuyaMath parameters  
  */
    
#define   DEADTIME_CNT  10
#define   SAMP_DELAY    (DEADTIME_CNT+20)
#define   SAMP_WINDOW   DEADTIME_CNT
    
extern const signed int SinTab_1024[];
extern int16_t ta1,tb1,tc1,ta2,tb2,tc2;
extern uint16_t t_min,TIMR1_CCR4;    
void voTabSinCos(SIN_COS *in);
void angle_cla(void);
	
int16_t pySqrt_31000(int16_t Vsd);//Calc Sqrt(31000*31000-Vsd*Vsd)

#endif
/*************************************************************************
 End of this File (EOF):
 !!!!!!Do not put anything after this part!!!!!!!!!!!
*************************************************************************/
