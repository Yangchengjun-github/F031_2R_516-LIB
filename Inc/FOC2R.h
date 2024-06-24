/**
  ******************************************************************************
  * @file    FOC2R.c
  * @author  MCU Application Team
  * @Version V1.0.0
  * @Date    2023-03-01
  * @brief   BLDC FOC Sensorless under 1R Sensing Current
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FOC2R_H
#define __FOC2R_H

#ifdef __cplusplus
extern "C" {
#endif

#define VDC_GAIN_COF			1354	
#define VDC_OFSET				3	
	
#define MOT_DIR_CWC	0
#define MOT_DIR_CCW	1

#define MOT_RUN_STOPING			0x00
#define MOT_RUN_CWC_DIR			0x11
#define MOT_RUN_CCW_DIR			0x22
#define MOT_RUN_CHANGE_DIR	    0x33
	
#define MCU_MAIN_FREQUANCY	72000000	//Hz
#define MOTOR_PWM_FREQUANCY	2000//20000	//Hz
#define MOTOR_PWM_TIM1_DUTY	(MCU_MAIN_FREQUANCY/MOTOR_PWM_FREQUANCY)
#define MOTOR_PWM_TIM1_ISR_TH	(MOTOR_PWM_TIM1_DUTY-100)
#define MOTOR_PWM_BASE_DUTY	(MCU_MAIN_FREQUANCY/MOTOR_PWM_FREQUANCY/4)	
//PA08--PWMU+; PA09--PWMV+; PA10--PWMW+; 
//PA07--PWMU-; PB00--PWMV-; PB01--PWMW-;

//PA08--PWMV+; PA09--PWMU+; PA10--PWMW+; 
//PA07--PWMV-; PB00--PWMU-; PB01--PWMW-;

#define abs(x) ( x < 0 ? ( -x ) : ( x ) )

#define PWM_IS_ONN		1
#define PWM_IS_OFF		0
	
#define MAX_IS	32000//25000//1900//2218//250
#define MOTOR_STALL_MAXCNT	65000
#define MAX_CUR_THRESHOLD		MAX_IS
#define MAX_VDC_THRESHOLD		2700//2400//DC 22V
#define MIN_VDC_THRESHOLD		600//DC 6.5V
#define MIN_NTC_THRESHOLD		1229//1.5V-1229, 2.6V-2048, 3V-2457 @ NTC=15K & R1=10K
	
#define MOTOR_NO_ERROR		0x0000
#define MOTOR_SOC_ERROR		0x0001	
#define MOTOR_HOC_ERROR		0x0002
#define MOTOR_OV_ERROR		0x0004
#define MOTOR_LV_ERROR		0x0008
#define MOTOR_OH_ERROR		0x0010	
#define MOTOR_STALL_ERROR	0x0020
#define MOTOR_OVSPD_ERROR	0x0040
#define MOTOR_ELSE_ERROR	0x0080	



/** @defgroup Exported_types  Exported_types
* @{
*/
#define STATE_POWERON           	0   // STATE_POWERON
#define STATE_STOP_NOMARL           1   // Stopped without error
#define STATE_STOP_ERROR            10   // Stopped with error
#define STATE_RUN_SPD       		1001   //  
#define STATE_RUN_StartB       		4000   // STATE_RUN_StartB
#define STATE_RUN_StartA       		5000   // STATE_RUN_StartA
#define STATE_RUN_FreeSpd       	8000   // STATE_RUN_FreeSpd
#define STATE_RUN_VF				7000
#define STATE_RUN_DC								2
#define STATE_RUN_IF								3
#define STATE_RUN_TQC								4
/** 
  * @brief FOC2R parameters  
  */
typedef struct{       //
    UWORD uwSitaPu;   //00    Q15
    SWORD swSinPu;    //02    Q15
    SWORD swCosPu;    //04    Q15
} SIN_COS;	



typedef struct 	{ 
	  UWORD	SinCosSita;
	  int16_t	SinAdc;
	  int16_t	CosAdc;
    int16_t	AdcIb;
    int16_t	AdcIc;
    int16_t	Ialfa;
    int16_t	Ibeta;
    int16_t	Id;
    int16_t	Iq;
    int16_t	wSin;
    int16_t	wCos;	
    int16_t Vd;
    int16_t Vq;	
    int16_t Valfa;
    int16_t Vbeta;	

    int16_t MotDirCmd;
    int16_t swIalfaLpf;
    int16_t swIbetaLpf;
    SLONG_UNION slIdLpf;
    SLONG_UNION slIqLpf;
    
    int32_t  Ta;				
    int32_t  Tb;				
    int32_t  Tc;				
    int32_t  tmp1;			  
    int32_t  tmp2;			
    int32_t  tmp3;			
    uint16_t VecSector;	
    uint8_t pwmflag; 
    uint16_t LpfCof;
    
    int16_t VdcAdc;
    uint16_t Cycle_cnt;
    uint16_t PwmIsr_cnt;	
    uint16_t SysStateId;		
    uint16_t MotorErrCode;
    uint32_t SpdLopCnt;
    uint16_t WeCmd;
		uint16_t Sita0;
    uint16_t We;
    uint16_t WeLPF;
    uint16_t VecSector0;
    uint16_t VecSector1;
}PUYA_SEMI_MCBase_TYPE ;

typedef struct
{
	int16_t swOut;
	
	int16_t Ref;
	int16_t Yk;
	
	int16_t ek;
	int16_t ek_1;	
	int16_t dek;
	
	int16_t Kp;
	int16_t Ki;	
	int16_t Max;
	int16_t Min;
	
	int16_t Cnt;
	SLONG_UNION slOut;
}PUYA_SEMI_MC_FOC2R_PI_TYPE;


typedef struct {
	     int32_t  Valpha;       	
	     int32_t  Ealpha;     
	     int32_t  Zalpha;       
	     int32_t  GPRA;      
	     int32_t  EstIalpha;   
	     int32_t  FPRA;      
	     int32_t  Vbeta;        
	     int32_t  Ebeta;  	   
	     int32_t  Zbeta;       
	     int32_t  EstIbeta;     
	     int32_t  Ialpha;  	    
	     int32_t  IalphaError;  
	     int32_t  Kslide;       
	     int32_t  Ibeta;  	  
	     int32_t  IbetaError;  
	     int32_t  Kslf;         //�˲���ϵ��
	     int32_t  E0;	          //��Ĥ�ĵ��������޷�ֵ 0.5
	     uint16_t  IQAngle;
	     int32_t  IQTan;
	     int16_t  evalue;
	     uint16_t  PIQAngle;
		 
	  }P_PYFOC_SMO;







typedef struct{
	  float  Rs; 			
	  float  Ls;			 
          float VIpram; 			
	  				 
         float  Ts;			 	 
         uint32_t   POLES; 
	  float  FPRA;		  
         float  GPRA;			 
  }PMSM_Motor;
typedef struct
{
	int16_t 	IaOffset;
	int32_t   IaOffsetS;
	int16_t 	IbOffset;
	int32_t   IbOffsetS;
	int16_t   OffsetCount;
	int16_t   OffsetTimes;
	int16_t   OffsetFlag;
}focOffset;

typedef struct
{
	int8_t SmoID;

	int16_t swIsVsLpfCofPu;

	int16_t swLpfCofPu;

	int16_t MotorTypeID;
	int16_t swRs[50];
	int16_t swLsDivTc[50];
	int16_t InitSitaCWC[50];
	int16_t InitSitaCCW[50];
	int16_t InitIqRef[50];
	int16_t MaxIqRef[50];

	int16_t swRsPu;
	int16_t swLsDivTcPu;
	int16_t IalfaLPF0;
	int16_t IbetaLPF0;

	int16_t Valfa;
	int16_t Vbeta;
	int16_t Ialfa;
	int16_t Ibeta;

	SLONG_UNION ValfaLPF;
	SLONG_UNION VbetaLPF;
	SLONG_UNION IalfaLPF;
	SLONG_UNION IbetaLPF;

	int16_t swEdCalc;
	int16_t EdError;
	int16_t Ealfa;
	int16_t Ebeta;
	int16_t EalfaLPF;
	int16_t EbetaLPF;
	SLONG_UNION slEalfa;
	SLONG_UNION slEbeta;

	int16_t EdRough;
	int16_t Ed;
	SLONG_UNION slEdLpf;
	int16_t EdError0;
	int16_t SinPLL;
	int16_t CosPLL;
	int16_t SinFOC;
	int16_t CosFOC;
	int16_t swDetaSitaPu;
	int16_t SitaErr;
	int16_t swTc2PiPu;
	int16_t swWeMaxPu;
	int16_t swKpPu;
	int16_t swKitPu;
	int16_t swCurTh;
	int16_t We;
	SLONG_UNION slWePu;
	int16_t WeLPF;
	int16_t WeID;
	SIN_COS SinCosPLL;
	SIN_COS SinCosFOC;
	SLONG_UNION slPLLSitaPu;
} PUYA_SEMI_MC_PLL_Base_TYPE; /*!< PLL Data Structure */

/**
  * @} 
  */
extern SIN_COS SinCos;
extern PUYA_SEMI_MCBase_TYPE   foc;
extern PUYA_SEMI_MC_PLL_Base_TYPE mc_PLL;

void PUYA_SEMI_MC_PLL_Init(PUYA_SEMI_MC_PLL_Base_TYPE *PLL);
void PUYA_SEMI_MC_PLL_Calc(PUYA_SEMI_MC_PLL_Base_TYPE *PLL);

void voTabSinCos(SIN_COS *in);
void PUYA_SEMI_MC_FOC2R_PI_Ctrl(PUYA_SEMI_MC_FOC2R_PI_TYPE *stPI);
void PUYA_SEMI_MC_FOC2R_SPDPI_Ctrl(PUYA_SEMI_MC_FOC2R_PI_TYPE *stPI);
void PUYA_SEMI_MC_FOC2R_INIT(void);
void PUYA_SEMI_MC_6StepAdc_PWM_int(void);
void PUYA_SEMI_MC_6StepAdc_ADC_Init(void);
void PUYA_SEMI_MC_FOC2R_Schedule(void);
void PUYA_SEMI_MC_FOC2R_PWMCtrl(void);
void TIM_Channel_Cmd(TIM_TypeDef *htim, uint32_t Channel, uint32_t ChannelState);
void PUYA_SEMI_MC_6StepAdc_CommunicatingPhase(void);
void InnerComp_Init(void); 

void USART_Config(void);
void Error_Handler(void);
void SystemClock_Config(void);
void LED_Handler(uint16_t LED_Err);

void APP_DMAConfig(void);

void APP_ActivateADC(void);
void APP_ConfigureADC(void);

void APP_DmaConfig(void);
void APP_AdcConfig(void);
void APP_AdcEnable(void);
void APP_AdcCalibrate(void);
void APP_ConfigTIM1Base(void);
void APP_ConfigPWMChannel(void);
void APP_ConfigBDTR(void);
void PUYA_SEMI_MC_FocPWM_ON(void);
void PUYA_SEMI_MC_FocPWM_OFF(void);

extern PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidId;
extern PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidIq;
extern PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidSpd;

extern SIN_COS FocSinCos;
#endif
/*************************************************************************
 End of this File (EOF):
 !!!!!!Do not put anything after this part!!!!!!!!!!!
*************************************************************************/
