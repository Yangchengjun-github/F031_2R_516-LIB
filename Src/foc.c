#include "foc.h"

//==============================================================================


#include <math.h>




//==============================================================================


uint8_t rtt_buf1[512];
#pragma pack(push, 1)
struct
{
    float var1;
    float var2;
    float var3;
    float var4;
} varbuf;
#pragma pack(pop)
void foc_cal(void);
void get_current_offset(void);
//==============================================================================
void CCU80_IO_PWM_init(void);
void CCU80_IO_GPIO_init(void);

//==============================================================================

//==============================================================================
#define TWO_PI (2.0f * 3.1415926535f)
#define V_LIMIT (4.0f)
#define DELTA_T (0.001f)
//==============================================================================
uint16_t pwm_period = 1999;

uint16_t pwm_duty_a = 1000;
uint16_t pwm_duty_b = 1000;
uint16_t pwm_duty_c = 1000;

uint16_t pwmDuty[3] = {0};

uint16_t vadc_ia, vadc_ib, vadc_ic, vadc_vpot;

uint32_t adc_isr_flag, opa_cali_flag;
//==============================================================================
uint16_t ia_offset, ib_offset, ic_offset;
int16_t ia_mea, ib_mea, ic_mea;

float ia_mea_f, ib_mea_f, ic_mea_f;

float Ialpha, Ibeta;
float Id_mea, Iq_mea;

float Valpha = 0.0;
float Vbeta = 0.0;

float Vd = 0.0;
float Vq = 0.0;

float theta;
float theta_inc;
float theta_est;
//---------------------------------------------------------------------
float Id_ref = 0.0f;
float Id_err = 0.0f;
float Kp_d = 1.0f;
float Ki_d = 0.03f;
float sum_d = 0.0f;
//---------------------------------------------------------------------
float Ed = 0.0f;
float Eq = 0.0f;
float speed_est = 0.0f;
float R = 5.29f;
float L = 0.001058f;
float Kp_pll = 50.0f;
float Ki_pll = 5.0f;
float sum_pll = 0.0f;
float theta_err = 0.0f;
float theta_err_limit = 1.0471f;
//---------------------------------------------------------------------
float Va_abs, Vb3_abs;
float d1, d2, d3, d4, d5, d6, d7;
float da, db, dc;
float Vbat = 12.0f;
float K_svpwm = 1.5f;
float two_sqrt3 = 1.154700538f;
float one_sqrt3 = 0.5773502692f;
//---------------------------------------------------------------------
volatile uint16_t mode = 0;
uint16_t delay_cnt;
uint16_t ramp_cnt;
uint16_t ramp_val;
volatile uint16_t sw1_flag;
uint16_t chk_cnt;
uint16_t led_cnt;
uint16_t led_flag;

//==============================================================================
void d_axis_current_loop(void)
{
    sum_d += Ki_d * Id_err;
    if (sum_d > V_LIMIT)
    {
        sum_d = V_LIMIT;
    }
    else
    {
        if (sum_d < (-V_LIMIT))
        {
            sum_d = -V_LIMIT;
        }
    }

    Vd = sum_d + Kp_d * Id_err;

    if (Vd > V_LIMIT)
    {
        Vd = V_LIMIT;
    }
    else
    {
        if (Vd < (-V_LIMIT))
        {
            Vd = -V_LIMIT;
        }
    }
}
//==============================================================================
void position_estimate(void)
{
    Ed = Vd - R * Id_mea + speed_est * L * Iq_mea;
    Eq = Vq - R * Iq_mea - speed_est * L * Id_mea;

    theta_err = (-Ed / Eq);
    //-----------------------------------------------------
    if (theta_err_limit < theta_err)
    {
        theta_err = theta_err_limit;
    }
    else
    {
        if ((-theta_err_limit) > theta_err)
        {
            theta_err = -theta_err_limit;
        }
    }
    //-----------------------------------------------------
    sum_pll += Ki_pll * (theta_err);

    speed_est = sum_pll + Kp_pll * theta_err;

    theta_est = theta_est + speed_est * DELTA_T;

    if (theta_est > TWO_PI)
    {
        theta_est -= TWO_PI;
    }
    else if (theta_est < 0.0f)
    {
        theta_est += TWO_PI;
    }
    //-----------------------------------------------------
}
//==============================================================================
void svpwm(void)
{
    //-----------------------------------------------------------------
    Va_abs = fabs(Valpha);
    Vb3_abs = fabs(Vbeta * one_sqrt3);

    if ((Valpha >= 0.0f) && (Vbeta >= 0.0f) && (Va_abs >= Vb3_abs))
    { // sect 1
        d4 = K_svpwm * (Valpha - one_sqrt3 * Vbeta) / Vbat;
        d6 = K_svpwm * (two_sqrt3 * Vbeta) / Vbat;
        d7 = (1.0f - (d4 + d6)) * 0.5f;

        da = d4 + d6 + d7;
        db = d6 + d7;
        dc = d7;
    }

    if ((Vbeta > 0.0f) && (Va_abs <= one_sqrt3 * Vbeta))
    { // sect 2
        d6 = K_svpwm * (Valpha + one_sqrt3 * Vbeta) / Vbat;
        d2 = K_svpwm * (-Valpha + one_sqrt3 * Vbeta) / Vbat;
        d7 = (1.0f - (d6 + d2)) * 0.5f;

        da = d6 + d7;
        db = d6 + d2 + d7;
        dc = d7;
    }

    if ((Valpha <= 0.0f) && (Vbeta >= 0.0f) && (Va_abs >= Vb3_abs))
    { // sect 3
        d2 = K_svpwm * two_sqrt3 * Vbeta / Vbat;
        d3 = K_svpwm * (-Valpha - one_sqrt3 * Vbeta) / Vbat;
        d7 = (1.0f - (d2 + d3)) * 0.5f;

        da = d7;
        db = d2 + d3 + d7;
        dc = d3 + d7;
    }

    if ((Valpha <= 0.0f) && (Vbeta <= 0.0f) && (Va_abs >= Vb3_abs))
    { // sect 4
        d1 = -K_svpwm * two_sqrt3 * Vbeta / Vbat;
        d3 = K_svpwm * (-Valpha + one_sqrt3 * Vbeta) / Vbat;
        d7 = (1.0f - (d3 + d1)) * 0.5f;

        da = d7;
        db = d3 + d7;
        dc = d3 + d1 + d7;
    }

    if ((Va_abs <= -one_sqrt3 * Vbeta))
    { // sect 5
        d1 = K_svpwm * (-Valpha - one_sqrt3 * Vbeta) / Vbat;
        d5 = K_svpwm * (Valpha - one_sqrt3 * Vbeta) / Vbat;
        d7 = (1.0f - (d1 + d5)) * 0.5f;

        da = d5 + d7;
        db = d7;
        dc = d1 + d5 + d7;
    }

    if ((Valpha >= 0.0f) && (Vbeta <= 0.0f) && (Va_abs >= Vb3_abs))
    { // sect 6
        d4 = K_svpwm * (Valpha + one_sqrt3 * Vbeta) / Vbat;
        d5 = -K_svpwm * two_sqrt3 * Vbeta / Vbat;
        d7 = (1.0f - (d4 + d5)) * 0.5f;

        da = d4 + d5 + d7;
        db = d7;
        dc = d5 + d7;
    }

    //-----------------------------------------------------------------
    /* 配置通道1 */
    PWMConfig.Pulse = 36000 - (uint16_t)(36000 * da); /* CC1值为10，占空比=10/50=20% */
    pwmDuty[0] = PWMConfig.Pulse;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    /* 配置通道2 */
    PWMConfig.Pulse = 36000 - (uint16_t)(36000 * db);
    pwmDuty[1] = PWMConfig.Pulse;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    /* 配置通道3 */
    PWMConfig.Pulse = 36000 - (uint16_t)(36000 * dc);
    pwmDuty[2] = PWMConfig.Pulse;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_3) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    // TIM1->CCMR1 = 2000 - (uint16_t)(2000 * da);
    // TIM1->CCMR2 = 2000 - (uint16_t)(2000 * db);
    // TIM1->CCMR3 = 2000 - (uint16_t)(2000 * dc);

    // //-----------------------------------------------------------------
    // CCU80->GCSS |= 0X00000111;

#if 1
    /* 开启通道1输出 */
    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    /* 开启通道2输出 */
    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    /* 开启通道3输出 */
    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    /* 开启通道1输出 */
    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    /* 开启通道2输出 */
    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    /* 开启通道3输出 */
    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
    {
        APP_ErrorHandler();
    }
#endif

    /* 开启通道4输出 */
    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    //-----------------------------------------------------------------
}
//==============================================================================


//******************************************************************************
void foc_cal(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    //---------------------------------------------------------
    adc_isr_flag = 1;

    //---------------------------------------------------------
    vadc_ia = gADCxConvertedData[CHANNEL_IU];
    vadc_ib = gADCxConvertedData[CHANNEL_IV];
    //vadc_ic = gADCxConvertedData[];

    

    if (opa_cali_flag == 1)
    {
        //-----------------------------------------------------
        // current measure
        //-----------------------------------------------------
        ia_mea = (int16_t)(ia_offset - vadc_ia);
        ib_mea = (int16_t)(ib_offset - vadc_ib);
        //ic_mea = (int16_t)(ic_offset - vadc_ic);
        ic_mea = (int16_t)(0 - ia_mea - ib_mea);

        ia_mea_f = (float)ia_mea * 0.0038147f;
        ib_mea_f = (float)ib_mea * 0.0038147f;
        ic_mea_f = (float)ic_mea * 0.0038147f;

        //------------------------------------------------                                                                                                                                                                                                                                                                                                                                                                                               -----
        // clarke transform
        //-----------------------------------------------------
        // Ialpha = (2/3) * (Ia - Ib/2 - Ic/2)
        Ialpha = 0.6666666667f * (ia_mea_f - 0.5f * (ib_mea_f + ic_mea_f));

        // Ibeta = (1/sqrt(3)) * (Ib - Ic)
        Ibeta = 0.5773502692f * (ib_mea_f - ic_mea_f);

        //-----------------------------------------------------
        // Park transform
        //-----------------------------------------------------
        Id_mea = cosf(theta_est) * Ialpha + sinf(theta_est) * Ibeta;
        Iq_mea = -sinf(theta_est) * Ialpha + cosf(theta_est) * Ibeta;

        //-----------------------------------------------------
        // calc position
        //-----------------------------------------------------
       position_estimate();
      
#if 1
        //-----------------------------------------------------
        switch (mode)
        {
        //-------------------------------------------------
        // check user1 switch
        //-------------------------------------------------
        case 0:
        {
            //---------------------------------------------
            // if (++led_cnt == 1000)
            // {
            //     led_cnt = 0;

            //     if (led_flag)
            //     {
            //         led_flag = 0;
                   
            //     }
            //     else
            //     {
            //         led_flag = 1;
                   
            //     }
            // }
            //---------------------------------------------
   

                Id_ref = 0.0f;
                delay_cnt = 0;
                sum_d = 0.0f;
                Vd = 0.0f;
                Vq = 0.0f;
                Id_ref = 0.0f;
                sum_pll = 0.0f;
                speed_est = 0.0f;

                CCU80_IO_PWM_init();

               
               
                mode = 1;

            //---------------------------------------------
        }
        break;

        //-------------------------------------------------
        // align
        //-------------------------------------------------
        case 1:
        {
            theta = 0;
            Id_ref += 0.0002f; // 利用d轴电流对齐
            Vq = 0;

            if (++delay_cnt >= 2000) // lock time = 2s
            {
                delay_cnt = 0;
                theta_inc = 0.0f;

                mode = 2;
            }
        }
        break;

        //-------------------------------------------------
        // ramp up
        //-------------------------------------------------
        case 2:
        {
            if (++delay_cnt >= 50)
            {
                delay_cnt = 0;

                theta_inc += 0.0005f;
                if (theta_inc >= 0.3f)
                {
                    theta_inc = 0.3f;  
                    chk_cnt = 0;
                    ramp_cnt = 0;
                    ramp_val = 0;
                    Eq = 0.0f;

                   
                   
                    mode = 3;
                }
            }

            theta += theta_inc;
            if (theta >= TWO_PI)
            {
                theta = 0.0f;
            }
        }
        break;

        //-------------------------------------------------
        // sensorless running
        //-------------------------------------------------
        case 3:
        {
            //---------------------------------------------
            Id_ref = 0.0f;

            if (++ramp_cnt == 5)
            {
                ramp_cnt = 0;

                if (vadc_vpot > ramp_val)
                {
                    ramp_val++;
                }
                else if (vadc_vpot < ramp_val)
                {
                    ramp_val--;
                }
            }

            vadc_vpot = 0;

            Vq = (10.0f + ramp_val * 3700.0f / 4095.0f) * 6.92820323f / 4096.0f;
            
            //---------------------------------------------
            theta = theta_est;

            //---------------------------------------------
            if (++chk_cnt >= 3000)
            {
                if (Eq < 0.003f)
                {
                    mode = 0;
                    sw1_flag = 0;
                    sum_d = 0.0f;
                    Vd = 0.0f;
                    Vq = 0.0f;
                    Id_ref = 0.0f;
                    sum_pll = 0.0f;
                    speed_est = 0.0f;

                   
                   
                    CCU80_IO_GPIO_init();
                }
            }
            //---------------------------------------------
            if (sw1_flag)
            {
                sw1_flag = 0;

                sum_d = 0.0f;
                Vd = 0.0f;
                Vq = 0.0f;
                Id_ref = 0.0f;
                sum_pll = 0.0f;
                speed_est = 0.0f;

               
                CCU80_IO_GPIO_init();

                mode = 0;
            }
        }
        break;
        //-------------------------------------------------
        default:
        {
            mode = 0;
            sw1_flag = 0;
            sum_d = 0.0f;
            sum_pll = 0.0f;
            speed_est = 0.0f;

       
            CCU80_IO_GPIO_init();
        }

            //-------------------------------------------------
        }
#endif
        //-----------------------------------------------------
        // current control
        //-----------------------------------------------------
        
         Id_err = Id_ref - Id_mea;
         d_axis_current_loop();

        //-----------------------------------------------------
        // inverse Park transform
        //-----------------------------------------------------
        Valpha = cosf(theta) * Vd - sinf(theta) * Vq;
        Vbeta = sinf(theta) * Vd + cosf(theta) * Vq;

        //-----------------------------------------------------
        // svpwm modulation
        //-----------------------------------------------------
   
        svpwm();

        //-----------------------------------------------------
    }

    //=========================================================
    // JSCOPE debugger
    // varbuf.var1 = ia_mea;
    // varbuf.var2 = ib_mea;
    // varbuf.var3 = ic_mea;
    // varbuf.var4 = Id_mea;

    //=========================================================
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}
void log_out(void)
{
    // printf("%f,%f,%f\n", Valpha,Vbeta,theta);
      printf("%d,%d,%d\n",pwmDuty[0],pwmDuty[1],pwmDuty[2]);
}

void get_current_offset(void)
{
    uint16_t sum_a, sum_b, sum_c;
    uint16_t cnt;

    adc_isr_flag = 0;
    opa_cali_flag = 0;

    sum_a = 0;
    sum_b = 0;

    for (cnt = 0; cnt < 8; cnt++)
    {
        adc_isr_flag = 0;
        while (!adc_isr_flag)
            ;
        

        sum_a += vadc_ia;
        sum_b += vadc_ib;


        printf("cnt:%d,a:%d,b:%d\n",cnt,vadc_ia,vadc_ib);
        //sum_c += XMC_VADC_GROUP_GetResult(VADC_G0, 4);
    }

    ia_offset = sum_a >> 3;
    ib_offset = sum_b >> 3;
    //ic_offset = sum_c >> 3;

    opa_cali_flag = 1;

   // delay_ms(10);
}


void CCU80_IO_PWM_init(void)
{
    // XMC_GPIO_Init(P0_5, &CCU8_strong_sharp_config);
    // XMC_GPIO_Init(P0_4, &CCU8_strong_sharp_config);
    // XMC_GPIO_Init(P0_3, &CCU8_strong_sharp_config);
    // XMC_GPIO_Init(P0_2, &CCU8_strong_sharp_config);
    // XMC_GPIO_Init(P0_1, &CCU8_strong_soft_config);
    // XMC_GPIO_Init(P0_0, &CCU8_strong_soft_config);
}
//==============================================================================
void CCU80_IO_GPIO_init(void)
{
    // XMC_GPIO_Init(P0_5, &CCU8_gpio_strong_sharp_config);
    // XMC_GPIO_Init(P0_4, &CCU8_gpio_strong_sharp_config);
    // XMC_GPIO_Init(P0_3, &CCU8_gpio_strong_sharp_config);
    // XMC_GPIO_Init(P0_2, &CCU8_gpio_strong_sharp_config);
    // XMC_GPIO_Init(P0_1, &CCU8_gpio_strong_soft_config);
    // XMC_GPIO_Init(P0_0, &CCU8_gpio_strong_soft_config);
}
//==============================================================================
void delay_100_us(void)
{
    uint16_t m;
    for (m = 0; m < 1600; m++)
        ;
}
//==============================================================================
void delay_ms(uint16_t m)
{
    uint16_t p, q;
    for (p = 0; p < m; p++)
    {
        for (q = 0; q < 10; q++)
        {
            delay_100_us();
        }
    }
}
//==============================================================================

//==============================================================================

//==============================================================================
