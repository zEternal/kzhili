/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,野火科技
 *     All rights reserved.
 *     技术讨论：野火初学论坛 http://www.chuxue123.com
 *
 *     除注明出处外，以下所有内容版权均属野火科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留野火科技的版权声明。
 *
 * @file       main.c
 * @brief      野火K60 平台主程序
 * @author     野火科技
 * @version    v5.0
 * @date       2013-12-19
 */

#include "common.h"
#include "include.h"


extern float Gyro_Now,g_fCarAngle;

extern float OutData[4];                              //SCI示波器参数
extern float Gyro_Now,angle_offset_vertical;          //陀螺仪转化后的角速度，转化后的加速度角度
extern float g_fCarAngle,g_fGyroscopeAngleIntegral;   //融合后的角度
extern volatile int     MMA7361 ,ENC03,real_angle;    //加速度计AD ,陀螺仪AD，模块输出的角度


extern void PIT0_IRQHandler(void);
extern   void OutPut_Data(void);                              //SCI采参数

void main()
{
  /* FTM_PWM_init(FTM0, FTM_CH0,10000,0);
    FTM_PWM_init(FTM0, FTM_CH1,10000,0);
    FTM_PWM_init(FTM0, FTM_CH2,10000,0);
    FTM_PWM_init(FTM0, FTM_CH3,10000,0);

    while(1){
    FTM_PWM_Duty(FTM0, FTM_CH0, 0);
    FTM_PWM_Duty(FTM0, FTM_CH1, 100);
    FTM_PWM_Duty(FTM0, FTM_CH2, 0);
    FTM_PWM_Duty(FTM0, FTM_CH3, 100);
    }
  */
  
    DisableInterrupts;//禁止总中断

    // 在control.h定义了相关宏
    adc_init (ZOUT);          //MMA7361 Z轴
    adc_init (Gyro1);        // ENC03角速度
    //adc_init (Ang);          //角度

    uart_init (UART3, 9600);
    //初始化 PWM 输出
    //FTM 的管脚 可在  fire_port_cfg.h
    //宏定义FTM0_PRECISON   改为  1000u
    //PWM数值反转。
    FTM_PWM_init(FTM0, FTM_CH0,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH1,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH2,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH3,10000,1000);


    //开启使能端
    gpio_init(PTD15,GPO,0);
    gpio_init(PTA19,GPO,0);
    gpio_init(PTA5 ,GPO,0);
    gpio_init(PTA24,GPO,0);

    /*
    //3 左 反转
    //5 左 正转
    //4 右 正转
    //6 右 反转
     */
    led_init(LED0);                                         //初始化LED0，PIT0中断用到LED0

    pit_init_ms(PIT0, 5);                                //初始化PIT0，定时时间为： 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    EnableInterrupts;//中断允许
    DELAY_MS(3000);
    while(1)
   {
        #if 1
        OutData[0] = MMA7361;/*ENC03*/
        OutData[1] = Gyro_Now;/*MMA7361*///Gyro_Now;  角速度归一化
        OutData[2] = angle_offset_vertical;   //加速度计采集到的角度归一化
        OutData[3] = g_fCarAngle;//融合后的角度
        OutPut_Data();
        #endif

   }
}


/**********************中断服务程序*******************/
void PIT0_IRQHandler(void)
{
    led_turn(LED0);                             //闪烁 LED0

    AD_Calculate();                             //AD
    Speed_Calculate(g_fCarAngle,Gyro_Now);      //速度计算
    PIT_Flag_Clear(PIT0);                       //清中断标志位
}

