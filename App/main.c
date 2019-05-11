/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      Ұ��K60 ƽ̨������
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-12-19
 */

#include "common.h"
#include "include.h"


extern float Gyro_Now,g_fCarAngle;

extern float OutData[4];                              //SCIʾ��������
extern float Gyro_Now,angle_offset_vertical;          //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
extern float g_fCarAngle,g_fGyroscopeAngleIntegral;   //�ںϺ�ĽǶ�
extern volatile int     MMA7361 ,ENC03,real_angle;    //���ٶȼ�AD ,������AD��ģ������ĽǶ�


extern void PIT0_IRQHandler(void);
extern   void OutPut_Data(void);                              //SCI�ɲ���

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
  
    DisableInterrupts;//��ֹ���ж�

    // ��control.h��������غ�
    adc_init (ZOUT);          //MMA7361 Z��
    adc_init (Gyro1);        // ENC03���ٶ�
    //adc_init (Ang);          //�Ƕ�

    uart_init (UART3, 9600);
    //��ʼ�� PWM ���
    //FTM �Ĺܽ� ����  fire_port_cfg.h
    //�궨��FTM0_PRECISON   ��Ϊ  1000u
    //PWM��ֵ��ת��
    FTM_PWM_init(FTM0, FTM_CH0,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH1,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH2,10000,1000);
    FTM_PWM_init(FTM0, FTM_CH3,10000,1000);


    //����ʹ�ܶ�
    gpio_init(PTD15,GPO,0);
    gpio_init(PTA19,GPO,0);
    gpio_init(PTA5 ,GPO,0);
    gpio_init(PTA24,GPO,0);

    /*
    //3 �� ��ת
    //5 �� ��ת
    //4 �� ��ת
    //6 �� ��ת
     */
    led_init(LED0);                                         //��ʼ��LED0��PIT0�ж��õ�LED0

    pit_init_ms(PIT0, 5);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
    EnableInterrupts;//�ж�����
    DELAY_MS(3000);
    while(1)
   {
        #if 1
        OutData[0] = MMA7361;/*ENC03*/
        OutData[1] = Gyro_Now;/*MMA7361*///Gyro_Now;  ���ٶȹ�һ��
        OutData[2] = angle_offset_vertical;   //���ٶȼƲɼ����ĽǶȹ�һ��
        OutData[3] = g_fCarAngle;//�ںϺ�ĽǶ�
        OutPut_Data();
        #endif

   }
}


/**********************�жϷ������*******************/
void PIT0_IRQHandler(void)
{
    led_turn(LED0);                             //��˸ LED0

    AD_Calculate();                             //AD
    Speed_Calculate(g_fCarAngle,Gyro_Now);      //�ٶȼ���
    PIT_Flag_Clear(PIT0);                       //���жϱ�־λ
}

