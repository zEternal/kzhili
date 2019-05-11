#ifndef _CONTROL_H
#define _CONTROL_H

#define XOUT    ADC1_DM0
#define YOUT    ADC0_SE16
#define ZOUT    ADC1_SE8//ADC0_SE17

#define Gyro1   ADC1_SE9//ADC1_SE16
#define Gyro2   ADC1_DP0
#define Ang     ADC0_SE18

/***********************ֱ�����Ʋ���********************/
#define MMA7361_vertical           2921 /*1780*/    // 1860// 1760  //1850// 2600
#define GYRO_VAL                     3355/*2430*/   //��������ֵ        //�Ӵ���󣬼�����ǰ
#define Gyro_ratio                  0.4/*0.8*/    //0.8
#define GRAVITY_ADJUST_TIME_CONSTANT 2  //2
#define DT                           0.005
#define MMA7361_ratio                0.063/*0.12*///1150    
#define P_ANGLE             35
#define D_ANGLE             1
 //������������5���������Լ���е�ṹ
 
#define MOTOR_DEAD_VAL_L  15    //  ������ѹ
#define MOTOR_DEAD_VAL_R  15





/***********************��������********************/
 extern   void Rd_Ad_Value(void);                              //AD�ɼ�
 extern   void AD_Calculate(void);                              //AD�ɼ��ͼ���
 extern   void Speed_Calculate(float angle,float angle_dot);   //�ٶȼ���
 static   unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
 extern   void OutPut_Data(void);                              //SCI�ɲ���

#endif