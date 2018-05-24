#include "sys.h"
#include "TIM.h"

void timer_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	/************************************ͨ�ö�ʱ����ʼ��************************************/
	//��ʱ��ʱ��84M����Ƶϵ��16800������84M/16800=5Khz�ļ���Ƶ�ʣ�����50��Ϊ10ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM4ʱ��

	TIM_TimeBaseInitStructure.TIM_Period = 50-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=16800-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM4

	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE); //��ʼ����ʧ�ܶ�ʱ��4�����жϣ�ģʽѡ����ɺ�ʹ��

	TIM_Cmd(TIM4,ENABLE); 

}

