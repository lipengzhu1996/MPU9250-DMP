#include "sys.h"
#include "TIM.h"

void timer_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	/************************************通用定时器初始化************************************/
	//定时器时钟84M，分频系数16800，所以84M/16800=5Khz的计数频率，计数50次为10ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟

	TIM_TimeBaseInitStructure.TIM_Period = 50-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=16800-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4

	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE); //初始化先失能定时器4更新中断，模式选择完成后使能

	TIM_Cmd(TIM4,ENABLE); 

}

