#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	


//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOB6复用为USART1-TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOB7复用为USART1-RX
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOB6与GPIOB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB6,PB7

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
#endif
	
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif	

void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

} 


void MPU_Matlab_report(float pitch,float roll,float yaw,u8 minute,u8 second,u8 ms_100,u8 ms_10)
{
//	u16 p=0,r=0,y=0;
	u8 tbuf[19]={0};
	u8 checksum=0;
	int i;
	
	tbuf[0]=0xAA;
	tbuf[1]=0xBB;
	tbuf[2]=0x00;
	tbuf[3]=0x00;
	tbuf[4]=0x0D;
		
	if(pitch > 0)
	{
		tbuf[5]=0x00;
		tbuf[6]=(unsigned char) pitch;
		tbuf[7]=(unsigned char) ((pitch-tbuf[6])*100);
		
	}
	else
	{
		tbuf[5]=0x01;
		pitch=-pitch;
		tbuf[6]=(unsigned char) pitch;
		tbuf[7]=(unsigned char) ((pitch-tbuf[6])*100);

	}
	
	
	if(roll > 0)
	{
		tbuf[8]=0x00;
		tbuf[9]=(unsigned char) roll;
		tbuf[10]=(unsigned char) ((roll-tbuf[9])*100);
		
	}
	else
	{
		tbuf[8]=0x01;
		roll=-roll;
		tbuf[9]=(unsigned char) roll;
		tbuf[10]=(unsigned char) ((roll-tbuf[9])*100);

	}

	
	if(yaw > 0)
	{
		tbuf[11]=0x00;
		tbuf[12]=(unsigned char) yaw;
		tbuf[13]=(unsigned char) ((yaw-tbuf[12])*100);
		
	}
	else
	{
		tbuf[11]=0x01;
		yaw=-yaw;
		tbuf[12]=(unsigned char) yaw;
		tbuf[13]=(unsigned char) ((yaw-tbuf[12])*100);

	}
	
	tbuf[14]=minute;
	tbuf[15]=second;
	tbuf[16]=ms_100;
	tbuf[17]=ms_10;	
	
	for(i=4;i<18;i++)
	{
		checksum ^= tbuf[i];
	}
	tbuf[18]=checksum;
	
	for(i=0;i<19;i++)
		usart1_send_char(tbuf[i]);
}



