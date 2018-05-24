#include "myiic.h"
#include "delay.h"
	  

//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	//GPIOB8,B9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	IIC_SCL=1;
	IIC_SDA=1;

}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    /*if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   */
    return receive;
}

bool i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
	
	IIC_Start();
	IIC_Send_Byte(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop();
		return false;//0
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(addr << 1 | I2C_Receiver);
    IIC_Wait_Ack();
	while (len) 
    {
        *buf = IIC_Read_Byte(1);
        if (len == 1)
            IIC_NAck();
        else
            IIC_Ack();
        buf++;
        len--;
    }
    IIC_Stop();
    return true;
}



bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	IIC_Start();
	IIC_Send_Byte(addr << 1|I2C_Transmitter);
	if(IIC_Wait_Ack())//接收应答成功返回0，不成功返回1
	{
		IIC_Stop();
		return false;//0
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	for(i = 0; i < len; i++)
	{
		IIC_Send_Byte(data[i]);
		if(IIC_Wait_Ack())//接收应答成功返回0，不成功返回1
		{
			IIC_Stop();
			return false;//0
		}
	}
	IIC_Stop();
	return true;//1
}



uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(i2cReadBuffer(addr,reg,len,buf))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

















