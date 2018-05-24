#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"

//SDA    PB9
//SCL    PB8

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作函数	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //输入SDA 

//0表示写
#define	I2C_Transmitter   0
//１表示读
#define	I2C_Receiver      1	

#define true 1
#define false 0 
#define bool  uint8_t


#define TRUE  0
#define FALSE -1




//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


#endif
















