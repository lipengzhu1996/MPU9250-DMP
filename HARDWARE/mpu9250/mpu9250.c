#include "mpu9250.h"
#include "myiic.h"
#include "delay.h"
#include "usart.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"


/******************************************************************************
** 功能：   	 初始化MPU9250
** 参数：	 void
** 返回值:   0：成功    1：失败
** 说明： 
原始版本
Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	
Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
********************************************************************************/
MPU9250_STATUS Init_MPU9250()
{
	u8 id;
	Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80);	//复位MPU9250
	delay_ms(100);
	Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG, 0x00);	//唤醒MPU9250
	MPU9250_Set_Gyro_Fsr(3);						//设置陀螺仪满量程	±2000dps	
	MPU9250_Set_Accel_Fsr(2);						//设置加速度计满量程	±8g
	MPU9250_Set_Rate(100);							//设置采样率 其中会设置DLPF为采样率滤波带宽为采样率一半 50HZ
	Single_Write(MPU_ADDR,MPU_INT_EN_REG,0x00);		//关闭所有中断
	Single_Write(MPU_ADDR,MPU_USER_CTRL_REG,0x00);	//I2C主模式关闭
	Single_Write(MPU_ADDR,MPU_FIFO_EN_REG,0x00);	//关闭FIFO
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x80);	//INT引脚低电平有效
	id = Single_Read(MPU_ADDR,MPU_DEVICE_ID_REG);	//读取器件ID
	#ifdef MPU9250_DUBUG
		printf("MPU9250-ID = 0x%02x\n",id);
	#endif
	if(id == 0x71)
	{
		printf("MPU9250 Init Success...\n");
		Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01);	//设置CLKSEL,PLL X轴为参考
		Single_Write(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00);	//加速度与陀螺仪都工作
		MPU9250_Set_Rate(50);							//设置采样率为50Hz
		return MPU9250_OK;
	}
	else
	{
		printf("MPU9250 Init Fail...\n");
		return MPU9250_FAIL;
	}
}



/******************************************************************************
** 功能：   	 读取AKM8963器件ID
** 参数：	 u16 *id
** 返回值:   0：成功    1：失败
** 说明： 
1.要想读取AKM8963器件ID必须进入旁路模式
********************************************************************************/
MPU9250_STATUS READ_AKM8963_ID(u16 *id)
{
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);//进入旁路模式
	delay_ms(10);	
	Single_Write(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);//进入单次测量模式
	delay_ms(10);
	*id = Single_Read(MAG_ADDRESS,AKM8963_DEVICE_ID_REG);//0x48
	#ifdef MPU9250_DUBUG
		printf("AKM8963-ID = 0x%02x\n",*id);
	#endif	
	if(*id  == 0x48)
	{
		return  MPU9250_OK;
	}
	else
	{
		return  MPU9250_FAIL;
	}
}
	



/******************************************************************************
** 功能：   	单字节读取MPU9250寄存器
** 参数：	u8 SlaveAddress  从机地址
			u8 REG_Address   要读的寄存器地址 
** 返回值:  读出的数据
** 说明：
 110100x+读写位(读1写0)    0xD0-写   0xD1-读
1.发送起始信号
2.先发送从机地址+写   
3.在发送寄存器地址
4.在发送起始信号
5.发送从机地址+读
6.读取数据并回发NACK
7.发送停止信号
********************************************************************************/
u8 Single_Read(u8 SlaveAddress,u8 REG_Address)	 
{
  	u8 Data;//存储读出的数据
	IIC_Start();//起始信号
	IIC_Send_Byte(SlaveAddress);//发送从机地址+写信号 0xD0    110100x+读写位(读1写0)
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);//发送寄存器地址
	IIC_Wait_Ack();	
	IIC_Start();//起始信号
	//SlaveAddress << 1|1
	IIC_Send_Byte(SlaveAddress+1);//发送从机地址+读信号 0xD1    110100x+读写位(读1写0)
	IIC_Wait_Ack();//返回0  接收应答成功  1接收应答失败 
	Data = IIC_Read_Byte(0);//读取一个字节并回发NACK
	IIC_Stop();//发送停止信号
	return Data;//返回读取的数据
}




/******************************************************************************
** 功能：   	单字节读取MPU9250寄存器
** 参数：	u8 SlaveAddress  从机地址
			u8 REG_Address   要读的寄存器地址 
** 返回值:  读出的数据
** 说明：
 110100x+读写位(读1写0)    0xD0-写   0xD1-读
1.发送起始信号
2.先发送从机地址+写   
3.在发送寄存器地址
4.在发送起始信号
5.发送从机地址+读
6.读取数据并回发NACK
7.发送停止信号
********************************************************************************/
MPU9250_STATUS Multi_Read(u8 SlaveAddress,u8 REG_Address,u8 len, u8 *buf)
{
	IIC_Start();//起始信号
	IIC_Send_Byte(SlaveAddress);//发送从机地址+写信号 0xD0    110100x+读写位(读1写0)
	if(IIC_Wait_Ack())//返回0  接收应答成功  1接收应答失败
	{
		IIC_Stop();//发送停止信号
		return MPU9250_FAIL;//返回失败
	}
	IIC_Send_Byte(REG_Address);//发送寄存器地址
	IIC_Wait_Ack();	
	IIC_Start();//起始信号
	IIC_Send_Byte(SlaveAddress+1);//发送从机地址+读信号 0xD1    110100x+读写位(读1写0)
	IIC_Wait_Ack();
	while(len)
	{
		if(len == 1)//len=1表示只读取一个寄存器
		{
			*buf = IIC_Read_Byte(0);//读取一个字节并回发NACK
		}
		else
		{
			*buf = IIC_Read_Byte(1);//读取一个字节并回发ACK
		}
		len--;
		buf++;
	}
	IIC_Stop();//发送停止信号
	return MPU9250_OK;
}




/******************************************************************************
** 功能：   	单字节写MPU9250寄存器
** 参数：	u8 SlaveAddress  从机地址
			u8 REG_Address   要写入的寄存器地址 
			u8 REG_data		 要写入的寄存器数据
** 返回值:   0：成功    1：失败
** 说明：
110100x+读写位(读1写0)    0xD0-写   0xD1-读
1.先发送从机地址+写    
2.在发送寄存器地址
3.在发送要写入的数据
4.发送停止信号
********************************************************************************/
MPU9250_STATUS Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data)		 
{
	IIC_Start();//起始信号
	IIC_Send_Byte(SlaveAddress);//发送从机地址+写信号 0xD0    110100x+读写位(读1写0)
	if(IIC_Wait_Ack())//返回0  接收应答成功  1接收应答失败
	{
		IIC_Stop();//发送停止信号
		return MPU9250_FAIL;//返回失败
	}
	IIC_Send_Byte(REG_Address);//发送寄存器地址
	IIC_Wait_Ack();	
	IIC_Send_Byte(REG_data);//发送寄存器地址
	IIC_Wait_Ack();
	IIC_Stop();//发送停止信号
    return MPU9250_OK;
}




/******************************************************************************
** 功能：   	多字节写MPU9250寄存器
** 参数：	u8 SlaveAddress  从机地址
			u8 REG_Address   要写入的起始寄存器地址 
			u8 len			 要写入的数据的个数(要写入多少个连续的寄存器)
			u8 *buf		     写入寄存器的数据数组
** 返回值:   0：成功    1：失败
** 说明：
110100x+读写位(读1写0)    0xD0-写   0xD1-读
1.先发送从机地址+写    
2.在发送寄存器地址
3.连续发送要写入的数据
4.发送停止信号
********************************************************************************/
MPU9250_STATUS Multi_Write(u8 SlaveAddress,u8 REG_Address,u8 len,u8 *buf)
{
	u8 i;
	IIC_Start();//起始信号
	//SlaveAddress << 1 | 0
	IIC_Send_Byte(SlaveAddress);//发送从机地址+写信号 0xD0    110100x+读写位(读1写0)
	if(IIC_Wait_Ack())//返回0  接收应答成功  1接收应答失败
	{
		IIC_Stop();//发送停止信号
		return MPU9250_FAIL;//返回失败
	}
	IIC_Send_Byte(REG_Address);//发送寄存器地址
	IIC_Wait_Ack();	
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);//发送数据
		if(IIC_Wait_Ack())//返回0  接收应答成功  1接收应答失败
		{
			IIC_Stop();//发送停止信号
			return MPU9250_FAIL;//返回失败
		}
	}
	IIC_Stop();//发送停止信号
    return MPU9250_OK;
}




/******************************************************************************
** 功能：   	设置MPU9250采样率
** 参数：	u8 rate  采样率  单位HZ
** 返回值:   0：成功    1：失败
** 说明：
SAMPLE_RATE = 陀螺仪输出频率/(1+SMPLRT_DIV)==>SMPLRT_DIV=1KHZ/SAMPLE_RATE-1
DLPF=0/7   	陀螺仪输出频率=8KHZ
DLPF=1-6    陀螺仪输出频率=1KHZ
DLPF滤波频率通常设为采样频率的一半(MPU6050)
				Gyroscope
DLPF_CFG	带宽(HZ)	延时(ms)	采样率(KHZ)
	0		 250		 0.97			8
	1		 184		 2.9			1
	2		 92			 3.9			1
	3		 41		     5.9			1
	4		 20			 9.9			1
	5		 10			 17.85			1
	6		 5			 33.48			1
	7		 3600		 0.17   		8
********************************************************************************/
MPU9250_STATUS MPU9250_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data = 1000/rate-1;//data为SAMPLE_DIV       rate为采样频率 
	data = Single_Write(MPU_ADDR,MPU_SAMPLE_RATE_REG,data);//设置数字低通滤波器
	return MPU9250_Set_LPF(rate/2);//设置MPU9250-LPF为采样率一半
}



/******************************************************************************
** 功能：   	设置MPU9250陀螺仪数字低通滤波器
** 参数：	u16 数字低通滤波器
** 返回值:   0：成功    1：失败
** 说明：
				Gyroscope
DLPF_CFG	带宽(HZ)	延时(ms)	采样率(KHZ)
	0		 250		 0.97			8
	1		 184		 2.9			1
	2		 92			 3.9			1
	3		 41		     5.9			1
	4		 20			 9.9			1
	5		 10			 17.85			1
	6		 5			 33.48			1
	7		 3600		 0.17   		8
********************************************************************************/
MPU9250_STATUS MPU9250_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if(lpf >= 184)data=1;
	else if(lpf>=92)data=2;
	else if(lpf>=41)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return Single_Write(MPU_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}




/******************************************************************************
** 功能：   	设置MPU9250陀螺仪满量程范围
** 参数：	u8 fsr  0-3
** 返回值:   0：成功    1：失败
** 说明：
fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
********************************************************************************/
MPU9250_STATUS MPU9250_Set_Gyro_Fsr(u8 fsr)
{
	return Single_Write(MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}



/******************************************************************************
** 功能：   	设置MPU9250加速度计满量程范围
** 参数：	u8 fsr  0-3
** 返回值:   0：成功    1：失败
** 说明：
fsr:0,±2g;1,±4g;2,±8g;3,±16g
********************************************************************************/
MPU9250_STATUS MPU9250_Set_Accel_Fsr(u8 fsr)
{
	return Single_Write(MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度计满量程范围
}





/******************************************************************************
** 功能：   	读取MPU9250温度
** 参数：	short *temp  保存读取出来的温度值
** 返回值:   0：成功    1：失败
** 说明：	温度值扩大了100倍
********************************************************************************/
MPU9250_STATUS READ_MPU9250_TEMP(float *temp)
{
	u8 buf[2]; 
    short raw;
	Multi_Read(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    //*temp=(36.53+((double)raw)/340)*100;
	*temp = 21 + ((double)raw)/333.87;
	return MPU9250_OK;	
}




/******************************************************************************
** 功能：   	读取MPU9250陀螺仪
** 参数：	short *gx,short *gy,short *gz
** 返回值:   0：成功    1：失败
** 说明：
x,y,z轴原始读数(带符号)
********************************************************************************/
MPU9250_STATUS READ_MPU9250_GYRO(short *gx,short *gy,short *gz)
{
	u8 buf[6];
	if(Multi_Read(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;//返回失败
	}
	else
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;//返回成功
}




/******************************************************************************
** 功能：   	读取MPU9250加速度
** 参数：	short *ax,short *ay,short *az
** 返回值:   0：成功    1：失败
** 说明：
********************************************************************************/
MPU9250_STATUS READ_MPU9250_ACCEL(short *ax,short *ay,short *az)
{
	u8 buf[6];
	if(Multi_Read(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;//返回失败
	}
	else
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;//返回成功
}





/******************************************************************************
** 功能：   	读取MPU9250磁力计
** 参数：	short *ax,short *ay,short *az
** 返回值:   0：成功    1：失败
** 说明：
********************************************************************************/
MPU9250_STATUS READ_MPU9250_MAG(short *mx,short *my,short *mz)
{
	u8 buf[6];
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);//进入旁路模式
	delay_ms(10);	
	Single_Write(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);//进入单次测量模式
	delay_ms(10);
	if(Multi_Read(MAG_ADDRESS,AKM8963_MAG_XOUTL_REG,6,buf))
	{
		return MPU9250_FAIL;//返回失败
	}
	else
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	}
	return MPU9250_OK;//返回成功
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}



unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

 /*自检函数*/
void run_self_test(void)
{
    int result;

    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) 
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\n");
    }
	else
	{
		printf("bias has not been modified ......\n");
	}
    
}

