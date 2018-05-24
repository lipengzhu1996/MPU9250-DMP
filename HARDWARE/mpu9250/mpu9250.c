#include "mpu9250.h"
#include "myiic.h"
#include "delay.h"
#include "usart.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"


/******************************************************************************
** ���ܣ�   	 ��ʼ��MPU9250
** ������	 void
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵���� 
ԭʼ�汾
Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	
Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
********************************************************************************/
MPU9250_STATUS Init_MPU9250()
{
	u8 id;
	Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80);	//��λMPU9250
	delay_ms(100);
	Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG, 0x00);	//����MPU9250
	MPU9250_Set_Gyro_Fsr(3);						//����������������	��2000dps	
	MPU9250_Set_Accel_Fsr(2);						//���ü��ٶȼ�������	��8g
	MPU9250_Set_Rate(100);							//���ò����� ���л�����DLPFΪ�������˲�����Ϊ������һ�� 50HZ
	Single_Write(MPU_ADDR,MPU_INT_EN_REG,0x00);		//�ر������ж�
	Single_Write(MPU_ADDR,MPU_USER_CTRL_REG,0x00);	//I2C��ģʽ�ر�
	Single_Write(MPU_ADDR,MPU_FIFO_EN_REG,0x00);	//�ر�FIFO
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x80);	//INT���ŵ͵�ƽ��Ч
	id = Single_Read(MPU_ADDR,MPU_DEVICE_ID_REG);	//��ȡ����ID
	#ifdef MPU9250_DUBUG
		printf("MPU9250-ID = 0x%02x\n",id);
	#endif
	if(id == 0x71)
	{
		printf("MPU9250 Init Success...\n");
		Single_Write(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01);	//����CLKSEL,PLL X��Ϊ�ο�
		Single_Write(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00);	//���ٶ��������Ƕ�����
		MPU9250_Set_Rate(50);							//���ò�����Ϊ50Hz
		return MPU9250_OK;
	}
	else
	{
		printf("MPU9250 Init Fail...\n");
		return MPU9250_FAIL;
	}
}



/******************************************************************************
** ���ܣ�   	 ��ȡAKM8963����ID
** ������	 u16 *id
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵���� 
1.Ҫ���ȡAKM8963����ID���������·ģʽ
********************************************************************************/
MPU9250_STATUS READ_AKM8963_ID(u16 *id)
{
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);//������·ģʽ
	delay_ms(10);	
	Single_Write(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);//���뵥�β���ģʽ
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
** ���ܣ�   	���ֽڶ�ȡMPU9250�Ĵ���
** ������	u8 SlaveAddress  �ӻ���ַ
			u8 REG_Address   Ҫ���ļĴ�����ַ 
** ����ֵ:  ����������
** ˵����
 110100x+��дλ(��1д0)    0xD0-д   0xD1-��
1.������ʼ�ź�
2.�ȷ��ʹӻ���ַ+д   
3.�ڷ��ͼĴ�����ַ
4.�ڷ�����ʼ�ź�
5.���ʹӻ���ַ+��
6.��ȡ���ݲ��ط�NACK
7.����ֹͣ�ź�
********************************************************************************/
u8 Single_Read(u8 SlaveAddress,u8 REG_Address)	 
{
  	u8 Data;//�洢����������
	IIC_Start();//��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);//���ʹӻ���ַ+д�ź� 0xD0    110100x+��дλ(��1д0)
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);//���ͼĴ�����ַ
	IIC_Wait_Ack();	
	IIC_Start();//��ʼ�ź�
	//SlaveAddress << 1|1
	IIC_Send_Byte(SlaveAddress+1);//���ʹӻ���ַ+���ź� 0xD1    110100x+��дλ(��1д0)
	IIC_Wait_Ack();//����0  ����Ӧ��ɹ�  1����Ӧ��ʧ�� 
	Data = IIC_Read_Byte(0);//��ȡһ���ֽڲ��ط�NACK
	IIC_Stop();//����ֹͣ�ź�
	return Data;//���ض�ȡ������
}




/******************************************************************************
** ���ܣ�   	���ֽڶ�ȡMPU9250�Ĵ���
** ������	u8 SlaveAddress  �ӻ���ַ
			u8 REG_Address   Ҫ���ļĴ�����ַ 
** ����ֵ:  ����������
** ˵����
 110100x+��дλ(��1д0)    0xD0-д   0xD1-��
1.������ʼ�ź�
2.�ȷ��ʹӻ���ַ+д   
3.�ڷ��ͼĴ�����ַ
4.�ڷ�����ʼ�ź�
5.���ʹӻ���ַ+��
6.��ȡ���ݲ��ط�NACK
7.����ֹͣ�ź�
********************************************************************************/
MPU9250_STATUS Multi_Read(u8 SlaveAddress,u8 REG_Address,u8 len, u8 *buf)
{
	IIC_Start();//��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);//���ʹӻ���ַ+д�ź� 0xD0    110100x+��дλ(��1д0)
	if(IIC_Wait_Ack())//����0  ����Ӧ��ɹ�  1����Ӧ��ʧ��
	{
		IIC_Stop();//����ֹͣ�ź�
		return MPU9250_FAIL;//����ʧ��
	}
	IIC_Send_Byte(REG_Address);//���ͼĴ�����ַ
	IIC_Wait_Ack();	
	IIC_Start();//��ʼ�ź�
	IIC_Send_Byte(SlaveAddress+1);//���ʹӻ���ַ+���ź� 0xD1    110100x+��дλ(��1д0)
	IIC_Wait_Ack();
	while(len)
	{
		if(len == 1)//len=1��ʾֻ��ȡһ���Ĵ���
		{
			*buf = IIC_Read_Byte(0);//��ȡһ���ֽڲ��ط�NACK
		}
		else
		{
			*buf = IIC_Read_Byte(1);//��ȡһ���ֽڲ��ط�ACK
		}
		len--;
		buf++;
	}
	IIC_Stop();//����ֹͣ�ź�
	return MPU9250_OK;
}




/******************************************************************************
** ���ܣ�   	���ֽ�дMPU9250�Ĵ���
** ������	u8 SlaveAddress  �ӻ���ַ
			u8 REG_Address   Ҫд��ļĴ�����ַ 
			u8 REG_data		 Ҫд��ļĴ�������
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
110100x+��дλ(��1д0)    0xD0-д   0xD1-��
1.�ȷ��ʹӻ���ַ+д    
2.�ڷ��ͼĴ�����ַ
3.�ڷ���Ҫд�������
4.����ֹͣ�ź�
********************************************************************************/
MPU9250_STATUS Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data)		 
{
	IIC_Start();//��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);//���ʹӻ���ַ+д�ź� 0xD0    110100x+��дλ(��1д0)
	if(IIC_Wait_Ack())//����0  ����Ӧ��ɹ�  1����Ӧ��ʧ��
	{
		IIC_Stop();//����ֹͣ�ź�
		return MPU9250_FAIL;//����ʧ��
	}
	IIC_Send_Byte(REG_Address);//���ͼĴ�����ַ
	IIC_Wait_Ack();	
	IIC_Send_Byte(REG_data);//���ͼĴ�����ַ
	IIC_Wait_Ack();
	IIC_Stop();//����ֹͣ�ź�
    return MPU9250_OK;
}




/******************************************************************************
** ���ܣ�   	���ֽ�дMPU9250�Ĵ���
** ������	u8 SlaveAddress  �ӻ���ַ
			u8 REG_Address   Ҫд�����ʼ�Ĵ�����ַ 
			u8 len			 Ҫд������ݵĸ���(Ҫд����ٸ������ļĴ���)
			u8 *buf		     д��Ĵ�������������
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
110100x+��дλ(��1д0)    0xD0-д   0xD1-��
1.�ȷ��ʹӻ���ַ+д    
2.�ڷ��ͼĴ�����ַ
3.��������Ҫд�������
4.����ֹͣ�ź�
********************************************************************************/
MPU9250_STATUS Multi_Write(u8 SlaveAddress,u8 REG_Address,u8 len,u8 *buf)
{
	u8 i;
	IIC_Start();//��ʼ�ź�
	//SlaveAddress << 1 | 0
	IIC_Send_Byte(SlaveAddress);//���ʹӻ���ַ+д�ź� 0xD0    110100x+��дλ(��1д0)
	if(IIC_Wait_Ack())//����0  ����Ӧ��ɹ�  1����Ӧ��ʧ��
	{
		IIC_Stop();//����ֹͣ�ź�
		return MPU9250_FAIL;//����ʧ��
	}
	IIC_Send_Byte(REG_Address);//���ͼĴ�����ַ
	IIC_Wait_Ack();	
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);//��������
		if(IIC_Wait_Ack())//����0  ����Ӧ��ɹ�  1����Ӧ��ʧ��
		{
			IIC_Stop();//����ֹͣ�ź�
			return MPU9250_FAIL;//����ʧ��
		}
	}
	IIC_Stop();//����ֹͣ�ź�
    return MPU9250_OK;
}




/******************************************************************************
** ���ܣ�   	����MPU9250������
** ������	u8 rate  ������  ��λHZ
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
SAMPLE_RATE = ���������Ƶ��/(1+SMPLRT_DIV)==>SMPLRT_DIV=1KHZ/SAMPLE_RATE-1
DLPF=0/7   	���������Ƶ��=8KHZ
DLPF=1-6    ���������Ƶ��=1KHZ
DLPF�˲�Ƶ��ͨ����Ϊ����Ƶ�ʵ�һ��(MPU6050)
				Gyroscope
DLPF_CFG	����(HZ)	��ʱ(ms)	������(KHZ)
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
	data = 1000/rate-1;//dataΪSAMPLE_DIV       rateΪ����Ƶ�� 
	data = Single_Write(MPU_ADDR,MPU_SAMPLE_RATE_REG,data);//�������ֵ�ͨ�˲���
	return MPU9250_Set_LPF(rate/2);//����MPU9250-LPFΪ������һ��
}



/******************************************************************************
** ���ܣ�   	����MPU9250���������ֵ�ͨ�˲���
** ������	u16 ���ֵ�ͨ�˲���
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
				Gyroscope
DLPF_CFG	����(HZ)	��ʱ(ms)	������(KHZ)
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
	return Single_Write(MPU_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}




/******************************************************************************
** ���ܣ�   	����MPU9250�����������̷�Χ
** ������	u8 fsr  0-3
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
********************************************************************************/
MPU9250_STATUS MPU9250_Set_Gyro_Fsr(u8 fsr)
{
	return Single_Write(MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}



/******************************************************************************
** ���ܣ�   	����MPU9250���ٶȼ������̷�Χ
** ������	u8 fsr  0-3
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
fsr:0,��2g;1,��4g;2,��8g;3,��16g
********************************************************************************/
MPU9250_STATUS MPU9250_Set_Accel_Fsr(u8 fsr)
{
	return Single_Write(MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȼ������̷�Χ
}





/******************************************************************************
** ���ܣ�   	��ȡMPU9250�¶�
** ������	short *temp  �����ȡ�������¶�ֵ
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����	�¶�ֵ������100��
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
** ���ܣ�   	��ȡMPU9250������
** ������	short *gx,short *gy,short *gz
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
x,y,z��ԭʼ����(������)
********************************************************************************/
MPU9250_STATUS READ_MPU9250_GYRO(short *gx,short *gy,short *gz)
{
	u8 buf[6];
	if(Multi_Read(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;//����ʧ��
	}
	else
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;//���سɹ�
}




/******************************************************************************
** ���ܣ�   	��ȡMPU9250���ٶ�
** ������	short *ax,short *ay,short *az
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
********************************************************************************/
MPU9250_STATUS READ_MPU9250_ACCEL(short *ax,short *ay,short *az)
{
	u8 buf[6];
	if(Multi_Read(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;//����ʧ��
	}
	else
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;//���سɹ�
}





/******************************************************************************
** ���ܣ�   	��ȡMPU9250������
** ������	short *ax,short *ay,short *az
** ����ֵ:   0���ɹ�    1��ʧ��
** ˵����
********************************************************************************/
MPU9250_STATUS READ_MPU9250_MAG(short *mx,short *my,short *mz)
{
	u8 buf[6];
	Single_Write(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);//������·ģʽ
	delay_ms(10);	
	Single_Write(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);//���뵥�β���ģʽ
	delay_ms(10);
	if(Multi_Read(MAG_ADDRESS,AKM8963_MAG_XOUTL_REG,6,buf))
	{
		return MPU9250_FAIL;//����ʧ��
	}
	else
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	}
	return MPU9250_OK;//���سɹ�
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

 /*�Լ캯��*/
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

