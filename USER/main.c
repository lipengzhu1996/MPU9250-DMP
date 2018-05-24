#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "NVIC.h"
#include "myiic.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "MPU9250.h"
#include "TIM.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
    
#define q30  1073741824.0f


float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;


u8 show_tab[]={"0123456789"};

float Pitch,Roll,Yaw;
int temp;
u8 ms_10,ms_100,sec,min;
float second;
 
 
struct rx_s 
{
    unsigned char header[3];
    unsigned char cmd;
};


struct hal_s 
{
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};


static struct hal_s hal = {0};


/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

                                           
enum packet_type_e
{
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

 


//声明相关变量
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];

//误差纠正
#define  Pitch_error  1.0
#define  Roll_error   -2.0
#define  Yaw_error    0.0


int main()
{
	u8 result=0;
	
	delay_init(84);	    				//延时函数初始化	  
	uart_init(9600);	 				//串口初始化为9600
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	NVIC_init();
	IIC_Init();
	timer_init();
	
	delay_ms(200);
	
	result = mpu_init();//初始化
	if(result)
	{
		printf("mpu init failed...%d\n",result);
		delay_ms(2000);
	}
	if(!result)   //返回0代表初始化成功
    {   
        printf("mpu initialization complete......\n ");
        
        //mpu_set_sensor
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//配置时钟；打开accel、gyro sensor；
        {
            printf("mpu_set_sensor complete ......\n");
        }
        else
        {
            printf("mpu_set_sensor come across error ......\n");
        }
        
        //mpu_configure_fifo
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//配置fifo；打开中断
        {
            printf("mpu_configure_fifo complete ......\n");
        }
        else
        {
            printf("mpu_configure_fifo come across error ......\n");
        }
        
        //mpu_set_sample_rate
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))		//设置accel、gyro、compass采样频率；配置lpf
        {
            printf("mpu_set_sample_rate complete ......\n");
        }
        else
        {
            printf("mpu_set_sample_rate error ......\n");
        }
        
        //dmp_load_motion_driver_firmvare			
        if(!dmp_load_motion_driver_firmware())
        {
            printf("dmp_load_motion_driver_firmware complete ......\n");
        }
        else
        {
			printf("dmp_load_motion_driver_firmware come across error ......\n");
        }
        
        //dmp_set_orientation
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        {
            printf("dmp_set_orientation complete ......\n");
        }
        else
        {
            printf("dmp_set_orientation come across error ......\n");
        }
        
        //dmp_enable_feature
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
        {
			printf("dmp_enable_feature complete ......\n");
        }
        else
        {
			printf("dmp_enable_feature come across error ......\n");
        }
        
        //dmp_set_fifo_rate
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        {
            printf("dmp_set_fifo_rate complete ......\n");
        }
        else
        {
            printf("dmp_set_fifo_rate come across error ......\n");
        }
        
        run_self_test();
        
        if(!mpu_set_dmp_state(1))
        {
            printf("mpu_set_dmp_state complete ......\n");
        }
        else
        {
            printf("mpu_set_dmp_state come across error ......\n");
        }
        
    }
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	
	while(1)
	{
		 //float Yaw,Roll,Pitch;
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 

		//四元数解姿态
        if (sensors & INV_WXYZ_QUAT )
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            
            Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll
            Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 + Yaw_error;
			printf("Pitch:%.1f ",Pitch);
			printf("Roll:%.1f  ",Roll);
			printf("Yaw:%.1f   ",Yaw);
			printf("\r\n");
			
//			MPU_Matlab_report(Pitch,Roll,Yaw,min,sec,ms_100,ms_10);
			delay_ms(10);
		}
	}

}

void TIM4_IRQHandler(void)		//定时器4中断处理函数
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		ms_10++;
		if(ms_10==10)
		{
			ms_100++;
			ms_10=0;
		}
		if(ms_100==10)
		{
			sec++;
			ms_100=0;
		}
		if(sec==60)
		{
			min++;
			sec=0;
		}

		
//		printf("%d:%d\r\n",min,sec);
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}





