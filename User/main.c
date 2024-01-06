#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
//#include "rc522.h"
#include "Servo.h"
#include "MPU6050.h"
#include "LED.h"
#include "HCSR04.h"
#include "Timer.h"
#include "math.h"
#include "Angle_Calculate.h"
#include "bsp_dht11.h"

// 基本数据读取\USER\src\main.c 
/* 传感器数据修正值（消除芯片固定误差，根据硬件进行调整） */
#define X_ACCEL_OFFSET -190 
#define Y_ACCEL_OFFSET -8
#define Z_ACCEL_OFFSET -2430 
#define X_GYRO_OFFSET 54 
#define Y_GYRO_OFFSET -44 
#define Z_GYRO_OFFSET 1 

uint8_t ID;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量
float ax,ay,az,gx,gy,gz;      			//用于存放处理后的数据
//CARD card_data;
//全局变量
extern unsigned char CT[2];


uint8_t cardnumber,KeyNum,tempcard,select=0,flag_scan=1,flag_addcard=0,flag_deletecard=0;

extern uint8_t UID[4],Temp[4];
extern uint8_t UI0[4];							//卡片0ID数组
extern uint8_t UI1[4];							//卡片1ID数组
extern uint8_t UI2[4];							//卡片2ID数组
extern uint8_t UI3[4];							//卡片3ID数组

void RFID_Check(void);
void Read_Card(void);

//读取卡号
u8 IC_Card_uid[4];

int Distance_mm;
float Distance_cm;		                 //获取距离测量结果，单位毫米（mm）		
int Distance_cm_int;	                         //转换为米（m）为单位，将整数部分放入Distance_m
int Distance_cm_p;	                     //转换为米（m）为单位，将小数部分放入Distance_m_p



extern float Yaw,Pitch,Roll;             //偏航角，俯仰角，翻滚角

extern uint64_t sys_time;

uint64_t numlen(uint64_t num)//计算数字的长度
{
    uint64_t len = 1;        // 初始长度为1
    for(; num > 9; ++len)    // 判断num是否大于9，否则长度+1
        num /= 10;	         // 使用除法进行运算，直到num小于1
    return len;              // 返回长度的值
}



/*
	函数：身份认证
*/
int authentication(void)
{
	
	int flag;
	
	return flag;
	
}

/*
	函数：获取当前倾角并显示出来
*/

float Get_Angle_Data(void)
{
	float angle;
	float pi = 3.1415926;
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
	
	ax = 9.8*(AX+X_ACCEL_OFFSET)/2048;
	ay = 9.8*(AY+Y_ACCEL_OFFSET)/2048;			//单位：m/s2
	az = 9.8*(AZ+Z_ACCEL_OFFSET)/2048;
	gx = (pi/180)*(GX+X_GYRO_OFFSET)/16.4;      //单位：rad/s
	gy = (pi/180)*(GY+Y_GYRO_OFFSET)/16.4;
	gz = (pi/180)*(GZ+Z_GYRO_OFFSET)/16.4;
	
	IMUupdate(gx,gy,gz,ax,ay,az);           //四元法计算角度
	
	//OLED_ShowSignedNum(2, 1, ax, 5);					//OLED显示数据
	//OLED_ShowSignedNum(3, 1, ay, 5);
	//OLED_ShowSignedNum(4, 1, az, 5);
	//OLED_ShowSignedNum(2, 8, Yaw, 5);
	//OLED_ShowSignedNum(3, 8, Pitch, 5);
	//OLED_ShowSignedNum(4, 8, Roll, 5);
	
	
	Serial_Printf("Yaw:%d\t",Yaw);
	Serial_Printf("Pitch:%d\t",Pitch);
	Serial_Printf("Roll:%d\r\n",Roll);

	//这里暂且设置为ax|ay|az>3,就开始报警叭
	
	
	return angle;
}


/*
	函数：获取当前距离并显示出来
	功能：在OLED上显示当前距离

*/
void Get_Distance_Data(void)
{
	/*
	Distance_cm=sonar_cm();			//获取距离测量结果，单位毫米（mm）		
	Serial_Serial_Printf("Distance:%f.cm\r\n",Distance_cm);
	
    OLED_ShowNum(3, 1, (int)(Distance_cm), 5);
    OLED_ShowNum(3, 4, ((int)(Distance_cm*100) % 100), 2);
	*/
	int Distance_mm=sonar_mm();			//获取距离测量结果，单位毫米（mm）		
	OLED_ShowNum(3, 1,Distance_mm,numlen(Distance_mm));		//显示单位为毫米的距离结果
	OLED_ShowString(3, 1 + numlen(Distance_mm), "mm");
	
}

/*
	函数：获取温湿度
*/

void Get_Temp_Humi_Data(void)
{
	DHT11_Data_TypeDef DHT11_Data;
	
	if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)//if( DHT11_Data.humi_deci)
			{
				Serial_Printf("Humi:%d.%dRH  ,Temp: %d.%dC \r\n",\
				DHT11_Data.humi_int,DHT11_Data.humi_deci,DHT11_Data.temp_int,DHT11_Data.temp_deci);
			}			
			else
			{
				Serial_Printf("Read DHT11 ERROR!\r\n");
			}

}



/*
	函数：开关门
*/
void Open_door(void);
void Close_door(void);


/*
	函数：蜂鸣器报警
*/
void BEEP1_ON(void);
void BEEP1_OFF(void);







int main(void)
{	

	DHT11_Data_TypeDef DHT11_Data;
	
	OLED_Init();       //OLED屏幕初始化
	BEEP_Init();	
	Serial_Init();		//串口初始化	
	//RC522_Init();		//RFID初始化
	Servo_Init();		//舵机初始化
	MPU6050_Init();
	HC_SR04_Init();
	DHT11_Init();
	Timer_Init();		//初始化定时器
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	Serial_Printf("test");
	OLED_ShowString(1,1,"Initialize...");     //初始化界面
	Servo_SetAngle(0);  //复位舵机
	while(1)
	{
		
		int Distance_mm=sonar_mm();			//获取距离测量结果，单位毫米（mm）		
		OLED_ShowNum(3, 1,Distance_mm,numlen(Distance_mm));		//显示单位为毫米的距离结果
		OLED_ShowString(3, 1 + numlen(Distance_mm), "mm");
		Delay_ms(1000);
		
		
		
		
			Get_Temp_Humi_Data();
			Get_Angle_Data();
			sys_time = 0;
		
		
			//OLED_Clear();
			//距离小于10mm就自动开门
			if(Distance_mm < 50)       
			{
				Open_door();

			}
			else
			{
				Close_door();
			}

			if(ax>3|ay>3|az>3)   //判断地震了
			{
				BEEP1_ON();
			}		
			
		
		
			Serial_Printf("sys_time:%d",sys_time);	
			
			
			/*
			RFID_Check();
			Delay_ms(500);
			*/
	}
}



/*
//读卡函数，读卡并获取卡编号
void RFID_Check()									
{
	cardnumber = Rc522Test();	//获取卡编号
	if(cardnumber == 0)			//如果为0，表示“卡片错误”，系统中没有这张卡
	{
		OLED_ShowString(1,1,"   Error card   ");
		//Buzzer_Alarm();		//蜂鸣器发出警报
		WaitCardOff();		//等待卡片移开
	}
	else if(cardnumber==1||cardnumber==2||cardnumber==3||cardnumber == 4)			//如果卡编号为1-4，说明是系统中的4张卡
	{	
		OLED_ShowString(1,1,"The CardID is:  ");
		OLED_ShowNum(1,15,cardnumber,2);
		//Buzzer2();			//蜂鸣器响两声
		Servo_SetAngle(90);	//舵机旋转90度维持1.5秒
		Delay_ms(1500);
		Servo_SetAngle(0);
		WaitCardOff();		//等待卡片移开
	}	
}

//从flash中读取各卡信息

void Read_Card()
{
	UI0[0]=FLASH_R(FLASH_ADDR1);
	UI0[1]=FLASH_R(FLASH_ADDR1+2);
	UI0[2]=FLASH_R(FLASH_ADDR1+4);
	UI0[3]=FLASH_R(FLASH_ADDR1+6);
	
	UI1[0]=FLASH_R(FLASH_ADDR2);
	UI1[1]=FLASH_R(FLASH_ADDR2+2);
	UI1[2]=FLASH_R(FLASH_ADDR2+4);
	UI1[3]=FLASH_R(FLASH_ADDR2+6);
	
	UI2[0]=FLASH_R(FLASH_ADDR3);
	UI2[1]=FLASH_R(FLASH_ADDR3+2);
	UI2[2]=FLASH_R(FLASH_ADDR3+4);
	UI2[3]=FLASH_R(FLASH_ADDR3+6);
	
	UI3[0]=FLASH_R(FLASH_ADDR4);
	UI3[1]=FLASH_R(FLASH_ADDR4+2);
	UI3[2]=FLASH_R(FLASH_ADDR4+4);
	UI3[3]=FLASH_R(FLASH_ADDR4+6);
}
*/

