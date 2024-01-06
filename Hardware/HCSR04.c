#include "stm32f10x.h"
#include "Delay.h"

/*
具体使用说明请到我博客：// https://blog.zeruns.tech
*/

#define Echo GPIO_Pin_12		//HC-SR04模块的Echo脚接GPIOA12
#define Trig GPIO_Pin_11		//HC-SR04模块的Trig脚接GPIOA11

uint64_t time=0;			//声明变量，用来计时
uint64_t time_end=0;		//声明变量，存储回波信号时间
uint64_t sys_time=0;		//系统时间，每200ms读取一次时间

void HC_SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//启用GPIOA的外设时钟	
	GPIO_InitTypeDef GPIO_InitStructure;					//定义结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设置GPIO口为推挽输出
	GPIO_InitStructure.GPIO_Pin = Trig;						//设置GPIO口11
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//设置GPIO口速度50Mhz
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//初始化GPIOA
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			//设置GPIO口为下拉输入模式
	GPIO_InitStructure.GPIO_Pin = Echo;						//设置GPIO口12
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//初始化GPIOA
	GPIO_WriteBit(GPIOA,GPIO_Pin_12,0);						//输出低电平
	Delay_us(15);											//延时15微秒
}

float sonar_cm(void)									//测距并返回单位为厘米的距离结果
{
	uint32_t Distance_cm = 0;
	GPIO_WriteBit(GPIOA,Trig,1);						//输出高电平
	Delay_us(15);										//延时15微秒
	GPIO_WriteBit(GPIOA,Trig,0);						//输出低电平
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==0);		//等待低电平结束
	time=0;												//计时清零
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==1);		//等待高电平结束
	time_end=time;										//记录结束时的时间
	Distance_cm=((float)time_end/10/58);					//计算距离，25°C空气中的音速为346m/s
					//因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
	
	return Distance_cm;									//返回测距结果
}

int16_t sonar_mm(void)									//测距并返回单位为毫米的距离结果
{
	uint32_t Distance,Distance_mm = 0;
	GPIO_WriteBit(GPIOA,Trig,1);						//输出高电平
	Delay_us(15);										//延时15微秒
	GPIO_WriteBit(GPIOA,Trig,0);						//输出低电平
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==0);		//等待低电平结束
	time=0;												//计时清零
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==1);		//等待高电平结束
	time_end=time;										//记录结束时的时间
	if(time_end/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
	{
		Distance=(time_end*346)/2;						//计算距离，25°C空气中的音速为346m/s
		Distance_mm=Distance/100;						//因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
	}
	return Distance_mm;									//返回测距结果
}


float sonar(void)										//测距并返回单位为米的距离结果
{
	uint32_t Distance,Distance_mm = 0;
	float Distance_m=0;
	GPIO_WriteBit(GPIOA,Trig,1);					//输出高电平
	Delay_us(15);
	GPIO_WriteBit(GPIOA,Trig,0);					//输出低电平
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==0);     //如果echo一直为0，那就一直等待等待，直到echo为1，当echo为1的时候，立刻将时间重置为零
	time=0;											//然后一直等待echo为0的时候，结束计时，把此时的时间算出来就是当前的总共经历的时间
	while(GPIO_ReadInputDataBit(GPIOA,Echo)==1);     //一个time就是10us
	time_end=time;
	if(time_end/100<38)
	{
		Distance=(time_end*346)/2;
		Distance_mm=Distance/100;
		Distance_m=Distance_mm/1000;
	}
	return Distance_m;
}

/*
void TIM3_IRQHandler(void)			//更新中断函数，用来计时，每10微秒变量time加1
{									// https://blog.zeruns.tech
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)		//获取TIM3定时器的更新中断标志位
	{
		time++;
		sys_time++;
		if(sys_time ==20000)         //200毫秒读取一次数据
		{
			Get_Angle_Data();              //获取数据并将数据显示出来
			Get_Distance_Data();
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);			//清除更新中断标志位
	}
	
}
*/
