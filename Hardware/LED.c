#include "stm32f10x.h"                  // Device header

/**
  * 函    数：BEEP初始化
  * 参    数：无
  * 返 回 值：无
  */
void BEEP_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA0引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOA, GPIO_Pin_0);							//设置PA0引脚为高电平
}

/**
  * 函    数：BEEP1开启
  * 参    数：无
  * 返 回 值：无
  */
void BEEP1_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);		//设置PA0引脚为低电平
}

/**
  * 函    数：BEEP1关闭
  * 参    数：无
  * 返 回 值：无
  */
void BEEP1_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_0);		//设置PA0引脚为高电平
}

/**
  * 函    数：BEEP1状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void BEEP1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_0);					//则设置PA0引脚为高电平
	}
	else													//否则，即当前引脚输出高电平
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);					//则设置PA0引脚为低电平
	}
}


