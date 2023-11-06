#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
//灰色 SWIO  7 左4
//白色 SWCLK 9 左5
//黑色 GND 右 2


extern vu16 var_Exp;
int main(void)
{

	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	//uart_init(115200);	 //串口初始化为115200
	uart_init(9600);
 	LED_Init();			     //LED端口初始化
	//1000 ms 50 
	TIM2_PWM_Init(999,7199); // 10 Hz    pin_A1       

 	TIM3_PWM_Init(9999,7199);	 // 1 Hz  pin_B5


	
	while(1)
	{
		
	}
}
