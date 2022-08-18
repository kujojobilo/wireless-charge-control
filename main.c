/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  王田
  * @date    2022/08/17
  * @brief   Main program body
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_uart1.h"
#include "stm8s_tim4.h"
#include "stm8s_tim1.h"



/* Private defines -----------------------------------------------------------*/
#define u8 uint8_t;
#define u16 uint16_t;
#define u32 uint32_t;

/* Private function prototypes -----------------------------------------------*/

void watchDog();//配置看门狗CLK输出
void wakeInit();//配置唤醒方式
void init();//初始化
void blueToothInit();//蓝牙模块(即UART1）初始化
void CLKInit();//时钟切换初始化
void GPIOInit();//GPIO口初始化
void recieveData();//接受蓝牙传输信息，数据量包括：距离信息，进给/复位信息
void motorControl();//控制电机过程
void timerInit();//时间控制器初始化
void enableOutput();//电机使能位输出
void messageInput();//接受电机返回信息
void delay(int delayTime);//延时函数

static bool isDone = 0;//工作完成标志位
static float distance = 0;//待移动距离
static bool direction = 0;//进给方向，false复位，true前进
static float disPerPulse = 12/10000;//单圈进给距离/每圈对应脉冲数  (进给距离待测定）
static int num = 0;//转动圈数

/* Private functions ---------------------------------------------------------*/

void watchDog(){//配置pc1（TIM1-CH1）引脚输出，为看门狗提供周期信号
   GPIO_Init(GPIOC,GPIO_PIN_1,GPIO_MODE_OUT_PP_LOW_SLOW);
   TIM1_DeInit();
   TIM1_TimeBaseInit(0x0100,TIM1_COUNTERMODE_UP,20,0x00);
   TIM1_OC1Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_DISABLE,0xFF00,
                TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,
               TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_SET);
   TIM1_Cmd(ENABLE);
   TIM1_CtrlPWMOutputs(ENABLE);
}

void wakeInit(){
  GPIO_Init(GPIOA,GPIO_PIN_4,GPIO_MODE_IN_PU_IT);//配置PA4(UART1_RX)为外部唤醒引脚，传输的第一份数据用于唤醒
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,EXTI_SENSITIVITY_FALL_LOW);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTA,ITC_PRIORITYLEVEL_2);//设定次一级中断等级
}

void init(){
  blueToothInit();
  CLKInit();
  GPIOInit();
}

void blueToothInit(){
  UART1_DeInit();
  UART1_Init(9600,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TXRX_ENABLE);
}

void CLKInit(){
  CLK_HSECmd(ENABLE);
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;               // CPU时钟分频1，CPU时钟 = 外部时钟
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);

}


void GPIOInit(){
  GPIO_Init(GPIOB,GPIO_PIN_LNIB|GPIO_PIN_4|GPIO_PIN_5,GPIO_MODE_OUT_PP_LOW_SLOW);//PB0-5推挽低速输出
  GPIO_Init(GPIOC,GPIO_PIN_1,GPIO_MODE_OUT_PP_LOW_SLOW);//PC1推挽低速输出,用于喂看门狗
  GPIO_Init(GPIOC,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6,GPIO_MODE_IN_PU_NO_IT);//PC2356上拉禁中断输入
}

void recieveData(){
  distance = UART1_ReceiveData8();//距离
  direction =(bool) (UART1_ReceiveData8() >> 7);//转动方向,false复位，true前进
}

void motorControl(){
  num = (int)(distance / disPerPulse);//相除得到对应圈数
  enableOutput();
  timerInit();
  while(num);
  messageInput();
}


void timerInit(){
  TIM4_DeInit();
  TIM4_TimeBaseInit(TIM4_PRESCALER_16,10);//频率为16/16=1Mhz，清除计数为10，即每10us触发中断
  TIM4_Cmd(ENABLE);
  TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF,ITC_PRIORITYLEVEL_3);//设定高中断等级
  CLK_ITConfig(CLK_PERIPHERAL_TIMER4,ENABLE);
}

void enableOutput(){
  if(direction){
    GPIO_Write(GPIOB,0x24);//PB:0010|0100
  }else{
    GPIO_Write(GPIOB,0x08);//PB:0000|1000
  }
}


void messageInput(){
  isDone = GPIO_ReadInputPin(GPIOC,GPIO_PIN_5); ;//TODO:其他每一位输入的作用尚不明白，未添加
}

void delay(int delayTime){
  while(delayTime--){
    int k = 10000;
    while(k--);
  }
}

/* Main functions ---------------------------------------------------------*/

void main()
{
  wakeInit();
  asm("RIM");
  delay(10);
  asm("HALT");
  while(1);
}

#pragma vector = 0x05//A端口外部唤醒中断
__interrupt void EXTI_PORTA_IRQHandler(){
  init();
  delay(10);
  recieveData();
  motorControl();
  delay(10);
  asm("HALT");
}

#pragma vector = 0x19//TIM4计数输出脉冲波
__interrupt void TIM4_UPD_OVF_IRQHandler(){
  GPIO_WriteReverse(GPIOB,GPIO_PIN_4);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);//清除相关
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  num--;
}





/******************************************END OF FILE****/
