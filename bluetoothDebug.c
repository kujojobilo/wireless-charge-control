/**
  ******************************************************************************
  * @file    Project/bluetoothDebug.c 
  * @author  王田
  * @date    2022/09/06
  * @brief   for debugging HC-08
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_uart1.h"
#include "stm8s_tim1.h"



/* Private defines -----------------------------------------------------------*/
#define u8 uint8_t;
#define u16 uint16_t;
#define u32 uint32_t;

/* Private function prototypes -----------------------------------------------*/
void CLKInit();
void watchDog();
void uart1Init();
void GPIOInit();
void hc08DebugInit();
void LED_SendSucceed();
void LED_RecieveCorrect();
void LED_RecieveWrong();
void sendAndCompare(char *ATCmd,char ATRcv[]);
void hc08DebugDeinit();
void delay(int delayTime);

/* Private functions ---------------------------------------------------------*/
void CLKInit(){
  CLK_HSECmd(ENABLE);
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;               // CPU时钟分频1，CPU时钟 = 外部时钟
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);

}

void watchDog(){ 
  //PC1初始化
  GPIO_Init(GPIOC,GPIO_PIN_1,GPIO_MODE_OUT_PP_LOW_SLOW);
  //TIM1初始化
   TIM1_DeInit();
   TIM1_TimeBaseInit(16,TIM1_COUNTERMODE_UP,7999,0x00);//16分频（1MHz)即1us，实际周期记数为8000个周期
   TIM1_OC1Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_DISABLE,3999,
                TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,
               TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_SET);
   TIM1_Cmd(ENABLE);
   TIM1_CtrlPWMOutputs(ENABLE);
}

void uart1Init(){
  UART1_DeInit();
  UART1_Init(9600,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TXRX_ENABLE);
}

void GPIOInit(){//PD347设置低速推挽输出
  GPIO_Init(GPIOD,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7,GPIO_MODE_OUT_PP_LOW_SLOW);
}

void hc08DebugInit(){
  GPIO_WriteHigh(GPIOD,GPIO_PIN_7);//PD7置高电平进入调试模式
  GPIO_WriteLow(GPIOD,GPIO_PIN_3);//PD3置低电平重启hc08
  delay(10);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
}

void LED_SendSucceed(){//成功发送信息
  int num = 3;
  while(num--){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
    delay(2);
    GPIO_WriteLow(GPIOD,GPIO_PIN_4);
    delay(8);
  }
}

void LED_RecieveCorret(){//接收到设想的字符串信息
  GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
  delay(20);
  GPIO_WriteLow(GPIOD,GPIO_PIN_4);
  delay(10);
}

void LED_RecieveWrong(){//接收到错误信息
  int num = 9;
    while(num--){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
    delay(1);
    GPIO_WriteLow(GPIOD,GPIO_PIN_4);
    delay(1);
   }
}

void sendAndCompare(char *ATCmd,char ATRcv[]){
  //发送不定长字符串
   int k = 0;
   do{
     UART1_SendData(*(ATCmd+k));
     while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
     k++;
   }while(*(ATCmd+k) != '\0');
   LED_SendSucceed();
   delay(10);
   //接收不定长字符串
   while(UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
   k = 0;
   char Rcv[64] = {0};
   do{
     Rcv[k] = UART1_ReceiveData8();
     while(UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
     k++;
   }while(Rcv[k-1] != '\0');
   delay(10);
   //与预想字符串做比较
   int l = strlen(ATRcv);
   k = 0;
   while(ATRcv[k] == Rcv[k]){
     k++;
   }
   if(k == l){
     LED_RecieveCorrect();
   }else{
     LED_RecieveWrong();
   } 
}

void hc08DebugDeinit(){
  GPIO_WriteLow(GPIOD,GPIO_PIN_7);//PD7置低电平进入正常模式
  GPIO_WriteLow(GPIOD,GPIO_PIN_3);//PD3置低电平重启hc08
  delay(10);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
}

void delay(int delayTime){
  while(delayTime--){
    int k = 10000;
    while(k--);
  }
}




/* Main functions ---------------------------------------------------------*/
void main(){
  CLK_Init();
  watchDog();
  uart1Init();
  GPIOInit();
  hc08DebugInit();
  delay(10);
  /*
   以下内容为随需更改，通过sendAndCompare将AT指令传输至蓝牙芯片，并通过LED的状态判断是否成功输出指令
   更改日期2022/09/06
   目前功能：展示函数功能
*/
  sendAndCompare("AT","AT");
  delay(10);
  sendAndCompare("AT+ROLE=S","Slave");//设置该模块为丛机
  delay(10);
  hc08DebugDeinit();
}


/******************************************END OF FILE****/

