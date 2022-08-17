/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  ����
  * @date    2022/08/17
  * @brief   Main program body
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_uart1.h"
#include "stm8s_tim4.h"



/* Private defines -----------------------------------------------------------*/
#define u8 uint8_t;
#define u16 uint16_t;
#define u32 uint32_t;

/* Private function prototypes -----------------------------------------------*/

void wakeInit();//���û��ѷ�ʽ
void init();//��ʼ��
void blueToothInit();//����ģ��(��UART1����ʼ��
void CLKInit();//ʱ���л���ʼ��
void GPIOInit();//GPIO�ڳ�ʼ��
void recieveData();//��������������Ϣ��������������������Ϣ������/��λ��Ϣ
void motorControl();//���Ƶ������
void timerInit();//ʱ���������ʼ��
void enableOutput();//���ʹ��λ���
void messageInput();//���ܵ��������Ϣ
void delay(int delayTime);//��ʱ����

static bool isDone = 0;//������ɱ�־λ
static float distance = 0;//���ƶ�����
static bool direction = 0;//��������false��λ��trueǰ��
static float disPerPulse = 12/10000;//��Ȧ��������/ÿȦ��Ӧ������
static int num = 0;//ת��Ȧ��

/* Private functions ---------------------------------------------------------*/

void wakeInit(){
  GPIO_Init(GPIOA,GPIO_PIN_4,GPIO_MODE_IN_PU_IT);//����PA4(UART1_RX)Ϊ�ⲿ�������ţ�����ĵ�һ���������ڻ���
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,EXTI_SENSITIVITY_FALL_LOW);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTA,ITC_PRIORITYLEVEL_2);//�趨��һ���жϵȼ�
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
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;               // CPUʱ�ӷ�Ƶ1��CPUʱ�� = �ⲿʱ��
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);

}


void GPIOInit(){
  GPIO_Init(GPIOB,GPIO_PIN_LNIB|GPIO_PIN_4|GPIO_PIN_5,GPIO_MODE_OUT_PP_LOW_SLOW);//PB0-5����������
  GPIO_Init(GPIOC,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6,GPIO_MODE_IN_PU_NO_IT);//PC2356�������ж�����
}

void recieveData(){
  distance = UART1_ReceiveData8();//����
  direction =(bool) (UART1_ReceiveData8() >> 7);//ת������,false��λ��trueǰ��
}

void motorControl(){
  num = (int)(distance / disPerPulse);//����õ���ӦȦ��
  enableOutput();
  timerInit();
  while(num);
  messageInput();
}


void timerInit(){
  TIM4_DeInit();
  TIM4_TimeBaseInit(TIM4_PRESCALER_16,10);//Ƶ��Ϊ16/16=1Mhz���������Ϊ10����ÿ10us�����ж�
  TIM4_Cmd(ENABLE);
  TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF,ITC_PRIORITYLEVEL_3);//�趨���жϵȼ�
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
  isDone = GPIO_ReadInputPin(GPIOC,GPIO_PIN_5); ;//TODO:�������������б�
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

#pragma vector = 0x05//A�˿��ⲿ�����ж�
__interrupt void EXTI_PORTA_IRQHandler(){
  init();
  delay(10);
  recieveData();
  motorControl();
  delay(10);
  asm("HALT");
}

#pragma vector = 0x19//TIM4����������岨
__interrupt void TIM4_UPD_OVF_IRQHandler(){
  GPIO_WriteReverse(GPIOB,GPIO_PIN_4);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);//������
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  num--;
}





/******************************************END OF FILE****/
