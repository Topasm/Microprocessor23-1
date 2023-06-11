#include "stm32f767xx.h"
#include "OK-STM767.h"

void LCD_init(void) {}
volatile unsigned long t_cnt = 0;

uint32_t tick;
uint16_t cnt1, cnt2, diff, speed, dir;

void SysTick_Handler(void)         /* SysTick interrupt function */
{
  t_cnt++;
 }
void Init_SysTick(void)
{

  SysTick->LOAD  = 26;           //  0.00001초 계산하기(1MHz) 10us
  SysTick->VAL    = 0;                    // SysTick Counter 초기화
  SysTick->CTRL  = 0x00000003;         // 216MHz/8 = 27MHz로 SysTick 타이머 초기 설정
}
int main(){
  
      Initialize_MCU();				// initialize MCU and kit
  Delay_ms(50);					// wait for system stabilization
  Initialize_LCD();
    GPIOD->MODER &= 0xFAFFFFFF; // PD12,13(TIM CH1,2) = AF
    GPIOD->MODER |= 0x0A000000;
    GPIOD->AFR[1] &= 0xFF22FEEE; // PD12,13- TIM4_CH1, TIM4CH2
    GPIOD->AFR[1] |= 0x00220000;

    RCC->APB1ENR |= 0x00000004; // enable TIM4 clock

    TIM4->PSC = 0;            // No PSC
    TIM4->ARR = 4680;         // PPR(ich)" 90
    TIM4->CNT = 0;            // Clear counter
    TIM4->CCER &= 0xFFFFFFCC; // Positive polarity , Capture disable
    TIM4->CCMR1 = 0x00000101; // TIM4CHI ->TI1,TI_CH2 -> TI2 mapping

    TIM4->SMCR = 0x00000003; // 4-multiplication
    TIM4->CR1 = 0x0005;      // Counter enable

    Init_SysTick();

    cnt1 = TIM3->CNT;
    tick = t_cnt;
   
  LCD_init(); //LCD initialization

  while(1)
  {
    LCD_command(0x86);
    float encodercount = TIM4->CNT;
    LCD_unsigned_float(encodercount, 4, 1);
  }

}


