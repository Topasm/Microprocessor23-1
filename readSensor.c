#include "stm32f767xx.h"
#include "OK-STM767.h"
#include "OK-STM767_large.h"
void Delay_us(U32 time_us); // time delay for us in 216MHz
void Delay_ms(U32 time_ms);
volatile unsigned long t_cnt = 0;

//디버깅 해야되서 핀 바꿈 PD12, 13, 15(vcc)



void trig_pulse(void)

{
    GPIOD -> ODR |= 0x00001000; // 12
    Delay_us(12);
    GPIOD -> ODR &= ~(0x00001000);
    Delay_us(12);

}
    
void SysTick_Handler(void);         /* SysTick interrupt function */

volatile unsigned int count = 0;


void SysTick_Handler(void)         /* SysTick interrupt function */
{
  t_cnt++;
 }
void Init_SysTick(void)
{

  SysTick->LOAD  = 26;           //  0.000001초 계산하기
  SysTick->VAL    = 0;                    // SysTick Counter 초기화
  SysTick->CTRL  = 0x00000003;         // 216MHz/8 = 27MHz로 SysTick 타이머 초기 설정
}


unsigned long echo_time(void)
{
    unsigned long echo;
    trig_pulse(); // give trig pulse to u_sonic sensor
    while ((GPIOD -> IDR & 0x00002000) != 0x00002000); //에코가 하이인동안(포트13이 1이 읽히면) 
    SysTick->CTRL |= 0x00000001; // start systick timer
    //LCD_unsigned_decimal(t_cnt,1,4);
    while ((GPIOD -> IDR & 0x00002000) == 0x00002000); //( IDR은 입력값을 의미)  포트13의 0이 읽히면  에코가 로우
    echo = t_cnt;
    SysTick->CTRL &= ~(0x00000001); // stop systick timer
    t_cnt = 0; // clear t_cnt
    
    return echo;
}

int main(void)

{
    Init_SysTick();
    Initialize_MCU(); // initialize MCU and kit
    Delay_ms(50); // wait for system stabilization
    Initialize_LCD(); // initialize text LCD module
    
    unsigned long echo = 0;
 
    GPIOD -> MODER &= 0x00FFFFFF;
    GPIOD -> MODER |= 0x41000000; //      포트12 출력모드(초음파의 트리거 신호)   포트13 입력모드(에코신호를 키트의 입력으로 받음)




        
       while(1)
    {
       echo = echo_time();
       LCD_unsigned_decimal(17*echo/100,1,4);
       Delay_ms(400);
    }


        

}
