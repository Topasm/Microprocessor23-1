#include "stm32f767xx.h"

#include "OK-STM767.h"

#include "OK-STM767_large.h"

void Delay_us(U32 time_us); // time delay for us in 216MHz

void Delay_ms(U32 time_ms);

// unsigned char RX3_char(void);			    /* receivse a character by USART3 */

// void TX3_char(U08 data);			           /* transmit a character by USART3 */

// void TX3_string(U08 *string);			  /* transmit a string by USART3 */

int main(void)

{

  Initialize_MCU(); // initialize MCU and kit

  Delay_ms(50); // wait for system stabilization

  Initialize_LCD(); // initialize text LCD module

  Initialize_TFT_LCD();

  /* unsigned char RXD, count;

   unsigned short color;

   */

  float r;

  float x;

  float t;

  unsigned result;

  unsigned result2;

  float Distance;

  RCC->AHB1ENR |= 0x00000001; //      포트 a클락 온

  GPIOA->MODER &= 0xFFFF0FFF;

  GPIOA->MODER |= 0x00001000; //      포트6 출력모드(초음파의 트리거 신호)   포트7 입력모드(에코신호를 키트의 입력으로 받음)

  TIM7->PSC = 1079; // 108MHz/(1079+1) = 100khz

  TIM7->CNT = 0; // clear counter

  TIM7->DIER = 0x0000; // enable update interrupt

  TIM7->CR1 = 0x0005;

  while (1)

  {

    void trigger_input(); // 12us의 트리거 신호를 초음파센서에다 보냄

    while ((GPIOA->IDR & 0x00000080) != 0x00000080)
      ; //( IDR은 입력값을 의미)  포트7의 0이 읽히면  에코가 로우

    // 로우가 끝날때까지 대기

    TIM7->CNT = 0; // 로우가 끝나면 카운터 초기화하고 시작

    TIM7->CR1 = 0x0005;

    while ((GPIOA->IDR & 0x00000080) == 0x00000080)
      ; // 에코가 하이인동안(포트7이 1이 읽히면) 대기

    TIM7->CR1 = 0x0000; // high가 끝나면 타이머 끄고

    result2 = TIM7->CNT; // 현재의 시간값을 result2에 옮겨줌

    TIM7->CNT = 0; // 카운터 클리어

    TFT_xy(24, 9); // TFT의 시간값을 띄워준다

    TFT_color(Cyan, Black);

    TFT_signed_float((float)result2, 4, 4);

    r = result2 * 10 / 58; // 거리값 계산

    TFT_xy(24, 11);

    TFT_color(Cyan, Black);

    TFT_signed_float((float)r, 4, 4);

    Delay_ms(250);
  }
}

void trigger_input(void)

{

  GPIOA->ODR = 0x00000040; // 6번핀 mco2

  Delay_us(12);

  GPIOA->ODR = 0x00000000;

  Delay_us(12);
}
