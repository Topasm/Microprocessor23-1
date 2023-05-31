#include "stm32f767xx.h"
#include "OK-STM767.h"

void USART6_IRQHandler(void);	        /* USART6 interrupt function 선언 */

volatile unsigned int i=0;
volatile unsigned string[20] = “Hello my friend!”
volatile unsigned char RXD;

/* ----- 인터럽트 처리 프로그램 ----------------------------------------------- */

void USART6_IRQHandler(void)		/* USART3 interrupt function */
{
  if string[i] = != ‘\0’
    {TX6->TDR = string[i];
     TFT_English(string[i]);}
  i++;
}
   
/* ----- 메인 프로그램 -------------------------------------------------------- */

int main(void)
{
  Initialize_MCU();		// initialize MCU and kit
  Delay_ms(50);			// wait for system stabilization
 

  GPIOC->MODER &= 0xFFFF 0FFF; // PC6 : USART6_TX, PC7 ; USART6_RX (p.164)
  GPIOC->MODER |= 0x0000 A000;
  GPIOC->AFR[0] &= 0x00FF FFFF;  // PC6 : AF8, PC7 : AF8
  GPIOC->AFR[0] |= 0x8800 0000;


  RCC->APB2ENR |= 0x0000 0020;	// enable USART6 clock (p.121)
  RCC->AHB1ENR |= 0x0000 0004;	// enable GPIOC clock (p.121)


  USART6->CR1 = 0x0000008D;	// TXEIE=TE=RE=UE = 1, 8 data bit, even parity, 
                                // oversampling by 16, (p.339)
  USART6->CR2 = 0x0000 3000; // asynchronous mode, 1.5 stop bit, (p.340)
  USART6->CR3 = 0x00000000;	// 3 sampling bit (p.342)
  USART6->BRR = 5625;        // 9600 bps = 54MHz/5625, (p.343)
  Delay_ms(1);
  
  NVIC->ISER[2] = 0x00000080; // enable (71)USART6 interrupt. (p.144)

 USART6->TDR = ‘/0’ ;         // Send a dummy data
  RXD = USART3->RDR;	 // Read a dummy data

 while(1)
  {
    while(!(USART6->ISR & 0x00000020));	// wait until RXNE = 1 (p.344)
    RXD = USART6->RDR;
    TFT_English(RXD);
  }

 
}
