#include "stm32f767xx.h"
#include "OK-STM767.h"

volatile unsigned long t_cnt = 0;
float V_DAC = 0;
volatile unsigned char RXD, count;
volatile unsigned short color;
void SysTick_Handler(void); /* SysTick interrupt function */
// volatile unsigned int count = 0;
void USART7_IRQHandler(void);

float Kp = 20, Ki = 1, Kd = 10;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int targetcount = 3000;
int encodercount = 100;

void trig_pulse(void)
{
  GPIOD->ODR |= (1 << 0); // PD0 ON
  Delay_us(11);
  GPIOD->ODR &= ~(1 << 0); //PD0 OFF
  Delay_us(11);
}

void SysTick_Handler(void) /* SysTick interrupt function */
{
  t_cnt++;
}
void Init_SysTick(void)
{
  SysTick->LOAD = 26;         //  27M/ (26+1)의 역수 0.000001초 계산하기(1MHz) 1us
  SysTick->VAL = 0;           // SysTick Counter 초기화
  SysTick->CTRL = 0x00000003; // 216MHz/8 = 27MHz로 SysTick 타이머 초기 설정
}

unsigned long echo_time(void)
{
  unsigned long echo;
  trig_pulse();
  while ((GPIOD->IDR & (1 << 1)) == 0)
    ;                          // 에코가 하이인동안(포트1이 1이 읽히면)
  SysTick->CTRL |= 0x00000001; // start systick timer

  while ((GPIOD->IDR & (1 << 1)) != 0)
    ; //( IDR은 입력값을 의미)  포트13의 0이 읽히면  에코가 로우
  echo = t_cnt;
  SysTick->CTRL &= ~(0x00000001); // stop systick timer
  t_cnt = 0;                      // clear t_cnt

  return echo;
}

void USART7_IRQHandler(void)
{
  if (UART7->ISR & 0x00000020)
    ; // wait until RXNE = 1 (p.344)
  {
    RXD = UART7->RDR;
    LED_toggle();
  }
}

void TX7_char(U08 data)
{ // transmit a character by UART7
  while (!(UART7->ISR & 0x00000080))
    ; // wait until TXE = 1
  UART7->TDR = data;
}
void TX7_string(U08 *string)
{ // transmit a string by UART7
  while (*string != '\0')
  {
    TX7_char(*string);
    string++;
  }
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

int main()
{

  unsigned int PWM; // pulse width
  unsigned long echo;
  int distance;

  Initialize_MCU(); // initialize MCU and kit
  Delay_ms(50);     // wait for system stabilization
  Initialize_LCD();
  GPIOD->MODER &= 0xFAFFFFFF; // PD12,13(TIM CH1,2) = AF
  GPIOD->MODER |= 0x0A000000;
  GPIOD->AFR[1] &= 0xFF22FEEE; // PD12,13- TIM4_CH1, TIM4CH2
  GPIOD->AFR[1] |= 0x00220000;

  RCC->APB1ENR |= 0x00000004; // enable TIM4 clock
  RCC->APB1ENR |= 0x20000000;      // enable DAC clock
  TIM4->PSC = 0;            // No PSC
  TIM4->ARR = 4660;         // PPR(ich)" 90
  TIM4->CNT = 0;            // Clear counter
  TIM4->CCER &= 0xFFFFFFCC; // Positive polarity , Capture disable
  TIM4->CCMR1 = 0x00000101; // TIM4CHI ->TI1,TI_CH2 -> TI2 mapping

  TIM4->SMCR = 0x00000003; // 4-multiplication
  TIM4->CR1 = 0x0005;      // Counter enable
  PWM = 5400;              // initialize PWM = 5400 (duty ratio 50%)

  GPIOE->MODER &= 0xFFF0FFFF;  // PE11 = TIM1_CH2, PE10 = TIM1_CH2N
  GPIOE->MODER |= 0x000C0000;  //
  GPIOE->AFR[1] &= 0xFFFF00FF; // PE11, 10 = TIM1_CH2, TIM_CH2N
  GPIOE->AFR[1] |= 0x00001100;
  GPIOC->MODER |= 0x00000100;
  GPIOC->MODER &= 0xFFFFFBFF;
  GPIOC->ODR |= 0x00000010;

  RCC->APB2ENR |= 0x00000001; // enable TIM1 clock
  RCC->AHB1ENR |= 0x00000004; // port C clock enable(GPIOCEN = 1)

  TIM1->PSC = 0;     // 108MHz/(0+1) = 108MHz
  TIM1->ARR = 10799; // 108MHz / (10799+1) / 2 = 5kHz
  TIM1->CCR2 = PWM;
  TIM1->CNT = 0;            // clear counter
  TIM1->CCMR1 = 0x00006C00; // OC2M = 0110(PWM mode 1), CC2S = 00(output)
  TIM1->CCER = 0x00000050;  // CC2E = 1(enable OC2 output)
  TIM1->BDTR = 0x000080E2;  // MOE = 1, dead time = 10micro second
  TIM1->CR1 = 0x01A5;       // center-aligned, up-counter, enable TIM1, tdts=2tck_int
  TIM1->DIER = 0x0001;      // enable update interrupt

  
  GPIOC->BSRR = 0x00000010; // LED = VCC of HM-10
  //-----------UART7 start-----------------------------------------------------------------
  GPIOA->MODER &= 0xFFFCFFFF;  // PA8 = UART7_RX MCO1
  GPIOA->MODER |= 0x00020000;  // AF
  GPIOE->MODER &= 0xFFFCFFFF;  // PE8 = UART78_TX TP4
  GPIOE->MODER |= 0x00020000;  // AF
  GPIOE->AFR[1] &= 0xFFFFFFF0; // AFIO 8
  GPIOE->AFR[1] |= 0x00000008;
  GPIOA->AFR[1] = 0x0000000C; // AFIO 12
  RCC->APB1ENR |= 0x40000000; // enable UART7 clock

  UART7->CR1 = 0x0000000D; // TE=RE=UE = 1, 8 data bit, oversampling by 16
  UART7->CR2 = 0x00000000; // asynchronous mode, 1 stop bit
  UART7->CR3 = 0x00000000; // 3 sampling bitzd
  UART7->BRR = 5625;       // 9600 bps = 54MHz/5625
  Delay_ms(1);
  RXD = UART7->RDR;            // dummy read                            
  NVIC->ISER[2] |= 0x00040000; // enable (82)USART7 interrupt b18 on
   //---------------UART7 end----------------------------------------------------------------
  GPIOD->MODER &= 0xFFFFFFF0;
  GPIOD->MODER |= (1 << (2 * 0)); //      포트0 출력모드(초음파의 트리거 신호)   포트1 입력모드(에코신호를 키트의 입력으로 받음)
   //DAC conversion
  GPIOA->MODER |= 0x00000C00;      // PA5 = analog mode
  DAC->CR = 0x00030000;              // DAC channel 2 enable, output buffer disable
  
  

  count = 0; // initialize received data display
  Init_SysTick();
  int flag = 0;

  while (1)
  {
    calculate_pid();
    encodercount = TIM4->CNT;
    echo = echo_time();
    distance = 17 * echo / 100;
    if (distance > 300)
    {
      if (error * error < 400 && flag == 0 || flag ==2) 
      {
        targetcount = 3000;
        flag = 1;
      }

      else if (error * error < 400 && flag == 1|| flag ==2)
      {
        targetcount = 600;
        flag = 0;
      }
    }
    else
    {
      TX7_string("Detected!");
      PID_value = 0;
      TIM1->CCR2 = 5400;
      
      //LED_toggle();
      Delay_ms(100);
      flag =2;
      //targetcount = 600;
      V_DAC = distance;
      DAC->DHR12R2 = (unsigned int)(V_DAC*4095/300);

    }

    error = encodercount - targetcount;
    TIM1->CCR2 = 5400 - PID_value;
    LCD_command(0x86);
    LCD_unsigned_float(distance, 4, 1);
    LCD_command(0xC6);
    LCD_unsigned_float(encodercount, 4, 1);
    Delay_ms(100);
  }
}