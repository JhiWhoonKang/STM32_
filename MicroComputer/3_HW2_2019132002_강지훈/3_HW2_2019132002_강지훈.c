/////////////////////////////////////////////////////////////
// HW2: STEP Motor 및 DC Motor 구동 드라이브 신호 발생
// 제출자: 2019132002 강지훈
// 주요 내용 및 구현 방법
// -위치 명령값 입력에 따른 Step Motor 구동 펄스 발생
// -Torque 명령값 입려겡 따른 DC Motor 구동 펄스 발생
// -오실로스코프나 멀티미터로 펄스를 확인하고 
// - DR이 변경될 때마다 부저 소리 변경
// -STEP Position 및 DC Torque 값을 LCD에 display
// -SW 및 Joystick 이용 펄스 변경
/////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"

uint16_t KEY_Scan(void);
uint16_t JOY_Scan(void);

void DisplayInitScreen(void); // 초기 화면
void TIMER1_Iint(void);
void TIMER3_Init(void);
void TIMER4_Init(void);
void TIMER5_Init(void);
void TIMER8_Init(void);
void TIMER14_Init(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int torque = 0, position = 0;
char torqueStr[12];
char Position;
char Torque;

int main()
{
          LCD_Init();
          
          TIMER1_Iint();
          TIMER4_Init();
          TIMER5_Init();

          TIMER8_Init();
          TIMER3_Init();
          TIMER14_Init();
          
          DelayMS(10);
          DisplayInitScreen();

          while(1)
          {
            
          }
}

void DisplayInitScreen(void)
{
          LCD_Clear(RGB_YELLOW);   // 화면 클리어
          LCD_SetFont(&Gulim8);      // 폰트 : 굴림 8

          /* Init Display */
          LCD_SetBackColor(RGB_BLACK);   // 글자배경색 : Black
          LCD_SetTextColor(RGB_WHITE);   // 글자색 : WHITE
          LCD_DisplayText(0, 0, "Motor Control"); 
          LCD_DisplayText(1, 0, "2019132002 KJW"); 

          LCD_SetBackColor(RGB_YELLOW); // 글자배경색 : Yellow
          LCD_SetTextColor(RGB_BLUE);   // 글자색 : BLUE
          LCD_DisplayText(2, 0, "Step Motor");
          LCD_DisplayText(4, 0, "DC Motor");

          LCD_SetTextColor(RGB_BLACK);   // 글자색 : BLACK
          LCD_DisplayText(3, 1, "Position:");
          LCD_DisplayText(5, 1, "Torque:");

          LCD_SetTextColor(RGB_RED);   // 글자색 : RED
          LCD_DisplayText(3, 10, "0");
          LCD_DisplayText(5, 8, "0");
          /* */
}

void TIMER1_Iint(void)
{
          // TIM1_CH3(PE13, 78)
          // Clock Enable : GPIOE & TIMER1
          RCC->AHB1ENR    |= (1<<4);            // GPIOE Enable
          RCC->APB2ENR    |= (1<<0);            // TIMER1 Enable 

          // PE13을 출력설정하고 Alternate function(TIM1_CH3)으로 사용 선언
          GPIOE->MODER    |= (2<<2*13);       // PE13 Output Alternate function mode               
          GPIOE->OSPEEDR |= (3<<2*13);       // PE13 Output speed 
          GPIOE->OTYPER   = 0x00000000;      // GPIOE PIN13 Output type push-pull (reset state)
          GPIOE->PUPDR     |= (1<<2*13);       // GPIOE PIN13 Pull-up
          GPIOE->AFR[1]    |= (1<<20);          // Connect TIM1 pins(PE13) to AF1(TIM1/TIM2)

          // PSC, ARR
          TIM1->PSC   = 8400-1;   // Prescaler 168,000,000Hz/8400 = 20,000Hz  (1~65536)
          TIM1->ARR   = 10000-1;   // Auto reload  (1 / 20,000 * 10,000 = 0.5s, 한 주기 = 1s)
          
          // Clear the Counter
          TIM1->EGR |= (1<<0);    // UG: Update generation 
          
          TIM1->CCER   &= ~(1<<8);   // OC3(TIM1_CH3) Active(Capture/Compare 3 output disable)
          TIM1->CCER   &= ~(1<<9);   // CC3P=0: CC3 output Polarity High (OC3으로 반전없이 출력)

          TIM1->CCR3   = 5000;      // TIM1_Pulse

          TIM1->BDTR |= (1<<15);  // main output enable

          TIM1->CCMR2    &= ~(3<<0);    // CC3S(CC3 channel)='0b00' : Output 
          TIM1->CCMR2    &= ~(1<<3);    // OC3PE=1: Output Compare 3 preload Enable
          TIM1->CCMR2   |= (3<<4);   // OC3M: Output Compare 3 Mode : toggle

          TIM1->CR1    &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
          TIM1->CR1    &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
          TIM1->CR1    |= (2<<5);    // CMS(Center-aligned mode Sel): Center-aligned mode2
          TIM1->CR1   &= ~(1<<7);   // ARPE: Auto-reload preload disable        

          TIM1->DIER |= (1<<3);   // CC3IE: Enable the Tim1 CC3 interrupt
          NVIC->ISER[0] |= (1<<27); // TIM1_CC
          
          TIM1->CR1 &= ~(1<<0);   // CEN: Disable the Tim1 Counter 
}

uint8_t steppulse = 0;
void TIM1_CC_IRQHandler(void)
{
         if ((TIM1->SR & 0x08) != RESET)   // CC3 interrupt flag 
         {
                      TIM1->SR &= ~0x08;   // CC3 Interrupt Claer
                      steppulse++; // pulse 출력 count
                      if (steppulse >= TIM5->CNT*2 )  // 위치명령값의 2배 값에 해당하는 펄스 출력 완료 시
                      {
                                      steppulse = 0; // 출력 펄스 초기화
                                      TIM1->CCER &= ~(1<<8); // OC3(TIM1_CH3) Active(Capture/Compare 3 output disable)
                                      TIM1->CR1   &= ~(1<<0); // TIM1 Disable
                      }
         }
}

void TIMER4_Init(void)
{  
        // Clock Enable : TIMER4
        RCC->APB1ENR |= (1<<2);   // RCC_APB1ENR TIMER4 Enable
        
        // Time base Mode
       TIM4->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
       TIM4->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled)
       TIM4->CR1 &= ~(1<<2);   // URS=0(Update Request Source  Selection)
       TIM4->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
       TIM4->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
       TIM4->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
       TIM4->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)

        // PSC, ARR
       TIM4->PSC = 8400-1;   // Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
       TIM4->ARR = 1000-1;   // Auto reload  0.1ms * 1000 = 100ms

        // Clear the Counter
       TIM4->EGR |= (1<<0);   // UG(Update generation)=1 

       // Setting an UI(UEV) Interrupt 
       NVIC->ISER[0] |= (1<<30); // Enable Timer4 global Interrupt
       TIM4->DIER |= (1<<0);   // Enable the Tim4 Update interrupt

       TIM4->CR1 |= (1<<0);   // Enable the Tim4 Counter (clock enable)     
}

uint8_t oldcmd4 = 0;
uint8_t newcmd4 = 0;
void TIM4_IRQHandler(void)
{            
          TIM4->SR &= ~(1<<0);   // Interrupt flag Clear
          Position = '0' + TIM5->CNT;
          LCD_DisplayChar(3, 10, Position); // TIM5->CNT값을 읽어 LCD에 위치(position) 명령값으로 표시
          
          newcmd4 = TIM5->CNT; // 현재 위치 명령값 저장
          if(newcmd4 != 0) // 명령값이 0이 아닌 경우에만
          {
                if (newcmd4 != oldcmd4) // 명령값이 변경됐을 경우만 실행
                {
                          oldcmd4 = TIM5->CNT; // 과거 위치 명령값 업데이트
                          TIM1->CCER |= (1<<8); // OC3(TIM1_CH3) Active(Capture/Compare 3 output enable)
                          TIM1->CR1 |= (1<<0); // CEN: Enable the Tim1 Counter                     
                }
          }          
}

void TIMER5_Init(void)
{
          //SW3(PH11/TIM5_CH2)
          // Clock Enable : GPIOH & TIMER5
          RCC->AHB1ENR   |= (1<<7);   // GPIOH Enable
          RCC->APB1ENR    |= (1<<3);   // TIMER5 Enable 
          
          // PH11을 입력설정하고 Alternate function(TIM5_CH2)으로 사용 선언
          GPIOH->MODER    |= (2<<2*11);   // GPIOH PIN11 Output Alternate function mode                
          GPIOH->OSPEEDR    |= (2<<2*11);   // GPIOH PIN11 Output speed
          GPIOH->PUPDR   &= ~(3<<2*11);    // GPIOH PIN11 NO Pull-up
          GPIOH->AFR[1]   |= (2<<12);   // Connect TIM4 pins(PH11) to AF2(TIM3..5)

          // Time base Mode
          TIM5->CR1 &= ~(1<<4);    // DIR=0(Up counter)(reset state)
          TIM5->CR1 &= ~(1<<1);    // UDIS=0(Update event Enabled)
          TIM5->CR1 &= ~(1<<2);    // URS=0(Update event source Selection)
          TIM5->CR1 &= ~(1<<3);    // OPM=0(The counter is NOT stopped at update event) (reset state)
          TIM5->CR1 |=    (1<<7);    // ARPE=1(ARR Preload Enable)
          TIM5->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
          TIM5->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)

          // PSC, ARR
          TIM5->PSC = 1-1;   // Prescaler=1
          TIM5->ARR = 4;   // Auto reload  :  count값 범위: 0~4
          
          // Clear the Counter
          TIM5->EGR |= (1<<0);    // UG=1, REG's Update (CNT clear) 

          // External Clock Mode 1
          // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
          TIM5->CCMR1 |= (1<<8);    // CC2S(CC2 channel) = '0b01' : Input 
          TIM5->CCMR1 &= ~(15<<12); // IC2F='0b0000: No Input Filter 

          // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
          TIM5->CCER &= ~(1<<4);   // CC2E=0: Capture Disable
          // TI1FP1 NonInverting / Rising Edge  
          TIM5->CCER |= (1<<5);   // CC2P=01 
          TIM5->CCER &= ~(1<<7);   // CC2NP=0     

          // SMCR(Slave Mode Control Reg.) : External Clock Enable
          TIM5->SMCR |= (6<<4);   // TS(Trigger Selection)=0b101 :TI1FP1(Filtered Timer Input 1 출력신호)
          TIM5->SMCR |= (7<<0);   // SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

          TIM5->CR1 |= (1<<0);   // CEN: Enable the Tim4 Counter        
}

/*

*/

void TIMER8_Init(void)
{
          // PI5: TIM8_CH1
          // Clock Enable : GPIOI & TIMER4
          RCC->AHB1ENR   |= (1<<8);        // RCC_AHB1ENR GPIOI Enable
          RCC->APB2ENR   |= (1<<1);    // 0x02, TIMER8 Enable 
          
          // PI5를 출력설정하고 Alternate function(TIM8_CH1)으로 사용 선언
          GPIOI->MODER   |= (2<<2*5);      // GPIOI PIN5 Alternate function mode
          GPIOI->OSPEEDR |= (2<<2*5);      // GPIOI PIN5 Output speed 50MHz Fast speed
          GPIOI->PUPDR   |= (1<<2*5);      // GPIOI PIN5 Pull-up
          GPIOI->AFR[0]  |= (3<<4*5);      // Connect TIM8 pins(PI5) to AF3(TIM8)

          // Timerbase Mode
          TIM8->CR1 &= ~(1<<4);   // DIR=0(Up counter)(reset state)
          TIM8->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled)
          TIM8->CR1 &= ~(1<<2);   // URS=0(Update event source Selection)
          TIM8->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
          TIM8->CR1 |=  (1<<7);   // ARPE=1(ARR Preload Enable)
          TIM8->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
          TIM4->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=11 (Center-aligned mode3) (reset state)

          TIM8->PSC = 1-1;        // Prescaler=1
          TIM8->ARR = 7;          // Auto reload  :  count값 범위: 0~7

          TIM8->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
          TIM8->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
          TIM8->CR1 |= (3<<5);    // CMS(Center-aligned mode Sel): No(reset state)

          TIM8->EGR |= (1<<0);    // UG: Update generation 

          TIM8->CCMR1 |= (1<<0);    // CC1S(CC1 channel) = '0b01' : Input 
          TIM8->CCMR1 &= ~(15<<4); // IC1F='0b0000: No Input Filter 

          // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
          TIM8->CCER &= ~(1<<0);   // CC1E=0: Capture Disable
          // TI1FP1 NonInverting / Rising Edge  
          TIM8->CCER &= ~(1<<1);   // CC1P=0 
          TIM8->CCER &= ~(1<<3);   // CC1NP=0   

          // SMCR(Slave Mode Control Reg.) : External Clock Enable
          TIM8->SMCR |= (5<<4);   // TS(Trigger Selection)=0b101 :TI1FP1(Filtered Timer Input 1 출력신호)
          TIM8->SMCR |= (7<<0);   // SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

          TIM8->CR1 |= (1<<0);   // CEN: Enable the Tim4 Counter     
}

void TIMER3_Init(void)
{
          RCC->APB1ENR |= 0x02;   // RCC_APB1ENR TIMER3 Enable

          TIM3->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
          TIM3->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled)
          TIM3->CR1 &= ~(1<<2);   // URS=0(Update Request Source  Selection)
          TIM3->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
          TIM3->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
          TIM3->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
          TIM3->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)


           // Deciding the Period
          TIM3->PSC = 8400-1;   // Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
          TIM3->ARR = 200-1;   // Auto reload  0.1ms * 200 = 20ms

          // Clear the Counter
          TIM3->EGR |= (1<<0);   // UG(Update generation)=1 

          // Setting an UI(UEV) Interrupt 
          NVIC->ISER[0] |= (1<<29); // Enable Timer3 global Interrupt
          TIM3->DIER |= (1<<0);   // Enable the Tim3 Update interrupt
            
          TIM3->CR1 |= (1<<0);   // Enable the Tim3 Counter (clock enable)   
}

void TIM3_IRQHandler(void)     // 1ms Interrupt
{
          TIM3->SR &= ~(1<<0);   // Interrupt flag Clear
          torque=TIM8->CNT;
          sprintf(torqueStr, "%d", torque);
          LCD_DisplayText(5, 8, torqueStr); //NAVI_PUSH 횟수 출력
          
          TIM14->CCR1 = (8 * (10 - torque) * 10 / 100); // Duty 결정
}

void TIMER14_Init(void)
{
          // TIM14_CH1/PF9/27
          // Clock Enable : GPIOF & TIMER14
          RCC->AHB1ENR      |= (1<<5);   // GPIOF Enable
          RCC->APB1ENR      |= (1<<8);   // TIMER14 Enable 
                        
          // PF9을 출력설정하고 Alternate function(TIM14_CH1)으로 사용 선언 : PWM 출력
          GPIOF->MODER       |= (2<<9*2);   // PF9 Output Alternate function mode               
          GPIOF->OSPEEDR    |= (3<<9*2);   // PF9 Output speed (100MHz High speed)
          GPIOF->OTYPER      &= ~(1<<9);   // PF9 Output type push-pull (reset state)
          GPIOF->AFR[1]       |= (9<<4);    // Connect TIM14 pins(PF9) to AF9(TIM12..14)

          // PSC, ARR
          TIM14->PSC   = 420-1;   // Prescaler 84,000,000Hz/420 = 20,000 Hz(50us)  (1~65536)
          TIM14->ARR   = 8-1;   // Auto reload  (50us * 8 = 400us : PWM Period)

          TIM14->CCER   |= (1<<0);   // CC1E=1: OC1(TIM14_CH1) Active(Capture/Compare 1 output enable)
          TIM14->CCER   &= ~(1<<1);   // CC1P=0: CC1 output Polarity High (OC1으로 반전없이 출력)

          // Duty Ratio 
          TIM14->CCR1   = 0;      // CCR1 value, 초기 명령값: 0(DR=0, motor stop)

          TIM14->CCMR1    &= ~(3<<0);    // CC1S(CC1 channel)='0b00' : Output 
          TIM14->CCMR1    |= (1<<3);    // OC1PE=1: Output Compare 1 preload Enable
          TIM14->CCMR1   |= (7<<4);   // OC1M: Output compare 1 mode: PWM 2 mode
          TIM14->CCMR1   |= (1<<7);   // OC1CE: Output compare 1 Clear enable

          // CR1 : Up counting & Counter TIM5 enable
          TIM14->CR1    &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
          TIM14->CR1    &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
          TIM14->CR1    &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)
          TIM14->CR1   |= (1<<7);   // ARPE: Auto-reload preload enable

          TIM14->CR1   |= (1<<0);   // CEN: Counter TIM14 enable
}

void DelayMS(unsigned short wMS)
{
          register unsigned short i;
          for (i=0; i<wMS; i++)
          DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
        for(; Dly; Dly--);
}