/////////////////////////////////////////////////////////////
// HW1: ���� ������ �ð� ����
// ������: 2019132002 ������
// �ֿ� ���� �� ���� ���
// -���� �ֿ� ������ �ð��� ���� ������ �����Ͽ� ǥ��
// -�Է� ����ġ ���ÿ� ���� ���ø� �����Ͽ� ȭ�鿡 ǥ��
// -�ð� reset (clear) ����� ����
// - �ð� ���� �ӵ��������ϴ� ����� ����
// -���� �ð� reset
// -���� �ð� ���� �ӵ� ����
/////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER2_Init(void); // Timer2 �ʱ�ȭ
void TIMER6_Init(void); // Timer6 �ʱ�ȭ
void TIMER9_OC_Init(void); // Timer9 �ʱ�ȭ

void DisplayInitScreen(void); // �ʱ� ȭ��
void DisplayICNScreen(void); // ICN ��� ȭ��
void DisplayORDScreen(void); // ORD ��� ȭ��
void DisplayLHRScreen(void); // LHR ��� ȭ��

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint8_t SW0_Flag, SW1_Flag, SW2_Flag, SW6_Flag; // SW Flag

int main(void)
{
  _GPIO_Init();
  _EXTI_Init();
  LCD_Init();	
  DelayMS(10);
  DisplayInitScreen();	// LCD �ʱ�ȭ��
  GPIOG->ODR &= 0xFF00;// �ʱⰪ: LED0~7 Off

  TIMER2_Init(); // Timer2 �ʱ�ȭ ȣ��
  TIMER6_Init(); // Timer6 �ʱ�ȭ ȣ��
  TIMER9_OC_Init(); // Timer9 �ʱ�ȭ ȣ��
  
  /* �ʱ� sw flag ���� */
  SW0_Flag = 1;
  SW1_Flag = SW2_Flag = 0;
  SW6_Flag = 1;
  /* */
  
  while(1)
  {
    switch(KEY_Scan())
    {
       case SW0_PUSH: //SW0
         SW0_Flag = 1; // SW0 Flag 1
         SW1_Flag = SW2_Flag = 0; // SW1, SW2 Flag 0
         break;

       case SW1_PUSH: //SW1
         SW1_Flag = 1; // SW1 Flag 1
         SW0_Flag = SW2_Flag = 0; // SW0, SW2 Flag 0
         break;
    
       case SW2_PUSH: //SW2
         SW2_Flag = 1; // SW2 Flag 1
         SW0_Flag = SW1_Flag = 0; // SW0, SW1 Flag 0
         break;
    }
    
    if (SW0_Flag == 1) DisplayICNScreen(); // SW0 ������ ICN ȭ�� ���
    else if (SW1_Flag == 1) DisplayORDScreen(); // SW1 ������ ORD ȭ�� ���
    else if (SW2_Flag == 1) DisplayLHRScreen(); // SW2 ������ LHR ȭ�� ���
  }
}

/* Timer2 �ʱ�ȭ ���� */
void TIMER2_Init(void)
{
  RCC->APB1ENR |= 0x00000001;	// 0x0000 0001, RCC_APB1ENR TIMER2 Enable

  // Setting CR1 : 0x0000 
  TIM2->CR1 |= (1<<4);  // DIR=1(Down counter)(reset state)
  TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                      //  Counter Overflow/Underflow, 
                      //  Setting the UG bit Set,
                      //  Update Generation through the slave mode controller 
  TIM2->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                      //	Counter Overflow/Underflow, 
                      // Setting the UG bit Set,
                      //	Update Generation through the slave mode controller 
  TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  TIM2->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
  TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)


// Deciding the Period
  TIM2->PSC = 42-1;	// Prescaler 84,000,000Hz/42 =  2MHz (0.5us)  (1~65536)
  TIM2->ARR = 200000 ;	// Auto reload  0.5us*200000 = 100ms//  TIMER2 32bits
  
  // Clear the Counter
  TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                  // Re-initialize the counter(CNT=0) & generates an update of registers   

  // Setting an UI(UEV) Interrupt 
  NVIC->ISER[0] |= (1<<28); // Enable Timer2 global Interrupt
  TIM2->DIER |= (1<<0);	// Enable the Tim2 Update interrupt

  TIM2->CR1 |= (1<<0);	// Enable the Tim2 Counter (clock enable)
}
/* */

/* */
void TIMER6_Init(void)
{
  RCC->APB1ENR |= 0x00000010;	// 0x0000 0010, RCC_APB1ENR TIMER6 Enable

  // Setting CR1 : 0x0000 
  TIM6->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
  TIM6->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                      //  Counter Overflow/Underflow, 
                      //  Setting the UG bit Set,
                      //  Update Generation through the slave mode controller 
  TIM6->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                      //	Counter Overflow/Underflow, 
                      // Setting the UG bit Set,
                      //	Update Generation through the slave mode controller 
  TIM6->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM6->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  TIM6->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
  TIM6->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)


// Deciding the Period
  TIM6->PSC = 1680-1;	// Prescaler 84,000,000Hz/1680 =  50kHz (20us)  (1~65536)
  TIM6->ARR = 5000 - 1;	// Auto reload  20us*5000 = 100ms//  TIMER2 32bits
  
  // Clear the Counter
  TIM6->EGR |= (1<<0);	// UG(Update generation)=1 
                  // Re-initialize the counter(CNT=0) & generates an update of registers   

  // Setting an UI(UEV) Interrupt 
  NVIC->ISER[1] |= (1<<(54-32)); // Enable Timer2 global Interrupt  Timer
  TIM6->DIER |= (1<<0);	// Enable the Tim2 Update interrupt

  TIM6->CR1 |= (1<<0);	// Enable the Tim2 Counter (clock enable)
}
/* */

/* */
void TIMER9_OC_Init(void)
{
 RCC->APB2ENR |= (1<<16);   // 0x04, RCC_APB1ENR TIMER4 Enable

 // Setting CR1 : 0x0000 
 TIM9->CR1 &= ~(1<<4);   // DIR=0(Up counter)(reset state)
 TIM9->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
                          //  Counter Overflow/Underflow, 
                          //  Setting the UG bit Set,
                          //  Update Generation through the slave mode controller 
                          // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
 TIM9->CR1 &= ~(1<<2);   // URS=0(Update event source Selection): one of following events
                          //   Counter Overflow/Underflow, 
                          // Setting the UG bit Set,
                          //   Update Generation through the slave mode controller 
                          // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
 TIM9->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
 TIM9->CR1 |= (1<<7);   // ARPE=1(ARR is buffered): ARR Preload Enalbe 
 TIM9->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
 TIM9->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)

 // Setting the Period
 TIM9->PSC = 16800-1;   // Prescaler=16800, 168MHz/16800 = 10KHz (0.1ms)
 TIM9->ARR = 1000-1;   // Auto reload  : 0.1ms * 1K = 100ms(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����

 // Update(Clear) the Counter
 TIM9->EGR |= (1<<0);    // UG: Update generation    

  // Output Compare ����
 // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
 TIM9->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
 TIM9->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
 TIM9->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1�� �������� ���ο� ���� loading ����) 
 TIM9->CCMR1 |= (3<<4);   // OC1M=0b011 (Output Compare 1 Mode : toggle)
          // OC1REF toggles when CNT = CCR1
          
 // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
 TIM9->CCER &= ~(1<<0);   // CC1E=1: CC1 channel Output Enable
          // OC1(TIM9_CH1) Active: �ش���(100��)�� ���� ��ȣ���
 TIM9->CCER &= ~(1<<1);   // CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  

 // CC1I(CC ���ͷ�Ʈ) ���ͷ�Ʈ �߻��ð� �Ǵ� ��ȣ��ȭ(���)�ñ� ����: ��ȣ�� ����(phase) ����
 // ���ͷ�Ʈ �߻��ð�(10000 �޽�)�� 10%(1000) �ð����� compare match �߻�
 TIM9->CCR1 = 500;   // TIM9 CCR1 TIM9_Pulse

 TIM9->DIER |= (1<<0);   // UIE: Enable Tim9 Update interrupt
 TIM9->DIER |= (1<<1);   // CC1IE: Enable the Tim9 CC1 interrupt

 NVIC->ISER[0]    |= (1<<24);   // Enable Timer9 global Interrupt on NVIC

 TIM9->CR1 |= (1<<0);   // CEN: Enable the Tim9 Counter 
}
/* */

/* ICN �ð� ���� */
uint8_t ICN_min = 50, ICN_hour = 23; // MIN, HOUR ���� ����. �ʱ� 23 : 50
void TIM2_IRQHandler(void)
{
  TIM2->SR &= ~(1<<0);	// Interrupt flag Clear
  
  ICN_min++; // MIN ����
  if (ICN_min == 60) // 60MIN�� ����
  {
    ICN_min = 0; // 0MIN
    ICN_hour++; // 1HOUR ����
    if(ICN_hour == 24) ICN_hour = 0; // 24HOUR�� ���� 0HOUR
  } 
}

/* ORD �ð� ���� */
uint8_t  ORD_min = 50, ORD_hour = 9; // MIN, HOUR ���� ����. �ʱ� 09 : 50
void TIM6_DAC_IRQHandler(void)
{
  TIM6->SR &= ~(1<<0);	// Interrupt flag Clear

  ORD_min++; // MIN ����
  if (ORD_min == 60) // 60MIN�� ����
  {
    ORD_min = 0; // 0MIN
    ORD_hour++; // 1HOUR ����
    if(ORD_hour == 24) ORD_hour = 0; // 24HOUR�� ���� 0HOUR
  }
}

/* LHR �ð� ���� */
uint8_t  LHR_min = 50, LHR_hour = 15; // MIN, HOUR ���� ����. �ʱ� 15 : 50
void TIM1_BRK_TIM9_IRQHandler(void)
{
  if((TIM9->SR & 0x02) != RESET)   // Capture/Compare 1 interrupt flag
  {
    TIM9->SR &= ~(1<<1); // CC 1 Interrupt Claer
    
    LHR_min++;  // MIN ����
    if (LHR_min == 60) // 60MIN�� ����
    {
      LHR_min = 0; // 0MIN
      LHR_hour++; // 1HOUR ����
      if(LHR_hour == 24) LHR_hour = 0; // 24HOUR�� ���� 0HOUR
    }
  }
  
  else if ((TIM9->SR & 0x01) != RESET)   // Update interrupt flag
  {
    TIM9->SR &= ~(1<<0);   // Update Interrupt Claer
  }
}

void _GPIO_Init(void)
{
  // LED (GPIO G) ����
  RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
  GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
  GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
  GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

  // SW (GPIO H) ���� 
  RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
  GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
  GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

  // Buzzer (GPIO F) ���� 
  RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
  GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
  GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
  GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}	

/* SW7 (PH15/EXTI15) SW6 (PH14/EXTI14) */
void _EXTI_Init(void)
{
  RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
  
  GPIOH->MODER 	&= ~0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
  
  SYSCFG->EXTICR[3] |= 0x7700; 	// EXTI15, EXTICR14�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR4) (reset value: 0x0000)	
  
  EXTI->FTSR |= 0x00C000;		// Rising Trigger  Enable  (EXTI15:PH15, EXTI14:PH14) 
  EXTI->IMR  |= 0x00C000;  	// EXTI15, EXTI14 ���ͷ�Ʈ mask (Interrupt Enable)
          
  NVIC->ISER[1] |= (1<<(40-32));   	// Enable Interrupt EXTI15, EXTI14 Vector table Position ����
}

/* EXTI 14, 15 */
void EXTI15_10_IRQHandler(void)		// EXTI 15~10 ���ͷ�Ʈ �ڵ鷯
{
  /* SW7 */
  if(EXTI->PR & 0x8000) 		// EXTI15 nterrupt Pending?
  {
    EXTI->PR |= 0x8000; 	// Pending bit Clear
    
    if(SW0_Flag == 1) { //ICN ����� ���
      TIM2->CR1 &= ~ (1<<0); // Timer2 CR1 CEN 0 (Counter Disabled)
      GPIOG->ODR |= (1<<7); // LED7 ON
      BEEP();
      DelayMS(3000); // 3�� Delay
      GPIOG->ODR &= ~(1<<7); // LED7 OFF
      
      TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                                  // Re-initialize the counter(CNT=0) & generates an update of registers
      /* ICN �ʱ�ȭ */
      ICN_min = 0;
      ICN_hour = 0;
      
      /* ORD �ʱ�ȭ */
      ORD_hour = 10;
      ORD_min = 0;
      
      /* LHR �ʱ�ȭ */
      LHR_hour = 16;
      LHR_min = 0;
      
      /* ICN Mode Display */
      LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : Yellow
      LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
      char time[10];// time ��� �迭
      sprintf(time, "ICN %02d:%02d", ICN_hour, ICN_min);// ICN xx:xx
      LCD_DisplayText(1, 0, time); // Time Display
      /* */
      
      TIM2->CR1 |= (1<<0); // Timer2 CR1 CEN 1 (Counter Enabled)
    }
  }
  
  /* SW6 */
  if(EXTI->PR & 0x4000) 		// EXTI14 nterrupt Pending?
  {
    EXTI->PR |= 0x4000; 	// Pending bit Clear
    
    if(SW0_Flag == 1) { // ICN ����� ���
      if(SW6_Flag == 1) { // SW6 100ms -> 10ms ����
        TIM2->CR1 &= ~ (1<<0); // Timer2 CR1 CEN 0 (Counter Disabled)
        TIM6->CR1 &= ~ (1<<0); // Timer6 CR1 CEN 0 (Counter Disabled)
        TIM9->CR1 &= ~ (1<<0); // Timer9 CR1 CEN 0 (Counter Disabled)
        
        TIM2->ARR = 20000 - 1;	// Auto reload  0.5us*200000 = 10ms//  TIMER2 32bits
        TIM6->ARR = 500 - 1;	// Auto reload  20us*500 = 10ms//  TIMER2 32bits
        TIM9->ARR = 100 - 1;	// Auto reload  : 0.1ms * 100 = 10ms(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
        
        TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        TIM6->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        TIM9->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        
        /* period ���� ���� display*/
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayText(2, 14, "ms"); // ���� display
        LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
        LCD_DisplayText(2, 11, " 10"); //  10ms period
        /* */
        SW6_Flag = 0; //SW6 Flag 0
        
        TIM2->CR1 |= (1<<0); // Timer2 CR1 CEN 0 (Counter Disabled)
        TIM6->CR1 |= (1<<0); // Timer6 CR1 CEN 0 (Counter Disabled)
        TIM9->CR1 |= (1<<0); // Timer9 CR1 CEN 0 (Counter Disabled)// Re-initialize the counter(CNT=0) & generates an update of registers 
      }
      else if (SW6_Flag == 0) { // SW6 10ms -> 100ms ����
        TIM2->CR1 &= ~ (1<<0); // Timer2 CR1 CEN 0 (Counter Disabled)
        TIM6->CR1 &= ~ (1<<0); // Timer6 CR1 CEN 0 (Counter Disabled)
        TIM9->CR1 &= ~ (1<<0); // Timer9 CR1 CEN 0 (Counter Disabled)
        
        TIM2->ARR = 200000 - 1;	// Auto reload  0.5us*200000 = 10ms//  TIMER2 32bits
        TIM6->ARR = 5000 - 1;	// Auto reload  20us*500 = 10ms//  TIMER2 32bits
        TIM9->ARR = 1000 - 1;	// Auto reload  : 0.1ms * 100 = 10ms(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
        
        TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        TIM6->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        TIM9->EGR |= (1<<0);	// UG(Update generation)=1 
                                    // Re-initialize the counter(CNT=0) & generates an update of registers
        
        /* period ���� ���� display*/
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayText(2, 14, "ms"); // ���� display
        LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
        LCD_DisplayText(2, 11, "100"); // 100ms period
        /* */
        SW6_Flag = 1; // SW6 Flag 1
        
        TIM2->CR1 |= (1<<0); // Timer2 CR1 CEN 0 (Counter Disabled)
        TIM6->CR1 |= (1<<0); // Timer6 CR1 CEN 0 (Counter Disabled)
        TIM9->CR1 |= (1<<0); // Timer9 CR1 CEN 0 (Counter Disabled)// Re-initialize the counter(CNT=0) & generates an update of registers
      } 
    }
  }
}


void DisplayInitScreen(void)
{
  LCD_Clear(RGB_YELLOW);		// ȭ�� Ŭ����
  LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
  
  /* Init Display */
  LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : Yellow
  LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
  LCD_DisplayText(0, 0, "KJW 2019132002"); // �̸� display
  LCD_DisplayText(2, 0, "Int period"); // Period display
  LCD_DisplayText(2, 14, "ms"); // ���� display
  
  LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
  LCD_DisplayText(1, 0, "ICN 23:50"); //  ICN 23 : 60
  LCD_DisplayText(2, 11, "100"); // 100ms
  /* */
}

void DisplayICNScreen(void)
{
  /* ICN Clock Display */
  LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
  LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : Yellow  
  LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
  
  char time[10]; // time ��� �迭
  sprintf(time, "ICN %02d:%02d", ICN_hour, ICN_min); // ICN xx : xx
  LCD_DisplayText(1, 0, time); // Time Display
  /* */
}

void DisplayORDScreen(void)
{
  /* ORD Clock Display */
  LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
  LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : Yellow
  LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black

  char time[10]; // time ��� �迭
  sprintf(time, "ORD %02d:%02d", ORD_hour, ORD_min); // ORD xx : xx
  LCD_DisplayText(1, 0, time); // Time Display
  /* */
}

void DisplayLHRScreen(void)
{
  /* LHR�� �ð� ǥ�� */
  LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
  LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : Yellow
  LCD_SetTextColor(RGB_BLACK);	// ���ڹ��� : Black
  
  char time[10]; // time ��� �迭
  sprintf(time, "LHR %02d:%02d", LHR_hour, LHR_min); // LHR xx : xx
  LCD_DisplayText(1, 0, time); // Time Display
  /* */
}

void BEEP(void)			// Beep for 20 ms 
{ 	
  GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
  DelayMS(20);		// Delay 20 ms
  GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
    uint16_t key;
    key = GPIOH->IDR & 0xFF00;	// any key pressed ?
    if(key == 0xFF00)		// if no key, check key off
    {
      if(key_flag == 0)
        return key;
      else
      {	
        DelayMS(10);
        key_flag = 0;
        return key;
      }
    }
    else				// if key input, check continuous key
    {	
      if(key_flag != 0)	// if continuous key, treat as no key input
        return 0xFF00;
      else			// if new key,delay for debounce
      {
        key_flag = 1;
        DelayMS(10);
        return key;
      }
    }
}
