/////////////////////////////////////////////////////////////
// PR1: Car Tracking system
// ������: 2019132002 ������
// �ֿ䳻��
//-�Ÿ�����1:������ŰƮ���ǰ�������,PA1(ADC3_IN1)
//-�Ÿ�����2: 40pin Box header connector�� ����� �ܺ� ��������, PF3(ADC3_IN9)
//- ����������:PB7(TIM4_CH2(PWMmode)), LED�� PWM ��ȭȮ��
//* LED: 40pin Box header connector�� �߰� ������ �ܺ� LED
//- �������ڵ�:PF9(TIM14_CH1(PWM mode)), Buzzer�� PWM ��ȭȮ��
//- Off-line ������ �õ�: Move-key(SW4(EXTI12)), Stop-key(SW6(EXTI14))
 //- ����(PC������α׷�)�������õ�: Move-key(��M��), Stop-key(��S��)
 //- �Ÿ���ǥ��:GLCD(D),PC
 //- �������ӵ�ǥ��:GLCD(DR)
 //- ����������ǥ��:GLCD(��L��,��R��, ��F��)
/////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void DispayTitle(void);        //LCD Display �Լ�
void _GPIO_Init(void);        //GPIO �ʱ�ȭ �Լ�
void _EXTI_Init(void);        //EXTI �ʱ�ȭ �Լ�
void TIMER1_OC_Init(void);        //TIM1 �ʱ�ȭ �Լ�
void TIMER4_PWM_Init(void);        //TIM4 �ʱ�ȭ �Լ�
void TIMER14_PWM_Init(void);        //TIM14 �ʱ�ȭ �Լ�
void _ADC_Init(void);        //ADC �ʱ�ȭ �Լ�
void DMAInit(void);        //DMA �ʱ�ȭ �Լ�
void USART1_Init(void);        //USART �ʱ�ȭ �Լ�
void USART_BRR_Configuration(uint32_t USART_BaudRate);        //USART BRR �ʱ�ȭ �Լ�
void SerialSendChar(uint8_t c);        //�ѹ��� ������ �Լ�
void SerialSendString(char *s);        //���� ���� ������ �Լ�
void CARSTATUS_Init(void);      // ���� �ʱ� ���� �ʱ�ȭ �Լ�
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
uint16_t KEY_Scan(void);

char str[20];
int send_ch= 0x30;
int No_send_data;
uint16_t ADC_value[2];                                       // ADC  ��ȯ�� ����

typedef struct  {
  float voltage;
  uint16_t distance;
}ADAS;
ADAS internal = {0, 0};    // ŰƮ  ���� ������ ����
ADAS external = {0, 0};    // �ܺ� ���� ������ ����

typedef struct {
  char drivemode;
  uint8_t distanceDR;
  char handle;
  uint8_t motorDR;
}CARSTATUS;
CARSTATUS carstatus;

/* ���� �ʱ� �� ���� */
void CARSTATUS_Init(void)
{
	carstatus.drivemode='S';
	carstatus.distanceDR = 0;
        carstatus.handle='F';
        carstatus.motorDR=0;
}

/* SW4 & 6 EXTI */
void EXTI15_10_IRQHandler()
{
	if(EXTI->PR & 0x004000)			// SW4
	{
		EXTI->PR |= 0x004000;		// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
		carstatus.drivemode = 'M'; // ���� ���: MOVE
                LCD_DisplayChar(1,18,carstatus.drivemode); // LCD, FRAM ������Ʈ
                Fram_Write(1201,carstatus.drivemode);          
	}

	else if(EXTI->PR & 0x001000)		// SW6
	{
		EXTI->PR |= 0x001000;		// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
                carstatus.drivemode = 'S'; // ���� ���: STOP
                LCD_DisplayChar(1,18,carstatus.drivemode); // LCD, FRAM ������Ʈ
                Fram_Write(1201,carstatus.drivemode);          
	}
}

void ADC_IRQHandler(void)
{
	ADC3->SR &= ~(1<<1);		// EOC flag clear
        
        /* sensor equation */
        // 3.3 : 1023 =  Volatge : ADC_Value 
        // internal sensor distance(int) = voltage * 5 + 3
        // external sensor distance(int) = voltage
        // �Ÿ�->DR ��ȯ ���;  (distance-1)/2*10
        // D1 ���� ����: �ּ� 3, �ִ� 19
        // D2 ���� ����:: �ּ� 0, �ִ� 3
        
        /* voltage */
        internal.voltage = (ADC_value[0] * 3.3) / 1023;
        external.voltage = (ADC_value[1] * 3.3) / 1023; 
        
        /* distance */
        internal.distance = (int)(internal.voltage * 5 + 3);
        external.distance = (int)(external.voltage);
        
        /* drivemode: STOP -> Distance 0 */
        if (carstatus.drivemode == 'S')
        {
          internal.distance = 0;
          external.distance = 1;
        }
        
        /* �Ÿ� ���� 10 �̻� -> 10, 1�� �ڸ� �����Ͽ� ���� */
        if (internal.distance > 9)
        {
          SerialSendChar((internal.distance/10) + 0x30);
          LCD_DisplayChar(2,17,(internal.distance/10) + 0x30);
        }
        
        else
        {
          LCD_DisplayChar(2,17,' ');
        }
        
        /* distance -> duty ratio */
        carstatus.distanceDR = ((internal.distance -1 ) / 2 ) * 10;
        TIM4->CCR2	= 50000 / 100 *  carstatus.distanceDR;          //Duty Ratio �ݿ�
        LCD_DisplayChar(4,7, (carstatus.distanceDR/10) + 0x30);
        LCD_DisplayChar(4,8, (carstatus.distanceDR%10) + 0x30);
        /* */
        
        SerialSendChar((internal.distance%10) + 0x30);
        SerialSendChar('m');
        SerialSendChar(' ');
        LCD_DisplayChar(2,18,(internal.distance%10) + 0x30);
        /* */
            
        /* ����� ���� */
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect(29, 27, 100, 10);
        /* �ֽ�ȭ */
        LCD_SetBrushColor(RGB_RED);
        LCD_DrawFillRect(28, 27, (int)((float)internal.distance/19*100), 10);
        /* */
       
        // INTERNAL
        // #########################################################
        // EXTERNAL
        
        if(external.distance==0)
        {
                  carstatus.motorDR = 30;
                  carstatus.handle = 'R';
        }
        else if(external.distance==1)
        {
                  carstatus.motorDR = 0;
                  carstatus.handle = 'F';
        }
        else
        {
                  carstatus.motorDR = 90;    
                  carstatus.handle = 'L';
        }
        LCD_DisplayChar(3,18,(external.distance%10) + 0x30);
        LCD_DisplayChar(4,18, carstatus.handle);
        
        TIM14->CCR1 = 160 / 100 * carstatus.motorDR;          //Duty Ratio �ݿ�       
    
        /* ����� ���� */
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect(29, 40, 100, 10);
        /* �ֽ�ȭ */
        LCD_SetBrushColor(RGB_GREEN);
        LCD_DrawFillRect(28, 40, (int)((float)external.distance/3*100), 10);
        /* */
}

void USART1_IRQHandler(void)	
{       
	if ( (USART1->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
	{
		char ch;
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
		
                if(ch=='M')     //M ������ MOVE ���
                {
                  carstatus.drivemode = 'M';
                }
                
                else if(ch=='S')        //S ������ STOP ���
                {
                  carstatus.drivemode = 'S';
                }
                //�ֽ�ȭ
                Fram_Write(1201,carstatus.drivemode);
                LCD_DisplayChar(1,18,carstatus.drivemode);
        }
} 

int main(void)
{
	LCD_Init();	// LCD ���� �Լ�
	DelayMS(10);	// LCD���� ������
        _GPIO_Init();
        _EXTI_Init();
        TIMER1_OC_Init();
        TIMER4_PWM_Init();
        TIMER14_PWM_Init();
        _ADC_Init();
        DMAInit();        
        USART1_Init();
        Fram_Init();                    // FRAM �ʱ�ȭ H/W �ʱ�ȭ
        Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
        CARSTATUS_Init();
	GPIOG->ODR &= 0x00;	// LED0~7 Off 
        
        /* FRAM 1201�����κ��� ���� READ */
        char FramStatus = Fram_Read(1201);
        if (FramStatus  == 'M' || FramStatus  == 'S') // M �Ǵ� S ���� ����Ǿ� �ִٸ� �ֽ�ȭ
        {
          	carstatus.drivemode = FramStatus; 
        }
        else // M �Ǵ� S�� ���� ����Ǿ� ���� �ʴٸ� �ʱ���� STOP
        {
          	Fram_Write(1201,'S'); // FRAM �ֽ�ȭ
                carstatus.drivemode = 'S'; // ���� ���� ������Ʈ
        }
        
        DispayTitle();	//LCD �ʱ�ȭ�鱸�� �Լ�
    
	while(1) 
        {
          
        }
}

/* LCD ��� ���빰 */
void DispayTitle(void)
{	
	LCD_Clear(RGB_WHITE);                                   // ȭ�� Ŭ����
	LCD_SetFont(&Gulim8);                                   // ��Ʈ : ���� 8
	LCD_SetBackColor(RGB_WHITE);	                        // ���ڹ��� : White
	LCD_SetTextColor(RGB_BLACK);                                    // ���ڻ� : Black     
	LCD_DisplayText(0,0,"KJW 2019132002");
        LCD_DisplayText(1,0, "Tracking Car");
        LCD_DisplayText(2,0,"D1:");
	LCD_DisplayText(3,0,"D2:");
        LCD_DisplayText(4,0,"SP(DR):  %DIR(DR):");
  
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : Blue 
        LCD_DisplayChar(1,18,carstatus.drivemode);
        LCD_DisplayText(4,7,"00");
        LCD_DisplayChar(4,18,carstatus.handle);
}

/* GPIO �ʱ� ���� */
void _GPIO_Init(void)
{
	// LED (GPIO G) ����
	RCC->AHB1ENR	|=  RCC_AHB1ENR_GPIOGEN;	        // RCC_AHB1ENR GPIOG Enable
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	        // GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
        
         // SW (GPIO H) ���� 
        RCC->AHB1ENR    |=  RCC_AHB1ENR_GPIOHEN;           // RCC_AHB1ENR GPIOH Enable                     
        GPIOH->MODER    &= ~0xFFFF0000;         // GPIOH 8~15 : Input mode (reset state)            
        GPIOH->PUPDR    &= ~0xFFFF0000;         // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
}

/* EXTI (EXTI12(GPIOH.12, SW4), EXTI14(GPIOH.14, SW6)) �ʱ� ����  */
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOHEN;               // RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= RCC_APB2ENR_SYSCFGEN;              // RCC_APB2ENR  SYSCFG Enable
	
	GPIOH->MODER 	&= ~0xFFFF0000;    // GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[3] |= 0x0707;            // EXTI12,14�� ���� �ҽ� �Է��� GPIOH�� ����	
	EXTI->RTSR |= 0x005000;		      // EXTI12,14: Rising Trigger  Enable 
	EXTI->IMR  |= 0x005000;		              // EXTI12,14 ���ͷ�Ʈ mask (Interrupt Enable) ����		
	NVIC->ISER[1] |= (1<<(40 -32) );	// Enable 'Global Interrupt EXTI12,14'
}

/* TIM1_CH3 */
void TIMER1_OC_Init(void)
{
	RCC->APB2ENR   |= 0x01;// RCC_APB2ENR TIMER1 Enable

	TIM1->PSC = 16800-1;   // Prescaler 168,000,000Hz/16800= 10kHz (0.1ms)

	TIM1->ARR = 4000-1;    // 400m/0.1m = 4000
    
	TIM1->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
        TIM1->CR1 &= ~(1<<1);     // UDIS=0 (Update event Enabled)
        TIM1->CR1 &= ~(1<<2);       // URS = 0
        TIM1->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
        TIM1->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
        TIM1->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
        TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
    
        TIM1->EGR |= (1<<0);    // UG: Update generation    
    
        TIM1->CCER |= (1<<4*(3-1));   // CC3E=1: CC3 channel Output Enable
        TIM1->CCER &= ~(3<<(4*(3-1)+1));   // CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC2���� �������� ���)  
        
        TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channel) = '0b00' : Output 
        TIM1->CCMR2 &= ~(1<<3); // OC3P=0: Output Compare 3 preload disable
        TIM1->CCMR2 |= (3<<4);   // OC2M=0b011: Output Compare 3 Mode : toggle
        
        TIM1->CCR3 = 100;   // TIM1 CCR2 TIM1_Pulse
    
        TIM1->BDTR |= (1<<15);  // main output enable
      
        TIM1->CR1 |= (1<<0);   // CEN: Enable the Tim5 Counter   
}

/* TIM4_CH2(PB7) */
void TIMER4_PWM_Init(void)
{   
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOBEN;	// RCC_AHB1ENR GPIOB CLOCK Enable
	RCC->APB1ENR 	|= RCC_APB1ENR_TIM4EN;	// RCC_APB1ENR TIMER4 CLOCK Enable 
    						
	GPIOB->MODER 	|= (2<<2*7);	// 0x00020000 PB7 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<2*7);	// 0x00030000 PB7 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<7);	// PB7 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<2*7);	// 0x00010000 PB7 Pull-up
 	GPIOB->AFR[0]	|= (2<<4*7);	// 0x00000002 (AFR[0].(31~28)=0b0010): Connect TIM4 pins(PB7) to AF2(TIM3..5)
    
	TIM4->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 50000-1;	// Auto reload  (0.1ms * 50000= 5s : PWM Period)
        TIM4->CCR2	= 100;		// CCR2 value
        
	TIM4->CCER	|= (1<<4);	// CC2E=1: OC2(TIM4_CH2) Active(Capture/Compare 2 output enable)

	TIM4->CCMR1 &= ~(3<<8); // CC2S(CC3 channel)= '0b00' : Output 
	TIM4->CCMR1 |= (1<<11); 	// OC2PE=1: Output Compare 2 preload Enable
	TIM4->CCMR1	|= (6<<12);	// OC2M=0b110: Output compare 2 mode: PWM 1 mode
	TIM4->CCMR1	|= (1<<15);	// OC2CE=1: Output compare 2 Clear enable
	
	TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
}

/* TIM14_CH1(PF9, 27) */
void TIMER14_PWM_Init(void)
{
	RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOFEN;   // RCC_AHB1ENR GPIOF Enable
        RCC->APB1ENR   |= RCC_APB1ENR_TIM14EN;// RCC_APB1ENR TIMER14 Enable
        
	GPIOF->MODER   |= (2<<2*9);   // GPIOF PF9 Output Alternate function mode               
	GPIOF->OSPEEDR |= (3<<2*9);   // GPIOF PF9 Output speed (100MHz High speed)
	GPIOF->OTYPER  &= ~(1<<9);   // GPIOF PF9 Output type push-pull (reset state)
	GPIOF->AFR[1]  |= (9<<4*(9-8)); // Connect TIM14 pins(PF9) to AF9
        
	TIM14->PSC = 420-1;   // Prescaler 168,000,000Hz/420= 400kHz (2.5us)
	TIM14->ARR = 160-1;    // �ֱ� = 2.5us * 160 = 400us
        TIM14->CCR1 =100;   // TIM14_Pulse
    
	TIM14->CCER	|= (1<<0);	// CC1E=1: OC1(TIM14_CH1) Active(Capture/Compare 1 output enable)
	TIM14->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1���� �������� ���)
    
	TIM14->CCMR1 &= ~(3<<0); 	// CC1S(CC1 channel)='0b00' : Output 
        TIM14->CCMR1 |=    (1<<2); 	// CC1FE =1: Fastmode Enable 
	TIM14->CCMR1 |=    (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable

	TIM14->CCMR1 |=    (6<<4);	// OC1M: Output compare 1 mode: PWM 1 mode
	TIM14->CCMR1 |=    (1<<7);	// OC1CE: Output compare 1 Clear enable

	TIM14->CR1	    |=    (1<<0);	// CEN: Counter TIM14 enable
}

/* ADC �ʱ� ���� */
void _ADC_Init(void)
{	
	// ADC3_IN1: PA1(pin 41) -> ŰƮ ��������
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// RCC_AHB1ENR  GPIOA Enable
	GPIOA->MODER |= (3<<2*1);		                    // CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
    
        // ADC3_IN9: PF3(pin ) -> �ܺ� ��������
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;	// RCC_AHB1ENR   GPIOF Enable
	GPIOF->MODER |= (3<<2*3);		                    // CONFIG GPIOF PIN(PF3) TO ANALOG IN MODE
						
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;	// RCC_APB2ENR ADC3 Enable

	ADC->CCR &= ~(0X1F<<0);		          // MULTI[4:0]: Independent mode
	ADC->CCR |=  (1<<16); 		          // ADCPRE: PCLK2 divided by 4
       
	ADC3->CR1 |= (1<<24);		           // RES[1:0]:10bit Resolution
        ADC3->CR1 |= (1<<8);                                     // SCAN: Scan mode enabled
        
        ADC3->CR2 &= ~(1<<1);                                  // CONT: Single conversion mode
        ADC3->CR2 |= (3<<28);		                           // EXTEN: Trigger detection on both the rising and falling edges
        ADC3->CR2 |=   (2<<24);                                 // EXTSEL: Timer1 CC3 event	
	ADC3->CR2 &= ~(1<<10);		// EOCS: The EOC bit is set at the end of each sequence of regular conversions

	ADC3->SQR1 |= (1<<20);	// L[3:0]=0b0010: ADC Regular channel sequece length     
        
        // ŰƮ ��������
	ADC3->SMPR2 |= (0x7<<(3*1));	// ADC3_CH1 Sample TIme_480Cycles (3*Channel_1)
	ADC3->SQR3 |= (1<<0);		// SQ1[4:0]=0b00001 : CH1
    
        // �ܺ� ��������
        ADC3->SMPR2 |= (0x7<<(3*9));	// ADC3_CH9 Sample TIme_480Cycles (3*Channel_9)
        ADC3->SQR3 |= (9<<5);		// SQ1[4:0]=0b01001 : CH9
        
        // ADC interrupt����
        ADC3->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
        NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
    
        // DMA����
        ADC3->CR2 |= 0x00000200;	// DMA requests are issued as long as data are converted and DMA=1	
        ADC3->CR2 |= 0x00000100;	// DMA mode enabled  (DMA=1)    
	ADC3->CR2 |= (1<<0);		// ADON: ADC ON
        ADC3->CR2 |= 0x40000000;   // SWSTART=1
}

/* DMA �ʱ� ���� */
void DMAInit(void)
{
 	//ADC3=>DMA2 STREAM0 CH2
        // DMA2 Stream0 channel2 configuration *************************************
	RCC->AHB1ENR |= (1<<22);		//DMA2 clock enable
	DMA2_Stream0->CR |= (2<<25);	//DMA2 Stream0 channel 2 selected

	// ADC1->DR(Peripheral) ==> ADC_vlaue(Memory)
	DMA2_Stream0->PAR |= (uint32_t)&ADC3->DR;	   //Peripheral address - ADC3->DR(Regular data) Address
	DMA2_Stream0->M0AR |= (uint32_t)&ADC_value; //Memory address - ADC_Value address 
	DMA2_Stream0->CR &= ~(3<<6);		  //Data transfer direction : Peripheral-to-memory (P=>M)
	DMA2_Stream0->NDTR = 2;			  //DMA_BufferSize = 2 (ADC_Value[2])

	DMA2_Stream0->CR &= ~(1<<9); 	//Peripheral increment mode  - Peripheral address pointer is fixed
	DMA2_Stream0->CR |= (1<<10);	//Memory increment mode - Memory address pointer is incremented after each data transferd 
	DMA2_Stream0->CR |= (1<<11);	//Peripheral data size - halfword(16bit)
	DMA2_Stream0->CR |= (1<<13);	//Memory data size - halfword(16bit)   
	DMA2_Stream0->CR |= (1<<8);	//Circular mode enabled   
	DMA2_Stream0->CR |= (2<<16);	//Priority level - High

	DMA2_Stream0->FCR &= ~(1<<2);	//DMA_FIFO_direct mode enabled
	DMA2_Stream0->FCR |= (1<<0);	//DMA_FIFO Threshold_HalfFull , Not used in direct mode

	DMA2_Stream0->CR &= ~(3<<23);	//Memory burst transfer configuration - single transfer
	DMA2_Stream0->CR &= ~(3<<21);	//Peripheral burst transfer configuration - single transfer  
	DMA2_Stream0->CR |= (1<<0);	//DMA2_Stream0 enabled
}

/* USART �ʱ� ���� */
void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity

	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable

	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)

	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
	{                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else 			// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
	while((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���

	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
		else
		{		DelayMS(10);
        		key_flag = 0;
        		return key;
		}
	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
			return 0xFF00;
		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
		}
	}
}

void DelayMS(unsigned short wMS)
{	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);  // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{	volatile int Dly = (int)wUS*17;

	for(; Dly; Dly--);
}