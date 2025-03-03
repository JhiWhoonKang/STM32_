/////////////////////////////////////////////////////////////
// HW3: USART 통신을 이용한센서모니터링
// 제출자: 2019132002 강지훈
// 주요 내용 및 구현 방법
// - 외부 2개, 내부1개의 온도센서로부터 취득된 전압값을 AD 변환
// - ADC 결과값으로부터 전압값과 온도값을 구하여 LCD에 표시
// - USART를 이용하여 PC에 온도값을 업로딩
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
void TIMER1_OC_Init(void); // Timer1 관련 configuration
void DMAInit(void);              // DMA 관련 configuration
void _ADC_Init(void);           // ADC 관련 configuration
void USART1_Init(void);       // USART 관련 configuration
void SENSOR_Init(void);      // 초기 sensor 값 초기화

void USART_BRR_Configuration(uint32_t USART_BaudRate); // Baud rate 관련 configuration

void SerialSendChar(uint8_t c); // 문자 보내기 함수
void SerialSendString(char *s); // 여러문자 보내기 함수

uint16_t KEY_Scan(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

void DisplayTitle(void);

uint16_t ADC_Value, Voltage;
uint8_t str[30];
uint16_t ADC_value[3];
int send_ch= 0x30;
int No_send_data;

/* 3개의 sensor를 구조체화 하고 각각 이름을 sensor1, sensor2, sensor3으로 정의 */
typedef struct SENSOR
{
 	float voltage; // 3개 sensor의  voltage 값 저장
	uint16_t temperature; // 3개 sensor의 temperatur 값 저장
}SENSOR;
SENSOR sensor1, sensor2, sensor3; // Sensor 구조체를 갖는 3개의 sensor 생성

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
}

void TIMER1_OC_Init(void)
{
  // TIM1_CH2 (PA9) : 400ms 이벤트 발생
        // Clock Enable : GPIOA & TIMER1
	RCC->AHB1ENR	|= (1<<0);	        // GPIOA Enable
	RCC->APB2ENR       |=  (1<<0);	// TIMER1 Enable 
    						
        // PA9을 출력설정하고 Alternate function(TIM1_CH2)으로 사용 선언 
	GPIOA->MODER 	|= (2<<2*9);	// PA9 Output Alternate function mode					
	GPIOA->OSPEEDR 	|= (3<<2*9);	// PA9 Output speed (100MHz High speed)
	GPIOA->OTYPER	&= ~(1<<9);	// PA9 Output type push-pull (reset state)
        GPIOA->PUPDR       |= (1<<2*9);      // GPIOA PIN9 Pull-up
	GPIOA->AFR[1]	|= (1 <<4); 	// Connect TIM1 pins(PA9) to AF1(TIM1/2)
					// PA9 ==> TIM1_CH2

	// Setting the Period
	TIM1->PSC = 16800-1;	// Prescaler 168MHz/16800 = 10kHz (0.1ms)
	TIM1->ARR = 4000;	// Auto reload  : 0.1ms * 4K = 400ms(period)

	// CR1 : Up counting
	TIM1->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM1->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM1->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered): ARR Preload Disalbe 
	TIM1->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM1->EGR |= (1<<0);    // UG: Update generation    

  //Output Compare 설정
        // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM1->CCMR1 &= ~(3<<8);  // CC2 channel is configured as output
	TIM1->CCMR1 &= ~(1<<10); // OC2FE=0: Output Compare 1 Fast disable 
	TIM1->CCMR1 &= ~(1<<11); // OC2PE=0: Output Compare 1 preload disable
	TIM1->CCMR1 |=  (3<<12); // OC2M=0b011 (Output Compare 2 Mode : toggle)
        
        // CCER(Capture/Compare Enable Register) : Enable "Channel 2" 
	TIM1->CCER |= (1<<4);	// CC2E=1: CC2 channel Output Enable
	TIM1->CCER &= ~(3<<5);	// CC2P=0: CC2 channel Output Polarity (OCPolarity_High)  
        
        // 신호의 위상(phase) 결정
	TIM1->CCR2 = 1000;	// TIM4 CCR1 TIM4_Pulse

        TIM1->BDTR |= (1<<15);  // main output enable
            
        TIM1->CR1 |= (1<<0);   // CEN: Enable the Tim1 Counter 
}

void DMAInit(void)
{
 	// DMA2 Stream0 channel0 configuration *************************************
	RCC->AHB1ENR |= (1<<22);		//DMA2 clock enable
	DMA2_Stream0->CR &= ~(7<<25);	//DMA2 Stream0 channel 0 selected

	// ADC1->DR(Peripheral) ==> ADC_vlaue(Memory)
	DMA2_Stream0->PAR |= (uint32_t)&ADC1->DR;	   //Peripheral address - ADC1->DR(Regular data) Address
	DMA2_Stream0->M0AR |= (uint32_t)&ADC_value; //Memory address - ADC_Value address 
	DMA2_Stream0->CR &= ~(3<<6);		  //Data transfer direction : Peripheral-to-memory (P=>M)
	DMA2_Stream0->NDTR = 3;			  //DMA_BufferSize = 3 (ADC_Value[3])

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

void _ADC_Init(void)
{   	
        // ADC1: PA1(pin 41)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
        
        // ADC1: PB0(Pin 56)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOB->MODER |= (3<<2*0);		// CONFIG GPIOB PIN1(PB0) TO ANALOG IN MODE
        
        //ADC1 CLK ENA
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |=  (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        
	ADC1->CR1 &=  ~(3<<24);		// RES[1:0]= 0x00 : 12bit Resolution
	ADC1->CR1 |=     (1<<8);		// SCAN=1 : ADC_ScanCovMode Enable

	ADC1->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC1->CR2 |=  (3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Both Edge)
	ADC1->CR2 |= (0x01<<24);	// EXTSEL[3:0]: TIMER1 CC2 EVENT
	ADC1->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC1->SQR1 |= (2<<20);	// L[3:0]: ADC Regular channel sequece length 
        
        /* ADC regular sezuence register */
        // ADC1_IN1, ADC1_IN8, ADC1_IN16
	ADC1->SQR3 |= (1<<0);		        // SQ1[4:0]
        ADC1->SMPR2 |= (0x7<<(3*1));	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)

	ADC1->SQR3 |= (8<<5);		        // SQ2[4:0]
        ADC1->SMPR2 |= (0x7<<(3*8));	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)

	ADC1->SQR3 |= (16<<10);		// SQ3[4:0]
        ADC1->SMPR1 |= (0x7<<(3*6));	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
        /* */
        
        ADC->CCR |= (1<<23);                     // TSVREFE : Temperature sensor and V_REFINT enable
                
        ADC1->CR1 |=  (1<<5);		        // EOCIE=1: Interrupt enable for EOC
	NVIC->ISER[0] |= (1<<18);	        // Enable ADC global Interrupt
        
        
        ADC1->CR2 |= 0x00000200;	        // DMA requests are issued as long as data are converted and DMA=1	
        ADC1->CR2 |= 0x00000100;	        // DMA mode enabled  (DMA=1)
	ADC1->CR2 |= (1<<0);		        // ADON: ADC ON
        
        ADC1->CR2 |= 0x40000000;              // SWSTART=1
}

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
    
	USART_BRR_Configuration(9600); // USART Baud rate Configuration
        
        USART1->CR1	|=  (1<<12); //USART_WorldLength 9 Data bit	
        USART1->CR1	|= (1<<10); // USART_Parity
        USART1->CR1 |= (1<<9);
        
	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable

	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
        USART1->CR1	&= ~(1<<7); // 0x0080, TXE interrupt Disable   , 인터럽트 쓸라해도 먼저 꺼놔야함. 안쓰면 계속 인터럽트 걸려있음. MAIN문에 이에 대해 선언함.

	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)

	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

void SENSOR_Init(void)
{
    sensor1.voltage = 0;
    sensor1.temperature = 0;
    //sensor1.ADC = &ADC_value[0];

    sensor2.voltage = 0;
    sensor2.temperature = 0;
    //sensor2.ADC = &ADC_value[1];

    sensor3.voltage = 0;
    sensor3.temperature = 0;
    //sensor3.ADC = &ADC_value[2];
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
		{	DelayMS(10);
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

void BEEP(void)			// Beep for 20 ms 
{ 	
        GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);                              // 화면 클리어
	LCD_SetFont(&Gulim10);		                     // 폰트 : 굴림 10
        
	LCD_SetBackColor(RGB_WHITE);	             // 글자배경색 : White
	LCD_SetTextColor(RGB_BLACK);	             // 글자색 : Black        
	LCD_DisplayText(0,0,"KJW 2019132002");        // 글자 : 학번
	
        LCD_SetTextColor(RGB_BLUE);	              // 글자색 : Blue
        LCD_DisplayText(1,0,"EXT1 TMP:  C(   V)");     // Sensor 값 display
	LCD_DisplayText(3,0,"EXT2 TMP:  C(   V)");
        LCD_DisplayText(5,0,"INT  TMP:  C(   V)");
  
        LCD_SetTextColor(RGB_RED);	                       // 글자색 : Red
        LCD_DisplayChar(1,14,'.');
        LCD_DisplayChar(3,14,'.');
        LCD_DisplayChar(5,14,'.');
    
        LCD_DisplayChar(0,16,'0'); 	                        // 초기 명령값
}



int main(void)
{
	LCD_Init();	      // LCD 구동 함수
	DelayMS(10);	      // LCD구동 딜레이
 	DisplayTitle();	      //LCD 초기화면구동 함수
      
        _GPIO_Init();           //GPIO 초기화
        TIMER1_OC_Init();   //Timer 초기화
        DMAInit();                //DMA 초기화
	_ADC_Init();             //ADC 초기화
        USART1_Init();         //USART1 초기화
        SENSOR_Init();        //Sensor data 초기화
        
 	while(1)
	{          
	}
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
	while((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), 송신 가능한 상태까지 대기 , TDR이 비어있는지 확인은 SR레지스터 7번 비트에서 확인.(중요) (1(비어있을 떄까찌)이  될때까지 기다려야함)

	USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
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

void USART1_IRQHandler(void)	
{       
	// TX Buffer Empty Interrupt
	if ( (USART1->CR1 & (1<<7)) && (USART1->SR & USART_SR_TXE) )		// USART_SR_TXE=(1<<7)
	{
		USART1->CR1 &= ~(1<<7);	// 0x0080, TXE interrupt Enable 
		sprintf(str,"%2d",send_ch);
		LCD_DisplayText(2,8,str);

		USART1->DR = send_ch;	// 1 byte 전송
		DelayMS(500);
		send_ch++;
		if (send_ch < 0x30+No_send_data )   // 데이터(문자) 10byte 전송
			USART1->CR1 |= (1<<7); 	// TXE interrupt Enable 
	} 
	// RX Buffer Full interrupt
	if ( (USART1->SR & USART_SR_RXNE) )		// USART_SR_RXNE=(1<<5) 
	{
		char ch;
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
		
                if(ch=='1')
                {
                  sprintf(str,"%2d ",sensor1.temperature); // sensor1 temp 전송
                  SerialSendString(str);
                  LCD_DisplayChar(0,16,ch); 	// LCD  명령값 display
                }
                
                else if(ch=='2')
                {
                  sprintf(str,"%2d ",sensor2.temperature); //sensor2 temp 전송
                  SerialSendString(str);
                  LCD_DisplayChar(0,16,ch); 	// LCD  명령값 display
                }
                
                else if(ch=='3')
                {
                  sprintf(str,"%2d ",sensor3.temperature); // sensor3 temp 전송
                  SerialSendString(str);
                  LCD_DisplayChar(0,16,ch); 	// LCD  명령값 display
                }
                
                else
                {
                  LCD_DisplayText(0, 16, "undefined cmd"); 	// LCD  명령값 display
                }
	} 
}

void ADC_IRQHandler(void)
{
	ADC1->SR &= ~(1<<1);		// EOC flag clear
        
        /* sensor equation */
        // 3.3 : 4095 =  Volatge : ADC_Value 
        // sensor1, 2 Temperature = 3.5 * Voltage^2 + 1
        // sensor3 Temperature = ((Sensor_Voltage-0.76) / 2.5) + 25
        //  화면 가로 픽셀: 160, 좌우 10픽셀 씩 빼면 140픽셀
        // 막대 길이 1 ~ 39 => temperature / 39 * 140
        
        sensor1.voltage = (ADC_value[0] * 3.3) / 4095;
        sensor1.temperature = (int)(3.5*(sensor1.voltage*sensor1.voltage) + 1);
        
        sensor2.voltage = (ADC_value[1] * 3.3) / 4095;
        sensor2.temperature = (int)(3.5*(sensor2.voltage*sensor2.voltage) + 1);
        
        sensor3.voltage = (ADC_value[2] * 3.3) / 4095;
        sensor3.temperature = (int)(((sensor3.voltage-0.76) / 2.5) + 25);          
        /* */
        
        /* sensor1 */     
        // temperature display
        LCD_DisplayChar(1,9,(sensor1.temperature/10) + 0x30);
        LCD_DisplayChar(1,10,(sensor1.temperature%10) + 0x30);
        
        // voltage display
        LCD_DisplayChar(1,13,((int)sensor1.voltage)%10 + 0x30);
        LCD_DisplayChar(1,15,((int)(sensor1.voltage*10))%10 + 0x30);    
        
        // draw 온도 막대
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect(10, 37, 200, 10); // 먼저 지우고
        LCD_SetBrushColor(RGB_RED);
        LCD_DrawFillRect(10, 37, (int)((float)sensor1.temperature/39*140), 10); // 현재 온도 그리기
        /* */
        
        /* sensor2 */ 
        // temperature display
        LCD_DisplayChar(3,9,(sensor2.temperature/10) + 0x30);
        LCD_DisplayChar(3,10,(sensor2.temperature%10) + 0x30);
        
        // voltage display        
        LCD_DisplayChar(3,13,((int)sensor2.voltage)%10 + 0x30);
        LCD_DisplayChar(3,15,((int)(sensor2.voltage*10))%10 + 0x30);
        
        // draw 온도 막대
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect(10, 71, 200, 10);      // 먼저 지우고
        LCD_SetBrushColor(RGB_GREEN);    // 막대기 색: 초록색
        LCD_DrawFillRect(10, 71, (int)((float)sensor2.temperature/39*140), 10); // 현재 온도 그리기
        /* */
        
        /* sensor3 */
        // temperature display
        LCD_DisplayChar(5,9,(sensor3.temperature/10) + 0x30); 
        LCD_DisplayChar(5,10,(sensor3.temperature%10) + 0x30);
        
        // voltage display
        LCD_DisplayChar(5,13,((int)sensor3.voltage)%10 + 0x30); //voltage display
        LCD_DisplayChar(5,15,((int)(sensor3.voltage*10))%10 + 0x30);      
        
        // draw 온도 막대
        LCD_SetBrushColor(RGB_WHITE);  // 막대기 색: 하얀색
        LCD_DrawFillRect(10, 102, 200, 10); // 먼저 지우고
        LCD_SetBrushColor(RGB_BLUE);    // 막대기 색: 파란색
        LCD_DrawFillRect(10,102, (int)((float)sensor3.temperature/39*140), 10); // 현재 온도 그리기
        /* */       
}
