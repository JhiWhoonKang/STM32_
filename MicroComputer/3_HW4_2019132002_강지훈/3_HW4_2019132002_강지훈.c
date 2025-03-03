/////////////////////////////////////////////////////////////
// HW4: 가속도값(SPI)을 이용한 Ball game
// 제출자: 2019132002 강지훈
// 주요 내용 및 구현 방법
// - SPI 통신을 이용하여 가속도센서로부터가속도값(X,Y,Z)을 읽어서 표시
// - 키트를 기울임에따라변하는가속도값을반영하여공이이동하도록표시
/////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

#define Xmin 1          // x 경계 최소
#define Xmax 106        //x  경계 최대
#define Ymin 11         //y 경계 최소
#define Ymax 115        //y경계 최대
#define XmaxBoundary Xmax + Xmin        //x경계 최대
#define YmaxBoundary Ymax + Ymin        //y경계 최대
#define XminBoundary Xmin       //x경계 최소
#define YminBoundary Ymin       //y경계 최소
#define CenterX (Xmin + Xmax) / 2       //x 중심 좌표
#define CenterY (Ymin + Ymax) / 2       //y 중심 좌표

void DisplayTitle(void);
void _GPIO_Init(void);
void SPI1_Init(void);
void TIMER10_Init(void);
void Display_Process(int16 *pBuf);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

float Xcoordinate, Ycoordinate;         //x, y 좌표 변수
//// void SPI1_Process(UINT16 *pBuf);  // ACC.c (ACC.h) 
//// void ACC_Init(void) // ACC.c (ACC.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl;

int main(void)
{
	int16 buffer[3];
    
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
	DisplayTitle();		// LCD 초기화면구동 함수
    
	_GPIO_Init();		// LED, SW 초기화
	SPI1_Init();        	// SPI1 초기화
	ACC_Init();		// 가속도센서 초기화
	TIMER10_Init();		// 가속도센서 스캔 주기 생성
   
	while(1)
	{
		if(bControl)
		{
			bControl = FALSE;     
			SPI1_Process(&buffer[0]);	// SPI통신을 이용하여 가속도센서 측정
			Display_Process(&buffer[0]);	// 측정값을 LCD에 표시
		}
	}
}

///////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void SPI1_Init(void)
{
	/*!< Clock Enable  *********************************************************/
	RCC->APB2ENR 	|= (1<<12);	// 0x1000, SPI1 Clock EN
	RCC->AHB1ENR 	|= (1<<0);	// 0x0001, GPIOA Clock EN		
  
	/*!< SPI1 pins configuration ************************************************/
	
	/*!< SPI1 NSS pin(PA8) configuration : GPIO 핀  */
	GPIOA->MODER 	|= (1<<(2*8));	// 0x00010000, PA8 Output mode
	GPIOA->OTYPER 	&= ~(1<<8); 	// 0x0100, push-pull(reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*8));	// 0x00030000, PA8 Output speed (100MHZ) 
	GPIOA->PUPDR 	&= ~(3<<(2*8));	// 0x00030000, NO Pullup Pulldown(reset state)
    
	/*!< SPI1 SCK pin(PA5) configuration : SPI1_SCK */
	GPIOA->MODER 	|= (2<<(2*5)); 	// 0x00000800, PA5 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<5); 	// 0020, PA5 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*5));	// 0x00000C00, PA5 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*5)); 	// 0x00000800, PA5 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*5));	// 0x00500000, Connect PA5 to AF5(SPI1)
    
	/*!< SPI1 MOSI pin(PA7) configuration : SPI1_MOSI */    
	GPIOA->MODER 	|= (2<<(2*7));	// 0x00008000, PA7 Alternate function mode
	GPIOA->OTYPER	&= ~(1<<7);	// 0x0080, PA7 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*7));	// 0x0000C000, PA7 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*7)); 	// 0x00008000, PA7 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*7));	// 0x50000000, Connect PA7 to AF5(SPI1)
    
	/*!< SPI1 MISO pin(PA6) configuration : SPI1_MISO */
	GPIOA->MODER 	|= (2<<(2*6));	// 0x00002000, PA6 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<6);	// 0x0040, PA6 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*6));	// 0x00003000, PA6 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*6));	// 0x00002000, PA6 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*6));	// 0x05000000, Connect PA6 to AF5(SPI1)

	// Init SPI1 Registers 
	SPI1->CR1 |= (1<<2);	// MSTR(Master selection)=1, Master mode
	SPI1->CR1 &= ~(1<<15);	// SPI_Direction_2 Lines_FullDuplex
	SPI1->CR1 &= ~(1<<11);	// SPI_DataSize_8bit
	SPI1->CR1 |= (1<<9);  	// SSM(Software slave management)=1, 
				// NSS 핀 상태가 코딩에 의해 결정
	SPI1->CR1 |= (1<<8);	// SSI(Internal_slave_select)=1,
				// 현재 MCU가 Master이므로 NSS 상태는 'High' 
	SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
	SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
	SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
	SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, 두 번째 edge 에서 데이터가 샘플링
 
	SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER10_Init(void)	// 가속도센서 측정 주기 생성: 200ms
{
        // PB8: TIM10_CH1 (가장 중요 )/
        // PB8을 출력설정하고 Alternate function(TIM10_CH1)으로 사용 선언
	RCC->AHB1ENR	|= (1<<1);	// 0x02, RCC_AHB1ENR GPIOB Enable : AHB1ENR.1

	GPIOB->MODER    |= (2<<2*8);	// 0x02000000(MODER.(25,24)=0b10), GPIOD PIN12 Output Alternate function mode 					
	GPIOB->OSPEEDR 	|= (3<<2*8);	// 0x03000000(OSPEEDER.(25,24)=0b11), GPIOD PIN12 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<8);	// ~0x1000, GPIOD PIN12 Output type push-pull (reset state)
	GPIOB->PUPDR    |= (1<<2*8); 	// 0x01000000, GPIOD PIN12 Pull-up
  					// PD12 ==> TIM4_CH1
	GPIOB->AFR[1]	|= (3<<0);  // (AFR[1].(19~16)=0b0010): Connect TIM4 pins(PD12) to AF2(TIM3..5)
 
        // TIMER10 설정
	RCC->APB2ENR 	|= (1<<17);	// TIMER10 Clock Enable
     
        TIM10->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM10->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
        TIM10->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
        TIM10->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM10->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
	TIM10->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM10->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
            
	TIM10->PSC = 16800-1;	// Prescaler 168MHz/16800 = 10KHz (0.1ms)  
	TIM10->ARR = 2000-1;	// Auto reload  0.1ms * 2000 = 200ms
	TIM10->EGR |=(1<<0);	// Update Event generation    
        
        // Output Compare 설정
        /* 중요한 설정 */
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM10->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' /* Output , Output mode면 윗줄 */
	TIM10->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM10->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1에 언제든지 새로운 값을 loading 가능) 
	TIM10->CCMR1 |= (3<<4);    // OC1REF toggles when CNT = CCR1
				
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM10->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM4_CH1) Active: 해당핀(100번)을 통해 신호출력
	TIM10->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  

	// CC1I(CC 인터럽트) 인터럽트 발생시각 또는 신호변화(토글)시기 결정: 신호의 위상(phase) 결정
	// 인터럽트 발생시간(10000 펄스)의 10%(1000) 시각에서 compare match 발생
	TIM10->CCR1 = 1000;	// TIM10 CCR1 TIM10_Pulse

	TIM10->DIER |= (1<<0);	// UIE: Enable Timer10 Update interrupt
	TIM10->DIER |= (1<<1);	// CC1IE: Enable the Timer10 CC1 interrupt

	NVIC->ISER[0] 	|= (1<<25);	// Enable Timer10 global Interrupt on NVIC

	TIM10->CR1 |= (1<<0);	// CEN: Enable the Timer10 Counter  		
}

void TIM1_UP_TIM10_IRQHandler(void)	// 250ms int
{
        if ((TIM10->SR & 0x01) != RESET)	// Update interrupt flag (10ms)
	{
		TIM10->SR &= ~(1<<0);	// Update Interrupt Claer
                bControl = TRUE;	// 200ms마다 센서 측정
	}
    
	if((TIM10->SR & 0x02) != RESET)	// Capture/Compare 1 interrupt flag
	{
		TIM10->SR &= ~(1<<1);	// CC 1 Interrupt Claer
	}
}

void Display_Process(int16 *pBuf)
{
	UINT16 G_VALUE;
        float X_VALUE;  //x축 가속도 값
        float Y_VALUE;  //y축 가속도 값
        X_VALUE =pBuf[0]; Y_VALUE = pBuf[1]; //
	// X 축 가속도 표시		
	if (pBuf[0] < 0)  //음수
	{
		G_VALUE = abs(pBuf[0]);
		LCD_DisplayChar(1,21,'-'); // g 부호 표시
	}
	else				// 양수
	{
		G_VALUE = pBuf[0];
		LCD_DisplayChar(1,21,'+'); // g 부호 표시
	}        
	G_VALUE = 100 * G_VALUE / 0x4009; // 가속도 --> g 변환
        X_VALUE = 100 * X_VALUE / 0x4009; // 가속도 --> g 변환
	LCD_DisplayChar(1,22, G_VALUE/100 +0x30);       //1의 자리 가속도 값 display
	LCD_DisplayChar(1,23,'.');      //. display
	LCD_DisplayChar(1,24, G_VALUE%100/10 +0x30);    //소수점 첫째자리 display
	LCD_DisplayChar(1,25,'g');      //단위 display

	// Y 축 가속도 표시	
	if (pBuf[1] < 0)  //음수
	{
		G_VALUE = abs(pBuf[1]);
		LCD_DisplayChar(2,21,'-'); // g 부호 표시
	}
	else				// 양수
	{
		G_VALUE = pBuf[1];
		LCD_DisplayChar(2,21,'+'); // g 부호 표시
	}
	G_VALUE = 100 * G_VALUE / 0x4009;       // 가속도 --> g 변환
        Y_VALUE = 100 * Y_VALUE / 0x4009;       // 가속도 --> g 변환
	LCD_DisplayChar(2,22, G_VALUE/100 +0x30);       //1의 자리 가속도 값 display
	LCD_DisplayChar(2,23,'.');      //. display
	LCD_DisplayChar(2,24, G_VALUE%100/10 +0x30);    //소수점 첫째자리 display
	LCD_DisplayChar(2,25,'g');      //단위 display

	// Z 축 가속도 표시	
	if (pBuf[2] < 0)  //음수
	{
		G_VALUE = abs(pBuf[2]);
		LCD_DisplayChar(3,21,'-'); // g 부호 표시
	}
	else				// 양수
	{
		G_VALUE = pBuf[2];
		LCD_DisplayChar(3,21,'+'); // g 부호 표시
	}
	G_VALUE = 100 * G_VALUE / 0x4009; 
	LCD_DisplayChar(3,22, G_VALUE/100 +0x30);
	LCD_DisplayChar(3,23,'.');
	LCD_DisplayChar(3,24, G_VALUE%100/10 +0x30);
	LCD_DisplayChar(3,25,'g');
        
        /* 현재 좌표 지우기 */
        LCD_SetPenColor(RGB_WHITE);
        LCD_DrawRectangle(Xcoordinate - 2, Ycoordinate - 2, 4, 4);
        
        /* 이동할 좌표 그리기 */       
        Xcoordinate = Xcoordinate - X_VALUE / 3; 
        Ycoordinate = Ycoordinate - Y_VALUE / 3;
        
        /* 경계 제한 */
        if(Xcoordinate + 2 >= XmaxBoundary)
        {
            Xcoordinate = XmaxBoundary - 3;
        }
        
        if(Xcoordinate - 2<= XminBoundary)
        {
            Xcoordinate = XminBoundary + 3;
        }
        
        if(Ycoordinate + 2 >= YmaxBoundary)
        {
            Ycoordinate = YmaxBoundary - 3;
        }
        
        if(Ycoordinate - 2 <= YminBoundary)
        {
            Ycoordinate = YminBoundary + 3;
        }                
        /* */
        LCD_SetPenColor(RGB_RED);
        LCD_DrawRectangle(Xcoordinate - 2, Ycoordinate - 2, 4, 4);      //위치 update
}

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

	GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;

	for (i=0; i<wMS; i++)
		DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim7);		//폰트 
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);    //글자색
	LCD_DisplayText(0,0," Ball game: KJW 2019132002");  // Title

	LCD_DisplayText(1,18,"Ax:");	//X AXIS
	LCD_DisplayText(2,18,"Ay:");	//Y AXIS
	LCD_DisplayText(3,18,"Az:");	//Z AXIS
        
        LCD_SetPenColor(RGB_BLUE);
        LCD_DrawRectangle(Xmin,Ymin,Xmax,Ymax);
                
        Xcoordinate = (Xmax + Xmin) / 2;        //초기 x좌표
        Ycoordinate = (Ymax + Ymin) / 2;        //초기 y좌표
        LCD_SetPenColor(RGB_RED);
        LCD_DrawRectangle(Xcoordinate - 2, Ycoordinate - 2, 4, 4);      //초기 위치에 사각형 draw
}
