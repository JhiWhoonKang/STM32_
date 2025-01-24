/////////////////////////////////////////////////////////////
// 과제명: 마이컴구조2023_TP2_이진 연산기
// 과제개요: 이진(2비트) 연산을 하는 계산기 프로그램 제작
// 사용한 하드웨어(기능): GPIO, Joy-stick, EXTI, GLCD , FRAM...
// 제출일: 2023. 6. 11
// 제출자 클래스: 수요일반
// 학번: 2019132002
// 이름: 강지훈
///////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayInitScreen(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
uint16_t KEY_Scan(void);
uint16_t JOY_Scan(void);

int PLUS(int Aopnd, int Bopnd); // PLUS 연산 함수
int MINUS(int Aopnd, int Bopnd); // MINUS 연산 함수
int MUL(int Aopnd, int Bopnd); // MUX 연산 함수
int AND(int Aopnd, int Bopnd); // AND 연산 함수
int OR(int Aopnd, int Bopnd); // OR 연산 함수
int XOR(int Aopnd, int Bopnd); // XOR 연산 함수
char _operator = '0'; // 연산자 변수 선언
uint8_t result = 0; // 결과값 변수 선언

uint8_t SW1_Flag = 0; // 연산 SW
uint8_t SW2_Flag, SW3_Flag; // Operand A SW
uint8_t SW4_Flag, SW5_Flag; // Operand B SW
int Operator_flag; // Operator Flag
int stop_flag = 0; // Stop Flag

int main(void)
{
  LCD_Init(); // LCD 모듈 초기화
  _GPIO_Init(); // GPIO 초기화
  _EXTI_Init(); // EXTI 초기화
  Fram_Init();                    // FRAM 초기화 H/W 초기화
  Fram_Status_Config();   // FRAM 초기화 S/W 초기화
  
  DisplayInitScreen(); // LCD 초기 화면
  
  _operator = Fram_Read(530); // 연산자 변수에 Fram_Read
  result = Fram_Read(531); // 결과값 변수에 Fram_Read
  
  // 이전 연산 Flag get
  if(_operator == '+')
    Operator_flag = 1;
  else if(_operator == '-')
    Operator_flag = 2;
  else if(_operator == 'x')
    Operator_flag = 3;
  else if (_operator == '&')
    Operator_flag = 4;
  else if (_operator == '|')
    Operator_flag = 5;
  else if (_operator == '^')
    Operator_flag = 6;
  LCD_DrawChar(76, 60, _operator); // 저장된 operator Display
  
  // 읽어온 result에서 이전 결과값이 - 경우 - Display
  if((result>>4) & 0x1) {
    LCD_SetBackColor(RGB_YELLOW); // 배경색 : 노란색
    LCD_DrawText(114,16,"-");
  }
    
  
  /*저장된 Bit3~Bit0 Display*/
  LCD_SetBackColor(RGB_YELLOW); //  글자 배경색 : 노란색
  LCD_DrawChar(114,  41, '0' + ((result >> 3) & 0x1));
  LCD_DrawChar(114,  61, '0' + ((result >> 2) & 0x1));
  LCD_DrawChar(114,  86, '0' + ((result >> 1) & 0x1));
  LCD_DrawChar(114,  106, '0' + (result & 0x1));
  
  /*Operand A & B SW Flag*/
  SW2_Flag = 0, SW3_Flag = 0, SW4_Flag = 0, SW5_Flag = 0;
  
  while(1){
    LCD_SetBackColor(RGB_YELLOW);
    switch(KEY_Scan()){ // SW 입력
    case 0xFB00: // SW2 입력 (MSB)
      //BEEP();
      if(SW2_Flag==0){
        LCD_DrawText(39,21,"1");
        SW2_Flag=1;
      }
      else{
        LCD_DrawText(39,21,"0");
        SW2_Flag=0;
      }
      break;
      
    case 0xF700: // SW3 입력 (LSB)
      //BEEP();
      if(SW3_Flag==0){
        LCD_DrawText(39,41,"1");
        SW3_Flag=1;
      }
      else{
        LCD_DrawText(39,41,"0");
        SW3_Flag=0;
      }
      break;
        
    case 0xEF00: // SW4 입력 (MSB)
      //BEEP();
      if(SW4_Flag==0){
        LCD_DrawText(39,86,"1");
        SW4_Flag=1;
      }
      else{
        LCD_DrawText(39,86,"0");
        SW4_Flag=0;
      }
      break;
      
    case 0xDF00: // SW5 입력 (LSB)
      //BEEP();
      if(SW5_Flag==0){
        LCD_DrawText(39,106,"1");
        SW5_Flag=1;
      }
      else{
        LCD_DrawText(39,106,"0");
        SW5_Flag=0;
      }
      break;
    }
    
    /*계산 시작 SW 눌렀을 경우*/
    if(SW1_Flag==1){
      LCD_SetBackColor(RGB_YELLOW); // 배경색 : 노란색
      int Aopnd = (SW2_Flag << 1) | SW3_Flag; // Aopnd에 SW2, SW3 bit 정보 저장
      int Bopnd = (SW4_Flag << 1) | SW5_Flag; // Bopnd에 SW2, SW3 bit 정보 저장
      
      if(Operator_flag == 2){ // MINUS 연산인 경우
        int _result = MINUS(Aopnd, Bopnd); // MINUS 연산 결과 (부호 포함)
        result = abs(MINUS(Aopnd, Bopnd)); // MINUS 연산 결과 (절댓값)
        if(_result < 0) { // 연산 결과가 -일 경우 - Display
          LCD_DrawChar(114, 16, '-');
          result |= 0x10; // bit4에 - 정보 result에 저장 0x0001 ????
        }
        else // 연산 결과가 +일 경우 + Display
          LCD_DrawChar(114, 16, '+');
      }
      else if(Operator_flag == 1) { // PLUS 연산인 경우
        result = PLUS(Aopnd, Bopnd);
        LCD_DrawChar(114, 16, '+');
      }
      else if(Operator_flag == 3) { // MUL 연산인 경우
        result = MUL(Aopnd, Bopnd);
        LCD_DrawChar(114, 16, '+');
      }
      else if(Operator_flag == 4) { // AND 연산인 경우
        result = AND(Aopnd, Bopnd);
        LCD_DrawChar(114, 16, '+');
      }
      else if(Operator_flag == 5) { // OR 연산인 경우
        result = OR(Aopnd, Bopnd);
        LCD_DrawChar(114, 16, '+');
      }
      else { // XOR 연산인 경우
        result = XOR(Aopnd, Bopnd);
        LCD_DrawChar(114, 16, '+');
      }
      // Fram에 연산 결과 저장
      Fram_Write(531, result);
      
      // LCD에 출력
      LCD_DrawChar(114,  41, '0' + ((result >> 3) & 0x1));
      LCD_DrawChar(114,  61, '0' + ((result >> 2) & 0x1));
      LCD_DrawChar(114,  86, '0' + ((result >> 1) & 0x1));
      LCD_DrawChar(114,  106, '0' + (result & 0x1));
      
      SW1_Flag = 0; // 연산이 끝나면 Flag 0 으로 초기화
    }
  }
}


void _GPIO_Init(void){
  // LED (GPIO G) 설정 : Output mode
  RCC->AHB1ENR |= 0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable
  GPIOG->MODER |= 0x00005555;   // GPIOG 0~7 : Output mode (0b01)
  GPIOG->OTYPER &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)
  GPIOG->OSPEEDR |= 0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed
  
  // SW (GPIO H) 설정 : Input mode
  RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable
  GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)
  GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
  
  // Buzzer (GPIO F) 설정 : Output mode
  RCC->AHB1ENR |= 0x00000020;   // RCC_AHB1ENR : GPIOF(bit#5) Enable
  GPIOF->MODER |= 0x00040000;   // GPIOF 9 : Output mode (0b01)
  GPIOF->OTYPER &= ~0x0200;   // GPIOF 9 : Push-pull
  GPIOF->OSPEEDR |= 0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed
  
  //Joy Stick SW(GPIO I) 설정 : Input mode
  RCC->AHB1ENR	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable 0x00000100
  GPIOI->MODER	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state) 
  GPIOI->PUPDR	&= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}

/* GLCD 초기화면 설정 함수 */
void DisplayInitScreen(void){
  LCD_Clear(RGB_WHITE); // 화면 클리어 : 하얀색
  LCD_SetFont(&Gulim8); // 글자 폰트 : 굴림 8
  LCD_SetBackColor(RGB_WHITE); //  글자 배경색 : 하얀색
  
  /* 초록색 선 도형 */
  LCD_SetPenColor(GET_RGB(0, 102, 51));
  LCD_DrawRectangle(65,10,30,110);
  LCD_DrawRectangle(15,30,15,17);
  LCD_DrawRectangle(15,95,15,17);
  LCD_DrawRectangle(130,50,15,17);
  /* */
  
  /* 파란색 선 */
  LCD_SetPenColor(RGB_BLUE);
  LCD_DrawHorLine(50, 27, 15);
  LCD_DrawHorLine(50, 47, 15);
  
  LCD_DrawHorLine(50, 92, 15);
  LCD_DrawHorLine(50, 112, 15);
  
  LCD_DrawHorLine(95, 22, 15);
  LCD_DrawHorLine(95, 47, 15);
  LCD_DrawHorLine(95, 67, 15);
  LCD_DrawHorLine(95, 92, 15);
  LCD_DrawHorLine(95, 112, 15);
  /* */
  
  /* 연보라 박스 */
  LCD_SetBrushColor(GET_RGB(238, 130, 238));
  LCD_DrawFillRect(73,60,15,17);
  /* */
  
  /* 노란색 박스 */
  LCD_SetBrushColor(RGB_YELLOW);
  LCD_DrawFillRect(35, 20, 15, 15);
  LCD_DrawFillRect(35, 40, 15, 15);
  
  LCD_DrawFillRect(35, 85, 15, 15);
  LCD_DrawFillRect(35, 105, 15, 15);
  
  LCD_DrawFillRect(110, 15, 15, 15);
  
  LCD_DrawFillRect(110, 40, 15, 15);
  LCD_DrawFillRect(110, 60, 15, 15);
  
  LCD_DrawFillRect(110, 85, 15, 15);
  LCD_DrawFillRect(110, 105, 15, 15);
  
  LCD_DrawFillRect(71, 100, 20, 15);
  /* */
  
  /* 검정색 글자 */
  LCD_SetTextColor(RGB_BLACK);
  LCD_SetBackColor(RGB_YELLOW);
  LCD_DrawText(39,21,"0");
  LCD_DrawText(39,41,"0");
  
  LCD_DrawText(39,86,"0");
  LCD_DrawText(39,106,"0");
  
  LCD_DrawText(114,16,"+");
  
  LCD_DrawText(114,41,"0");
  LCD_DrawText(114,61,"0");
  
  LCD_DrawText(114,86,"0");
  LCD_DrawText(114,106,"0");
  
  LCD_DrawText(73,101,"+0");
  
  LCD_SetFont(&Gulim10); // 글자 폰트 : 굴림 10
  LCD_SetBackColor(RGB_WHITE);
  LCD_DrawText(19, 31, "A");
  LCD_DrawText(19, 96, "B");
  LCD_DrawText(134, 51, "C");
  
  LCD_SetBackColor(GET_RGB(238, 130, 238));
  LCD_DrawText(76,60,"+");
  /* */
}

int PLUS(int Aopnd, int Bopnd) { // PLUS 연산
  return Aopnd + Bopnd;
}

int MINUS(int Aopnd, int Bopnd) { // MINUS 연산
  return Aopnd - Bopnd;
}

int MUL(int Aopnd, int Bopnd) { // MUL 연산
  return Aopnd * Bopnd;
}

int AND(int Aopnd, int Bopnd) { // AND 연산
  return Aopnd &= Bopnd;
}

int OR(int Aopnd, int Bopnd) { // OR 연산
  return Aopnd |= Bopnd;
}

int XOR(int Aopnd, int Bopnd) { // XOR 연산
  return Aopnd ^= Bopnd;
}

/* EXTI 초기 설정 */
void _EXTI_Init(void){
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR GPIOH Enable
  RCC->AHB1ENR |= 0x00000100; // RCC_AHB1ENR GPIOI Enable
  RCC->APB2ENR |= 0x00004000; // Enable System Configuration Controller Clock
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH PIN8~PIN15 Input mode (reset state)
  GPIOI->MODER &= ~0x000FFC00; // GPIOI PIN5~PIN9 Input mode (reset state)
    
  /*EXTI6*/
  SYSCFG->EXTICR[1] |= 0x0800; // EXTI6에 대한 소스 입력은 GPIOI로 설정
  EXTI->RTSR |= 0x000040; // EXTI6: Rising Trigger Enable
  EXTI->IMR |= 0x000040; // EXTI6 인터럽트 mask (Interrupt Enable) 설정
  
  /*EXTI8*/ 
  SYSCFG->EXTICR[2] |= 0x0008; // EXTI8에 대한 소스 입력은 GPIOI로 설정
  EXTI->RTSR |= 0x000300; // EXTI8: Rising Trigger Enable
  EXTI->IMR |= 0x000300; // EXTI8 인터럽트 mask (Interrupt Enable) 설정
  
  /*EXTI9*/
  SYSCFG->EXTICR[2] |= 0x0070; // EXTI9에 대한 소스 입력은 GPIOH로 설정
  EXTI->RTSR |= 0x000300; // EXTI9: Rising Trigger Enable
  EXTI->IMR |= 0x000300; // EXTI9 인터럽트 mask (Interrupt Enable) 설정
  
  
  /*EXTI14*/
  SYSCFG->EXTICR[3] |= 0x0700; // EXTI14에 대한 소스 입력은 GPIOH로 설정
  EXTI->RTSR |= 0x004000; // EXTI14: Rising Trigger Enable
  EXTI->IMR |= 0x004000; // EXTI14 인터럽트 mask (Interrupt Enable) 설정
  
  NVIC->ISER[0] |= 1<<23; // Enable 'Global Interrupt EXTI6, EXTI8, EXTI9'
  NVIC->ISER[1] |= 1<<(40-32); // Enable 'Global Interrupt EXTI14'
  
  NVIC->IP[23]= 0xF0;  // High Priority 
  NVIC->IP[40]= 0xE0;  // Low Priority
}

/* EXTI6, EXTI8, EXTI9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void){
  
  /*EXTI6*/
  if(EXTI->PR & 0x0040){ // EXTI6 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x0040; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    //BEEP(); // 부저 1회
    LCD_SetBackColor(RGB_YELLOW); // 배경색 : 노란색
    LCD_DrawText(73,101,"+1"); // +1 Display
    GPIOG->ODR |= 0x0080; // LED7 ON
    
    while(stop_flag != 1){ // Stop Flag가 입력되기 전까지 반복
      DelayMS(500); // 0.5초마다 반복 수행
      result += 1; // 연산 결과를 +1
      
      // 4비트 결과 추출
      int bit3 = (result >> 3) & 0x1;
      int bit2 = (result >> 2) & 0x1;
      int bit1 = (result >> 1) & 0x1;
      int bit0 = result & 0x1;

      // LCD에 출력
      LCD_DrawText(114, 41, (bit3 ? "1" : "0"));
      LCD_DrawText(114, 61, (bit2 ? "1" : "0"));
      LCD_DrawText(114, 86, (bit1 ? "1" : "0"));
      LCD_DrawText(114, 106, (bit0 ? "1" : "0"));
    }
    /*연속 증가 모드 탈출*/
    LCD_DrawText(73,101,"+0"); // 종료되면 +0 Display
    //BEEP(); DelayMS(500); BEEP(); DelayMS(500); BEEP(); DelayMS(500); // 부저 3회
    GPIOG->ODR &= ~0x0080; // LED7 OFF
    stop_flag = 0; // Stop Flag 0 으로 초기화
  }
  
  /*EXTI8*/
  if(EXTI->PR & 0x0100){ // EXTI8 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x0100; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    //BEEP(); // 부저 1회
    ++Operator_flag; // Operator Flag 증가
    LCD_SetBackColor(GET_RGB(238, 130, 238)); // 배경색 : 보라색
    
    if(Operator_flag == 1) { // Flag1 = PLUS
      LCD_DrawText(76,60,"+");
      _operator = '+';
    }
    else if(Operator_flag == 2) { // Flag2 = MINUS
      LCD_DrawText(76,60,"-");
      _operator = '-';
    }
    else if(Operator_flag == 3) { // Flag3 = MUL
      LCD_DrawText(76,60,"x");
      _operator = 'x';
    }
    else if(Operator_flag == 4) { // Flag4 = AND
      LCD_DrawText(76,60,"&");
      _operator = '&';
    }
    else if(Operator_flag == 5) { // Flag5 = OR
      LCD_DrawText(76,60,"|");
      _operator = '|';
    }
    else if(Operator_flag == 6){ // Flag6 = XOR
      LCD_DrawText(76,60,"^");
      _operator = '^';
      Operator_flag = 0;
    }
    Fram_Write(530, _operator); // Operator 정보 Fram_Write
  }
  
  /*EXTI9*/
  if(EXTI->PR & 0x0200){ // EXTI9 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x0200; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    //BEEP(); // 부저 1회
    SW1_Flag = 1; // SW1 Flag = 1
  }
}

/* EXTI14 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x4000){ // EXTI14 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x4000; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    //BEEP(); // 부저 1회
    DelayMS(1000); // 1초 뒤 Stop 동작
    stop_flag = 1; // Stop Flag = 1
  }
}

/* Switch가 입력되었는지 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수  */ 
uint8_t key_flag = 0;
uint16_t KEY_Scan(void){	// input key SW0 - SW7
  uint16_t key;
  key = GPIOH->IDR & 0xFF00;
  if(key == 0xFF00){
    if(key_flag == 0){
      return key;
    }
    else{
      DelayMS(10);
      key_flag = 0;
      return key;
    }
  }
  else{
    if(key_flag != 0)
      return 0xFF00;
    else{
      key_flag=  1;
      DelayMS(10);
      return key;
    }
  }
}

/* Joy stick이 입력되었는지 여부와 어떤 Joy stick이 입력되었는지의 정보를 return하는 함수  */ 
uint8_t joy_flag = 0;
uint16_t JOY_Scan(void){ // input joy stick NAVI_*  
  uint16_t key;
  key = GPIOI->IDR & 0x0FF0;	// any key pressed ?
  if(key == 0x0FF0){ // if no key, check key off
    if(joy_flag == 0){
      return key;
    }
    else{
      DelayMS(10);
      joy_flag=0;
      return key;
    }
  }
  else{ // if key input, check continuous key
    if(joy_flag != 0) // if continuous key, treat as no key input
      return 0x0FF0;
    else{ // if new key,delay for debounce
      joy_flag = 1;
      DelayMS(10);
      return key;
    }
  }
}

/* Buzzer: Beep for 30 ms */
void BEEP(void)			
{ 	
	GPIOF->ODR |=  0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);	// 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}