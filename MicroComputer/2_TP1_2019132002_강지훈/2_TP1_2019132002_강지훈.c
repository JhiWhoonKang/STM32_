/////////////////////////////////////////////////////////////
// 과제명: 마이컴구조2023_TP1_TWO ELEVATOR
// 과제개요: 6층 건물에 두대의 엘리베이터 설치, 출발층에서 목표층 스위치(1~6층)를 선택하면 엘리베이터가 이동하도록 프로그램 작성.
//              이동은 이동 막대의 크기 변화로 표시
// 사용한 하드웨어(기능): GPIO, Joy-stick, EXTI, GLCD , FRAM...
// 제출일: 2023. 6. 06
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
void MOVE_Right_E(int Start_FL, int Des_FL); // 우측 엘리베이터 동작 함수 선언
void MOVE_Left_E(int Start_FL, int Des_FL); // 좌측 엘리베이터 동작 함수 선언
void ExecuteMode(int Start_FL, int Des_FL); // 엘리베이터 Execute mode 함수 선언
void HoldMode(void); // 엘리베이터 Hold mode 함수 선언
uint16_t KEY_Scan(void);

uint8_t Floor_mode = 0, Hold_mode = 0, Execute_mode = 0; // 동작 모드 변수
uint8_t Left_E = 1, Right_E = 1, Start_FL = 1, Des_FL = 1; // 좌/우측 현재 층 및 목표 층 변수
uint8_t left = 0, right = 0; //  좌/우측 이동막대 위치 변수

int main(void)
{
  LCD_Init(); // LCD 모듈 초기화
  _GPIO_Init(); // GPIO 초기화
  _EXTI_Init(); // EXTI 초기화
  Fram_Init();                    // FRAM 초기화 H/W 초기화
  Fram_Status_Config();   // FRAM 초기화 S/W 초기화
  
  DisplayInitScreen(); // LCD 초기 화면
  int SW1_Flag, SW2_Flag;
  left = (Fram_Read(2023) - 1) * 13;
  right = (Fram_Read(2024) - 1) * 13;
  LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
  LCD_DrawFillRect(20, 107-left, 10, 13 * Fram_Read(2023)); // 폭 10, 층 수만큼의 직사각형 생성
  LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
  LCD_DrawFillRect(120, 107-right, 10, 13 * Fram_Read(2024)); // 폭 10, 층 수 만큼의 직사각형 생성
  
  SW1_Flag=Fram_Read(2023), SW2_Flag=Fram_Read(2023);

  Floor_mode = 1; Execute_mode = 0; //. mode 변수 초기화
  Left_E = Fram_Read(2023), Right_E = Fram_Read(2024); // 초기 좌/우측 엘리베이터 위치 Read
  
  while(1){
    switch(KEY_Scan()){ // SW 입력
    case 0xFD00: // SW1 입력(출발층)
      BEEP(); // SW1 입력시 부저 1회
      SW1_Flag++; // SW1_Flag 증가
      GPIOG->ODR &= ~0x0004; // LED2 OFF
      GPIOG->ODR |= 0x0002; // LED1 ON
      break;
    case 0xFB00: // SW2 입력(목표층)
      BEEP(); // SW2 입력시 부저 1회
      SW2_Flag++; // SW2_Flag 증가
      GPIOG->ODR &= ~0x0002; // LED1 OFF
      GPIOG->ODR |= 0x0004; // LED2 ON
      break;
    } // switch(KEY_Scan())
    
    /* 출발층 */
    if(SW1_Flag > 6) // 만약 출발층 선택 입력이 6보타 커지면 다시 1로 복귀
      SW1_Flag = 1;
    char Start_floorChar = '0' + SW1_Flag; // 정수형을 아스키코드를 고려하여 문자형으로 변환
    LCD_DisplayChar(27, 8, Start_floorChar); // 출발층 위치에 층 숫자 표시
    
    /* 목표층 */
    if(SW2_Flag > 6) // 만약 목표층 선택 입력이 6보타 커지면 다시 1로 복귀
      SW2_Flag=1;
    char Des_floorChar = '0' + SW2_Flag; // 정수형을 아스키코드를 고려하여 문자형으로 변환
    LCD_DisplayChar(27, 10, Des_floorChar); // 목표층 위치에 층 숫자 표시
    
    Start_FL = SW1_Flag;
    Des_FL = SW2_Flag; // 목표층 == SW2 누른 횟수
    if((Floor_mode == 1) && (Execute_mode == 1)) // 층선택 모드와 실행 모드가 모두 1이면 실행 모드 시행
      ExecuteMode(Start_FL, Des_FL);
  } // While(1)
} // int main(void)

void _GPIO_Init(void){
  // LED (GPIO G) 설정 : Output mode
   RCC->AHB1ENR |= 0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
   GPIOG->MODER |= 0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
   GPIOG->OTYPER &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
   GPIOG->OSPEEDR |= 0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed

   // SW (GPIO H) 설정 : Input mode 
   RCC->AHB1ENR |= 0x00000080;   // RCC_AHB1ENR : GPIOH(bit#7) Enable                     
   GPIOH->MODER &= ~0xFFFF0000;   // GPIOH 8~15 : Input mode (reset state)            
   GPIOH->PUPDR &= ~0xFFFF0000;   // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

   // Buzzer (GPIO F) 설정 : Output mode
   RCC->AHB1ENR |= 0x00000020;   // RCC_AHB1ENR : GPIOF(bit#5) Enable                     
   GPIOF->MODER |= 0x00040000;   // GPIOF 9 : Output mode (0b01)                  
   GPIOF->OTYPER &= ~0x0200;   // GPIOF 9 : Push-pull     
   GPIOF->OSPEEDR |= 0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed
}

/* GLCD 초기화면 설정 함수 */
void DisplayInitScreen(void){
  LCD_Clear(RGB_WHITE); // 화면 클리어 : 하얀색
  LCD_SetFont(&Gulim8); // 글자 폰트 : 굴림 8
  LCD_SetBackColor(RGB_WHITE); //  글자 배경색 : 하얀색
  LCD_SetTextColor(RGB_BLACK); // 글자색 : 검정색
  LCD_DisplayText(0, 0, "MC-Elevator(KJW)"); // (0,0)에 글 입력 [Title]
  LCD_DisplayText(27, 9, ">"); // GLCD (27, 9)에 > Display
  
  LCD_SetTextColor(RGB_BLUE); // 글자색 : 파란색
  LCD_DisplayText(28, 4, "1"); // Left 1F
  LCD_DisplayText(27, 4, "2"); // Left 2F
  LCD_DisplayText(26, 4, "3"); // Left 3F
  LCD_DisplayText(25, 4, "4"); // Left 4F
  LCD_DisplayText(24, 4, "5"); // Left 5F
  LCD_DisplayText(23, 4, "6"); // Left 6F
  LCD_DisplayText(25, 8, "L-E"); // 초기 엘리베이터 : L-E Display
  
  LCD_SetTextColor(RGB_GREEN); // 글자색 : 초록색
  LCD_DisplayText(28, 14, "1"); // Right 1F
  LCD_DisplayText(27, 14, "2"); // Right 2F
  LCD_DisplayText(26, 14, "3"); // Right 3F
  LCD_DisplayText(25, 14, "4"); // Right 4F
  LCD_DisplayText(24, 14, "5"); // Right 5F
  LCD_DisplayText(23, 14, "6"); // Right 6F
  
  LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
  LCD_DisplayText(23, 8, "FL"); // 초기 실행 모드 FL Display
  LCD_DisplayText(26, 9, "S"); //  현재 동작 상태 : STOP Display
  LCD_DisplayText(27, 8, "1"); // 초기 좌측 엘리베이터 위치 1
  LCD_DisplayText(27, 10, "1"); // 초기 우측 엘리베이터 위치 1
  
  LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
  LCD_DrawFillRect(20, 107, 10, 13); // 폭 10, 높이 13 크기의 직사각형 생성
  LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
  LCD_DrawFillRect(120, 107, 10, 13); // 폭 10, 높이 13 크기의 직사각형 생성
  
  GPIOG->ODR &= ~0x00FF; // 모든 LED OFF
  GPIOG->ODR |= 0x0080; // 초기 LED7 ON
}

/* EXTI 초기 설정 */
void _EXTI_Init(void){
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR |= 0x00004000; // Enable System Configuration Controller Clock
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH PIN8~PIN15 Input mode (reset state)
  
  /*EXTI8*/
  SYSCFG->EXTICR[2] |= 0x0007; // EXTI8에 대한 소스 입력은 GPIOH로 설정
  EXTI->RTSR |= 0x0100; // EXTI8: Rising Trigger Enable
  EXTI->IMR |= 0x0100; // EXTI8 인터럽트 mask (Interrupt Enable) 설정
  NVIC->ISER[0] |= 1<<23; // Enable 'Global Interrupt EXTI8'
  
  /*EXTI15*/
  SYSCFG->EXTICR[3] |= 0x7000; // EXTI15에 대한 소스 입력은 GPIOH로 설정
  EXTI->RTSR |= 0x8000; // EXTI15: Rising Trigger Enable
  EXTI->IMR |= 0x8000; // EXTI15 인터럽트 mask (Interrupt Enable) 설정
  NVIC->ISER[1] |= 1<<8; // Enable 'Global Interrupt EXTI15'
}

/* EXTI8 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void){
  if(EXTI->PR & 0x0100){
    EXTI->PR |= 0x0100;
    Execute_mode = 1;
  } // if(EXTI->PR & 0x0100)
} // void EXTI9_5_IRQHandler(void)

/* EXTI15 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){ // EXTI15 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x8000; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    if(Execute_mode == 0)
      Hold_mode = 0;
    else
      Hold_mode = 1; // Execute mode가 실행 중일 때만 Hold mode Flag 발생
  } // if(EXTI->PR & 0x8000)
} // void EXTI15_10_IRQHandler(void)

/* ExecuteMode 함수 */
void ExecuteMode(int Start_FL, int Des_FL){
  LCD_DisplayText(23, 8, "EX"); // Execute mode Text Display
  GPIOG->ODR &= ~0x0002; GPIOG->ODR &= ~0x0004; GPIOG->ODR &= ~0x0040; GPIOG->ODR &= ~0x0080; // Led 1, 2, 6, 7 OFF
  GPIOG->ODR |= 0x0001; // Led 0 ON
  if(abs(Left_E - Start_FL) > abs(Right_E-Start_FL)){ // 출발층이 오른쪽 엘리베이터와 더 가까울 경우 오른쪽 엘리베이터 이동
    MOVE_Right_E(Start_FL, Des_FL);
  } // if(abs(Left_E - SW1_Flag) > abs(Right_E-SW1_Flag))
  else if(abs(Left_E-Start_FL) <= abs(Right_E-Start_FL)){ // 출발층이 왼쪽 엘리베이터와 더 가까울 경우 왼쪽 엘레베이터 이동
    MOVE_Left_E(Start_FL, Des_FL);
  } // else if(abs(Left_E-SW1_Flag) <= abs(Right_E-SW1_Flag))
} // void ExecuteMode(int Des_FL)

/* HoldMode 함수 */
void HoldMode(void){
  LCD_DisplayText(23, 8, "HD"); // LCD에 HD모드로 전환됐음을 Display
  GPIOG->ODR &= ~0x0001; GPIOG->ODR &= ~0x0080; // LED0, 7 OFF
  GPIOG->ODR |= 0x0040; // LED6 ON
  for(int i = 0; i < 10; i++){ // 0.5초 간격으로 10회 부저 발생
    BEEP();
    DelayMS(500);
  } // for(int i = 0; i < 10; i++)
  LCD_DisplayText(23, 8, "EX"); // LCD에 EX모드로 전환됐음을 Display
  GPIOG->ODR &= ~0x0040; GPIOG->ODR &= ~0x0080; // LED6, 7 OFF
  GPIOG->ODR |= 0x0001; // LED0 ON
  Hold_mode = 0; // Hold mode Flag 0으로 복귀
} // void HoldMode(void)

/* 왼쪽 엘레베이터 동작 함수 */
void MOVE_Left_E(int Start_FL, int Des_FL) {
  LCD_SetTextColor(RGB_BLUE); // 글자색 : 파란색
  LCD_DisplayText(25, 8, "L-E"); // LCD에 Left Elevator 동작함을 Display
  
  // 현재 층이 출발 층과 같은 경우 1초 후 목표 층으로 이동
  if (Left_E == Start_FL) {
    DelayMS(1000);  
    if(Left_E < Des_FL){ // 상승 조건
      for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Left_E = floor; // 현재 층 Left_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
        left += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
        if (Left_E == Des_FL) { // 목표층에 도착할 경우
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop Display
          DelayMS(500); // 0.5초 대기
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
    } // if(Left_E < Des_FL)
    
    else if(Left_E > Des_FL){ // 하강 조건
      for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Left_E = floor; // 현재 층 Left_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "D"); // Down Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
        left -= 13; // Elevator 동작 막대기 좌표 Update
        if (Left_E == Des_FL) { // 목표층에 도착할 경우
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop Display
          DelayMS(500); // 0.5초 대기
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
    } // else if(Left_E > Des_FL)
  } // if (Left_E == SW1_Flag)
  
  // 현재 층과다른 층 입력 시 동작
  else if (Left_E != Start_FL){
    LCD_SetTextColor(RGB_BLUE); // 글자색 : 파란색
    LCD_DisplayText(25,8,"L-E"); // LCD에 Left Elevator 동작함을 Display
    
    if(Left_E < Des_FL){ // 상승 조건
      if(Left_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor = Left_E + 1; floor <= Start_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26,9,"U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
          left += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26,9,"S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          left -= 13; // Elevator 동작 막대기 좌표 Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26,9,"S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Left_E = floor; // 현재 층 Left_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
        left += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Left_E == Des_FL)
       } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
    } // if(Left_E < Des_FL)
    
    /* 하강 조건 */
    else if(Left_E > Des_FL){
       if(Left_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor =Left_E + 1; floor <= Start_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
          left += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          left -= 13; // Elevator 동작 막대기 좌표 Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      
      for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Left_E = floor; // 현재 층 Left_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "D"); // Down 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
        left -= 13; // Elevator 동작 막대기 좌표 Update
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
    } // else if(Left_E > Des_FL)
    
    /* Left_E == Des_FL*/
    else {
      if(Left_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor =Left_E + 1; floor <= Start_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
          left += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26,9,"S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          left -= 13; // Elevator 동작 막대기 좌표 Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      
      if(Left_E < Des_FL) {
        for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동    
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Left_E = floor; // 현재 층 Left_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_BLUE); // 도형색 : 파란색
        left += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Left_E == Des_FL)
       } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
      } // if(Left_E < Des_FL)
      
      else if(Left_E > Des_FL) {
        for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // 현재 층 Left_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Left_E = floor; // 현재 층 Left_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator 동작 막대기 Draw
          left -= 13; // Elevator 동작 막대기 좌표 Update
          if (Left_E == Des_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Left_E == Des_FL)
        } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
      } // else if(Left_E > Des_FL)
    } // else
  } // else if (Left_E != SW1_Flag)
  Fram_Write(2023, Left_E); // FRAM WRITE
  LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
  LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
  LCD_DisplayText(23, 8, "FL"); // 층 선택 모드 변경 (EX->FL)
  GPIOG->ODR &= ~0x00FF; // 모든 LED OFF
  GPIOG->ODR |= 0x0080; // LED7 ON
  BEEP(); DelayMS(500); BEEP(); DelayMS(500); BEEP(); DelayMS(500); // 부저 3회 출림
  Execute_mode = 0; // Execute mode Flag 0
} // void MOVE_Left_E(int Des_FL)

/* 오른쪽 엘레베이터 동작 함수 */
void MOVE_Right_E(int Start_FL, int Des_FL){
  LCD_SetTextColor(RGB_GREEN); // 글자색 : 초록색
  LCD_DisplayText(25, 8, "R-E");
  
  // 현재 층이 출발 층과 같은 경우 1초 후 목표 층으로 이동
  if (Right_E == Start_FL) {
    DelayMS(1000);  
    if(Right_E < Des_FL){ // 상승 조건
      for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Right_E = floor; // 현재 층 Right_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
        right += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E+ 1; floor <= Des_FL; floor++)
    } // if(Right_E< Des_FL)
    
    else if(Right_E > Des_FL){ // 하강 조건
      for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Right_E = floor; // 현재 층 Right_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "D"); // Down 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
        right -= 13; // Elevator 동작 막대기 좌표 Update
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E- 1; floor >= Des_FL; floor--)
    } // else if(Right_E> Des_FL)
  } // if (Right_E == SW1_Flag)
  
  // 다른 층 입력 시 동작
  else if (Right_E != Start_FL){
    if(Right_E < Des_FL){ // 상승 조건
      if(Right_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
          right += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          right -= 13; // Elevator 동작 막대기 좌표 Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Right_E = floor; // 현재 층 Right_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
        right += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E + 1; floor <= Des_FL; floor++)
    } // if(Right_E < Des_FL)
    
    /* 하강 조건 */
    else if(Right_E > Des_FL){
       if(Right_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
          right += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          right -= 13; // Elevator 동작 막대기 좌표 Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Right_E = floor; // 현재 층 Right_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "D"); // Down 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
        right -= 13; // Elevator 동작 막대기 좌표 Update
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E - 1; floor >= Des_FL; floor--)
    } // else if(Right_E > Des_FL)
    
    /* Right_E == Des_FL */
    else {
      if(Right_E < Start_FL) { // 입력층보다 현재 위치가 낮을 경우
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "U"); // Up 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
          right += 13; // Elevator 동작 막대기 좌표 Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // 입력층보다 현재 위치가 높을 경우
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          right -= 13; // Elevator 동작 막대기 좌표 Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      if(Right_E < Des_FL) { // 현재층이 목표층보다 낮을 경우
        for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
          HoldMode();
        Right_E = floor; // 현재 층 Right_E 업데이트
        LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
        LCD_DisplayText(26, 9, "U"); // Up 동작 Display
        DelayMS(500); // 0.5초 대기
        LCD_SetBrushColor(RGB_GREEN); // 도형색 : 초록색
        right += 13; // Elevator 동작 막대기 좌표 Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5초 대기
          LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
          DelayMS(500); // 0.5초 대기
        } // if (Right_E == Des_FL)
       } // for (int floor = Right_E + 1; floor <= Des_FL; floor++)
      } // if(Right_E < Des_FL) left Right_E
      
      else if(Right_E > Des_FL) { // 현재층이 목표층보다 높을 경우
        for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // 현재 층 Right_E에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
          if(Hold_mode) // Hold mode 발생 시 Hold mode 동작
            HoldMode();
          Right_E = floor; // 현재 층 Right_E 업데이트
          LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
          LCD_DisplayText(26, 9, "D"); // Down 동작 Display
          DelayMS(500); // 0.5초 대기
          LCD_SetBrushColor(RGB_WHITE); // 도형색 : 하얀색
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator 동작 막대기 Draw
          right -= 13; // Elevator 동작 막대기 좌표 Update
          if (Right_E == Des_FL) {
            DelayMS(500); // 0.5초 대기
            LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
            DelayMS(500); // 0.5초 대기
          } // if (Right_E == Des_FL)
        } // for (int floor = Right_E - 1; floor >= Des_FL; floor--)
      } // else if(Right_E > Des_FL)
    } // else
  } // else if (Right_E != SW1_Flag)
  Fram_Write(2024, Right_E); // FRAM WRITE
  LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
  LCD_DisplayText(26, 9, "S"); // Stop 동작 Display
  LCD_DisplayText(23, 8, "FL"); // 층 선택 모드 변경 (EX->FL)
  GPIOG->ODR &= ~0x00FF; // 모든 LED OFF
  GPIOG->ODR |= 0x0080; // LED7 ON
  BEEP(); DelayMS(500); BEEP(); DelayMS(500); BEEP(); DelayMS(500); // 부저 3회 출림
  Execute_mode = 0; // Execute mode Flag 0
} // void MOVE_Right_E(int Des_FL)

/* Switch가 입력되었는지 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수  */ 
uint8_t key_flag = 0;
uint16_t KEY_Scan(void){   // input key SW0 - SW7
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

/* Buzzer: Beep for 30 ms */
void BEEP(void)         
{    
   GPIOF->ODR |=  0x0200;   // PF9 'H' Buzzer on
   DelayMS(30);      // Delay 30 ms
   GPIOF->ODR &= ~0x0200;   // PF9 'L' Buzzer off
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