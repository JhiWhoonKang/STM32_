/////////////////////////////////////////////////////////////
// 과제명: 마이컴구조2023_HW3_엘레베이터
// 과제개요: 빌딩(0~6층)에 설치된 엘리베이터에서, 목표층 스위치를 입력하여 목표층까지 이동하도록 제어하는 프로그램 작성 
// 사용한 하드웨어(기능): GPIO, Joy-stick, EXTI, GLCD ...
// 제출일: 2023. 5. 29
// 제출자 클래스: 수요일반
// 학번: 2019132002
// 이름: 강지훈
///////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayInitScreen(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void MOVE(int Des_FL); // 엘레베이터 동작 함수 선언

uint16_t KEY_SCAN(void);
uint8_t FLAG0, FLAG1, FLAG2, FLAG3, FLAG4, FLAG5, FLAG6 ;
int Cur_FL=0; // 현재 층 위치 전역 변수
int Des_FL; // 목표 층 위치 전역 변수

int main(void)
{
  LCD_Init(); // LCD 모듈 초기화
  _GPIO_Init(); // GPIO 초기화
  _EXTI_Init(); // EXTI 초기화
  
  DisplayInitScreen(); // LCD 초기 화면
  
  while(1){
    switch(KEY_SCAN()){ // SW 입력
    case 0xFE00: // SW0 입력
      FLAG0+=1;
      MOVE(0); // 0층으로 이동
      break;
    case 0xFD00: // SW1 입력
      FLAG1+=1;
      MOVE(1); // 1층으로 이동
      break;
    case 0xFB00: // SW2 입력
      FLAG2+=1;
      MOVE(2); // 2층으로 이동
      break;
    case 0xF700: // SW3 입력
      FLAG3+=1;
      MOVE(3); // 3층으로 이동
      break;
    case 0xEF00: // SW4 입력
      FLAG4+=1;
      MOVE(4); // 4층으로 이동
      break;
    case 0xDF00: // SW5 입력
      FLAG5+=1;
      MOVE(5); // 5층으로 이동
      break;
    case 0xBF00: // SW6 입력
      FLAG6+=1;
      MOVE(6); // 6층으로 이동
      break;
    }
  }
}

/* GLCD 초기화면 설정 함수 */
void DisplayInitScreen(void){
  LCD_Clear(RGB_YELLOW); // 화면 클리어 : 노란색
  LCD_SetFont(&Gulim8); // 글자 폰트 : 굴림 8
  LCD_SetBackColor(RGB_YELLOW); //  글자 배경색 : 노란색
  LCD_SetTextColor(RGB_BLUE); // 글자색 : 파랑
  LCD_DisplayText(0,0,"MECHA Elevator(KJW)"); // (0,0)에 글 입력 [Title]
  LCD_SetTextColor(RGB_BLACK); // 글자색 : 검정색
  LCD_DisplayText(1,0,"Cur FL: "); // (1,0)에 글 입력 [현재층]
  LCD_DisplayText(2,0,"Des FL: "); // (2,0)에 글 입력 [목표층]
  LCD_SetTextColor(RGB_RED); // 글자색 : 빨간색
  LCD_DisplayText(1,7,"0"); // (1,7)에 글 입력 [초기 현재 위치: 0]
  LCD_DisplayText(2,7,"-"); // (2,7)에 글 입력 [초기 목표 위치: -]
  GPIOG->ODR |= 0x0001; // 초기 LED0 ON
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer)) 초기 설정 */
void _GPIO_Init(void){
  // LED (GPIO G) 설정 : Output mode
  RCC->AHB1ENR |= 0x00000040; // RCC_AHB1ENR : GPIOG(bit#6) Enable
  GPIOG->MODER |= 0x00005555; // GPIOG 0~7 : Output mode (0b01)
  GPIOG->OTYPER &= ~0x00FF; // GPIOG 0~7 : Push-pull  (GP8~15:reset state)
  GPIOG->OSPEEDR |= 0x00005555; // GPIOG 0~7 : Output speed 25MHZ Medium speed
  
  // SW (GPIO H) 설정 : Input mode
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR : GPIOH(bit#7) Enable
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH 8~15 : Input mode (reset state)
  GPIOH->PUPDR &= ~0xFFFF0000; // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
  
  // Buzzer (GPIO F) 설정 : Output mode
  RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable
  GPIOF->MODER |= 0x00040000; // GPIOF 9 : Output mode (0b01)
  GPIOF->OTYPER &= ~0x0200; // GPIOF 9 : Push-pull
  GPIOF->OSPEEDR |= 0x00040000; // GPIOF 9 : Output speed 25MHZ Medium speed
}

/* EXTI 초기 설정 */
void _EXTI_Init(void){
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR |= 0x00004000; // Enable System Configuration Controller Clock
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH PIN8~PIN15 Input mode (reset state)
  SYSCFG->EXTICR[3] |= 0x7000; // EXTI15에 대한 소스 입력은 GPIOH로 설정
  EXTI->RTSR |= 0x8000; // EXTI15: Rising Trigger Enable
  EXTI->IMR |= 0X8000; // EXTI15 인터럽트 mask (Interrupt Enable) 설정
  NVIC->ISER[1] |= 1<<8; // Enable 'Global Interrupt EXTI15'
}

/* EXTI15 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){ // EXTI15 Interrupt Pending(발생) 여부?
    EXTI->PR |= 0x8000; // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    GPIOG->ODR |= 0x0080; // LED8 ON
    BEEP(); DelayMS(1000);  BEEP(); // // 1초 간격 Buzzer 2회 울림
    DelayMS(5000); // 5초 대기
    GPIOG->ODR &= ~0x0080; // LED8 OFF
    BEEP(); DelayMS(1000);  BEEP();// 1초 간격 Buzzer 2회 울림
  } // if(EXTI->PR & 0x8000)
}

/* 엘레베이터 동작 함수 */
void MOVE(int Des_FL){  
  // 다른 층 입력 시 동작
  if (Cur_FL != Des_FL){
    BEEP(); // Buzzer 1회 울림
    if(FLAG0==1){
      LCD_DisplayText(2,7,"0"); // 목표층 표시
      FLAG0=0;
    } // if(FLAG0==1)
    else if(FLAG1==1){
      LCD_DisplayText(2,7,"1"); // 목표층 표시
      FLAG1=0;
    } // else if(FLAG1==1)
    else if(FLAG2==1){
      LCD_DisplayText(2,7,"2"); // 목표층 표시
      FLAG2=0;
    } // else if(FLAG2==1)
    else if(FLAG3==1){
      LCD_DisplayText(2,7,"3"); // 목표층 표시
      FLAG3=0;
    } //  else if(FLAG3==1)
    else if(FLAG4==1){
      LCD_DisplayText(2,7,"4"); // 목표층 표시
      FLAG4=0;
    } //  else if(FLAG4==1)
    else if(FLAG5==1){
      LCD_DisplayText(2,7,"5"); // 목표층 표시
      FLAG5=0;
    } // else if(FLAG5==1)
    else if(FLAG6==1){
      LCD_DisplayText(2,7,"6"); // 목표층 표시
      FLAG6=0;
    } // else if(FLAG6==1)
    
    if(Cur_FL<Des_FL){ // 상승 조건
      for (int floor = Cur_FL + 1; floor <= Des_FL; floor++){ // 현재 층 Cur_FL에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        Cur_FL=floor; // 현재 층 Cur_FL 업데이트
        DelayMS(1000); // 1초 대기
        GPIOG->ODR &= ~0x00FF; // 모든 LED OFF
        GPIOG->ODR |= (1 << Cur_FL); // 현재 층 Cur_FL에 맞게 bit 이동하여 LED ON
        char floorChar = '0' + Cur_FL; // 정수 값을 아스키 코드를 고려하여 문자로 전환
        LCD_DisplayChar(1, 7, floorChar); // 현재 층 LCD 업데이트
      } // for (int floor = Cur_FL + 1; floor <= Des_FL; floor++)
      BEEP();  DelayMS(1000);  BEEP();  DelayMS(1000);  BEEP(); // 1초 간격 Buzzer 3회 울림
    } // if(Cur_FL<Des_FL)
    else if (Cur_FL > Des_FL){ // 하강 조건
      for (int floor = Cur_FL - 1; floor >= Des_FL; floor--) { // 현재 층 Cur_FL에서 목표 층 Des_FL까지 아래 조건을 충족하며 한 층씩 이동
        Cur_FL = floor; // 현재 층 Cur_FL 업데이트
        DelayMS(1000); // 1초 대기
        GPIOG->ODR &= ~0x00FF;
        GPIOG->ODR |= (1 << Cur_FL); // 모든 LED OFF
        char floorChar = '0' + Cur_FL; // 정수 값을 아스키 코드를 고려하여 문자로 전환
        LCD_DisplayChar(1, 7, floorChar); // 현재 층 LCD 업데이트
      } // for (int floor = Cur_FL - 1; floor >= Des_FL; floor--)
      BEEP();  DelayMS(1000);  BEEP();  DelayMS(1000);  BEEP(); // 1초 간격 Buzzer 3회 울림
    } // else if (Cur_FL > Des_FL)
    LCD_DisplayText(2,7,"-"); // 목표 층에 도착하면 Des_FL '-' 표시
  } // if (Cur_FL != Des_FL)
  
  // 같은 층 입력 시 미동작
  else{}
} // void MOVE(int Des_FL)

/* Switch가 입력되었는지 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수  */ 
uint8_t key_flag = 0;
uint16_t KEY_SCAN(void){	// input key SW0 - SW7
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
void BEEP(void){
  GPIOF->ODR |= 0x0200;
  DelayMS(30);
  GPIOF->ODR &= ~0x0200;
}

void DelayMS(unsigned short wMS){
  register unsigned short i;
  for(i=0;i<wMS;i++)
    DelayUS(1000);
}

void DelayUS(unsigned short wUS){
  volatile int Dly = (int)wUS*17;
  for(; Dly; Dly--);
}