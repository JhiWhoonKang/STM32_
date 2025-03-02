/////////////////////////////////////////////////////////////
// ������: �����ı���2023_HW3_����������
// ��������: ����(0~6��)�� ��ġ�� ���������Ϳ���, ��ǥ�� ����ġ�� �Է��Ͽ� ��ǥ������ �̵��ϵ��� �����ϴ� ���α׷� �ۼ� 
// ����� �ϵ����(���): GPIO, Joy-stick, EXTI, GLCD ...
// ������: 2023. 5. 29
// ������ Ŭ����: �����Ϲ�
// �й�: 2019132002
// �̸�: ������
///////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayInitScreen(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void MOVE(int Des_FL); // ���������� ���� �Լ� ����

uint16_t KEY_SCAN(void);
uint8_t FLAG0, FLAG1, FLAG2, FLAG3, FLAG4, FLAG5, FLAG6 ;
int Cur_FL=0; // ���� �� ��ġ ���� ����
int Des_FL; // ��ǥ �� ��ġ ���� ����

int main(void)
{
  LCD_Init(); // LCD ��� �ʱ�ȭ
  _GPIO_Init(); // GPIO �ʱ�ȭ
  _EXTI_Init(); // EXTI �ʱ�ȭ
  
  DisplayInitScreen(); // LCD �ʱ� ȭ��
  
  while(1){
    switch(KEY_SCAN()){ // SW �Է�
    case 0xFE00: // SW0 �Է�
      FLAG0+=1;
      MOVE(0); // 0������ �̵�
      break;
    case 0xFD00: // SW1 �Է�
      FLAG1+=1;
      MOVE(1); // 1������ �̵�
      break;
    case 0xFB00: // SW2 �Է�
      FLAG2+=1;
      MOVE(2); // 2������ �̵�
      break;
    case 0xF700: // SW3 �Է�
      FLAG3+=1;
      MOVE(3); // 3������ �̵�
      break;
    case 0xEF00: // SW4 �Է�
      FLAG4+=1;
      MOVE(4); // 4������ �̵�
      break;
    case 0xDF00: // SW5 �Է�
      FLAG5+=1;
      MOVE(5); // 5������ �̵�
      break;
    case 0xBF00: // SW6 �Է�
      FLAG6+=1;
      MOVE(6); // 6������ �̵�
      break;
    }
  }
}

/* GLCD �ʱ�ȭ�� ���� �Լ� */
void DisplayInitScreen(void){
  LCD_Clear(RGB_YELLOW); // ȭ�� Ŭ���� : �����
  LCD_SetFont(&Gulim8); // ���� ��Ʈ : ���� 8
  LCD_SetBackColor(RGB_YELLOW); //  ���� ���� : �����
  LCD_SetTextColor(RGB_BLUE); // ���ڻ� : �Ķ�
  LCD_DisplayText(0,0,"MECHA Elevator(KJW)"); // (0,0)�� �� �Է� [Title]
  LCD_SetTextColor(RGB_BLACK); // ���ڻ� : ������
  LCD_DisplayText(1,0,"Cur FL: "); // (1,0)�� �� �Է� [������]
  LCD_DisplayText(2,0,"Des FL: "); // (2,0)�� �� �Է� [��ǥ��]
  LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
  LCD_DisplayText(1,7,"0"); // (1,7)�� �� �Է� [�ʱ� ���� ��ġ: 0]
  LCD_DisplayText(2,7,"-"); // (2,7)�� �� �Է� [�ʱ� ��ǥ ��ġ: -]
  GPIOG->ODR |= 0x0001; // �ʱ� LED0 ON
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer)) �ʱ� ���� */
void _GPIO_Init(void){
  // LED (GPIO G) ���� : Output mode
  RCC->AHB1ENR |= 0x00000040; // RCC_AHB1ENR : GPIOG(bit#6) Enable
  GPIOG->MODER |= 0x00005555; // GPIOG 0~7 : Output mode (0b01)
  GPIOG->OTYPER &= ~0x00FF; // GPIOG 0~7 : Push-pull  (GP8~15:reset state)
  GPIOG->OSPEEDR |= 0x00005555; // GPIOG 0~7 : Output speed 25MHZ Medium speed
  
  // SW (GPIO H) ���� : Input mode
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR : GPIOH(bit#7) Enable
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH 8~15 : Input mode (reset state)
  GPIOH->PUPDR &= ~0xFFFF0000; // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
  
  // Buzzer (GPIO F) ���� : Output mode
  RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable
  GPIOF->MODER |= 0x00040000; // GPIOF 9 : Output mode (0b01)
  GPIOF->OTYPER &= ~0x0200; // GPIOF 9 : Push-pull
  GPIOF->OSPEEDR |= 0x00040000; // GPIOF 9 : Output speed 25MHZ Medium speed
}

/* EXTI �ʱ� ���� */
void _EXTI_Init(void){
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR |= 0x00004000; // Enable System Configuration Controller Clock
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH PIN8~PIN15 Input mode (reset state)
  SYSCFG->EXTICR[3] |= 0x7000; // EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����
  EXTI->RTSR |= 0x8000; // EXTI15: Rising Trigger Enable
  EXTI->IMR |= 0X8000; // EXTI15 ���ͷ�Ʈ mask (Interrupt Enable) ����
  NVIC->ISER[1] |= 1<<8; // Enable 'Global Interrupt EXTI15'
}

/* EXTI15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){ // EXTI15 Interrupt Pending(�߻�) ����?
    EXTI->PR |= 0x8000; // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    GPIOG->ODR |= 0x0080; // LED8 ON
    BEEP(); DelayMS(1000);  BEEP(); // // 1�� ���� Buzzer 2ȸ �︲
    DelayMS(5000); // 5�� ���
    GPIOG->ODR &= ~0x0080; // LED8 OFF
    BEEP(); DelayMS(1000);  BEEP();// 1�� ���� Buzzer 2ȸ �︲
  } // if(EXTI->PR & 0x8000)
}

/* ���������� ���� �Լ� */
void MOVE(int Des_FL){  
  // �ٸ� �� �Է� �� ����
  if (Cur_FL != Des_FL){
    BEEP(); // Buzzer 1ȸ �︲
    if(FLAG0==1){
      LCD_DisplayText(2,7,"0"); // ��ǥ�� ǥ��
      FLAG0=0;
    } // if(FLAG0==1)
    else if(FLAG1==1){
      LCD_DisplayText(2,7,"1"); // ��ǥ�� ǥ��
      FLAG1=0;
    } // else if(FLAG1==1)
    else if(FLAG2==1){
      LCD_DisplayText(2,7,"2"); // ��ǥ�� ǥ��
      FLAG2=0;
    } // else if(FLAG2==1)
    else if(FLAG3==1){
      LCD_DisplayText(2,7,"3"); // ��ǥ�� ǥ��
      FLAG3=0;
    } //  else if(FLAG3==1)
    else if(FLAG4==1){
      LCD_DisplayText(2,7,"4"); // ��ǥ�� ǥ��
      FLAG4=0;
    } //  else if(FLAG4==1)
    else if(FLAG5==1){
      LCD_DisplayText(2,7,"5"); // ��ǥ�� ǥ��
      FLAG5=0;
    } // else if(FLAG5==1)
    else if(FLAG6==1){
      LCD_DisplayText(2,7,"6"); // ��ǥ�� ǥ��
      FLAG6=0;
    } // else if(FLAG6==1)
    
    if(Cur_FL<Des_FL){ // ��� ����
      for (int floor = Cur_FL + 1; floor <= Des_FL; floor++){ // ���� �� Cur_FL���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        Cur_FL=floor; // ���� �� Cur_FL ������Ʈ
        DelayMS(1000); // 1�� ���
        GPIOG->ODR &= ~0x00FF; // ��� LED OFF
        GPIOG->ODR |= (1 << Cur_FL); // ���� �� Cur_FL�� �°� bit �̵��Ͽ� LED ON
        char floorChar = '0' + Cur_FL; // ���� ���� �ƽ�Ű �ڵ带 ����Ͽ� ���ڷ� ��ȯ
        LCD_DisplayChar(1, 7, floorChar); // ���� �� LCD ������Ʈ
      } // for (int floor = Cur_FL + 1; floor <= Des_FL; floor++)
      BEEP();  DelayMS(1000);  BEEP();  DelayMS(1000);  BEEP(); // 1�� ���� Buzzer 3ȸ �︲
    } // if(Cur_FL<Des_FL)
    else if (Cur_FL > Des_FL){ // �ϰ� ����
      for (int floor = Cur_FL - 1; floor >= Des_FL; floor--) { // ���� �� Cur_FL���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        Cur_FL = floor; // ���� �� Cur_FL ������Ʈ
        DelayMS(1000); // 1�� ���
        GPIOG->ODR &= ~0x00FF;
        GPIOG->ODR |= (1 << Cur_FL); // ��� LED OFF
        char floorChar = '0' + Cur_FL; // ���� ���� �ƽ�Ű �ڵ带 ����Ͽ� ���ڷ� ��ȯ
        LCD_DisplayChar(1, 7, floorChar); // ���� �� LCD ������Ʈ
      } // for (int floor = Cur_FL - 1; floor >= Des_FL; floor--)
      BEEP();  DelayMS(1000);  BEEP();  DelayMS(1000);  BEEP(); // 1�� ���� Buzzer 3ȸ �︲
    } // else if (Cur_FL > Des_FL)
    LCD_DisplayText(2,7,"-"); // ��ǥ ���� �����ϸ� Des_FL '-' ǥ��
  } // if (Cur_FL != Des_FL)
  
  // ���� �� �Է� �� �̵���
  else{}
} // void MOVE(int Des_FL)

/* Switch�� �ԷµǾ����� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */ 
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