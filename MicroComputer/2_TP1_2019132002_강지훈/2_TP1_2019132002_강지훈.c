/////////////////////////////////////////////////////////////
// ������: �����ı���2023_TP1_TWO ELEVATOR
// ��������: 6�� �ǹ��� �δ��� ���������� ��ġ, ��������� ��ǥ�� ����ġ(1~6��)�� �����ϸ� ���������Ͱ� �̵��ϵ��� ���α׷� �ۼ�.
//              �̵��� �̵� ������ ũ�� ��ȭ�� ǥ��
// ����� �ϵ����(���): GPIO, Joy-stick, EXTI, GLCD , FRAM...
// ������: 2023. 6. 06
// ������ Ŭ����: �����Ϲ�
// �й�: 2019132002
// �̸�: ������
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
void MOVE_Right_E(int Start_FL, int Des_FL); // ���� ���������� ���� �Լ� ����
void MOVE_Left_E(int Start_FL, int Des_FL); // ���� ���������� ���� �Լ� ����
void ExecuteMode(int Start_FL, int Des_FL); // ���������� Execute mode �Լ� ����
void HoldMode(void); // ���������� Hold mode �Լ� ����
uint16_t KEY_Scan(void);

uint8_t Floor_mode = 0, Hold_mode = 0, Execute_mode = 0; // ���� ��� ����
uint8_t Left_E = 1, Right_E = 1, Start_FL = 1, Des_FL = 1; // ��/���� ���� �� �� ��ǥ �� ����
uint8_t left = 0, right = 0; //  ��/���� �̵����� ��ġ ����

int main(void)
{
  LCD_Init(); // LCD ��� �ʱ�ȭ
  _GPIO_Init(); // GPIO �ʱ�ȭ
  _EXTI_Init(); // EXTI �ʱ�ȭ
  Fram_Init();                    // FRAM �ʱ�ȭ H/W �ʱ�ȭ
  Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
  
  DisplayInitScreen(); // LCD �ʱ� ȭ��
  int SW1_Flag, SW2_Flag;
  left = (Fram_Read(2023) - 1) * 13;
  right = (Fram_Read(2024) - 1) * 13;
  LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
  LCD_DrawFillRect(20, 107-left, 10, 13 * Fram_Read(2023)); // �� 10, �� ����ŭ�� ���簢�� ����
  LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
  LCD_DrawFillRect(120, 107-right, 10, 13 * Fram_Read(2024)); // �� 10, �� �� ��ŭ�� ���簢�� ����
  
  SW1_Flag=Fram_Read(2023), SW2_Flag=Fram_Read(2023);

  Floor_mode = 1; Execute_mode = 0; //. mode ���� �ʱ�ȭ
  Left_E = Fram_Read(2023), Right_E = Fram_Read(2024); // �ʱ� ��/���� ���������� ��ġ Read
  
  while(1){
    switch(KEY_Scan()){ // SW �Է�
    case 0xFD00: // SW1 �Է�(�����)
      BEEP(); // SW1 �Է½� ���� 1ȸ
      SW1_Flag++; // SW1_Flag ����
      GPIOG->ODR &= ~0x0004; // LED2 OFF
      GPIOG->ODR |= 0x0002; // LED1 ON
      break;
    case 0xFB00: // SW2 �Է�(��ǥ��)
      BEEP(); // SW2 �Է½� ���� 1ȸ
      SW2_Flag++; // SW2_Flag ����
      GPIOG->ODR &= ~0x0002; // LED1 OFF
      GPIOG->ODR |= 0x0004; // LED2 ON
      break;
    } // switch(KEY_Scan())
    
    /* ����� */
    if(SW1_Flag > 6) // ���� ����� ���� �Է��� 6��Ÿ Ŀ���� �ٽ� 1�� ����
      SW1_Flag = 1;
    char Start_floorChar = '0' + SW1_Flag; // �������� �ƽ�Ű�ڵ带 ����Ͽ� ���������� ��ȯ
    LCD_DisplayChar(27, 8, Start_floorChar); // ����� ��ġ�� �� ���� ǥ��
    
    /* ��ǥ�� */
    if(SW2_Flag > 6) // ���� ��ǥ�� ���� �Է��� 6��Ÿ Ŀ���� �ٽ� 1�� ����
      SW2_Flag=1;
    char Des_floorChar = '0' + SW2_Flag; // �������� �ƽ�Ű�ڵ带 ����Ͽ� ���������� ��ȯ
    LCD_DisplayChar(27, 10, Des_floorChar); // ��ǥ�� ��ġ�� �� ���� ǥ��
    
    Start_FL = SW1_Flag;
    Des_FL = SW2_Flag; // ��ǥ�� == SW2 ���� Ƚ��
    if((Floor_mode == 1) && (Execute_mode == 1)) // ������ ���� ���� ��尡 ��� 1�̸� ���� ��� ����
      ExecuteMode(Start_FL, Des_FL);
  } // While(1)
} // int main(void)

void _GPIO_Init(void){
  // LED (GPIO G) ���� : Output mode
   RCC->AHB1ENR |= 0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
   GPIOG->MODER |= 0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
   GPIOG->OTYPER &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
   GPIOG->OSPEEDR |= 0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed

   // SW (GPIO H) ���� : Input mode 
   RCC->AHB1ENR |= 0x00000080;   // RCC_AHB1ENR : GPIOH(bit#7) Enable                     
   GPIOH->MODER &= ~0xFFFF0000;   // GPIOH 8~15 : Input mode (reset state)            
   GPIOH->PUPDR &= ~0xFFFF0000;   // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

   // Buzzer (GPIO F) ���� : Output mode
   RCC->AHB1ENR |= 0x00000020;   // RCC_AHB1ENR : GPIOF(bit#5) Enable                     
   GPIOF->MODER |= 0x00040000;   // GPIOF 9 : Output mode (0b01)                  
   GPIOF->OTYPER &= ~0x0200;   // GPIOF 9 : Push-pull     
   GPIOF->OSPEEDR |= 0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed
}

/* GLCD �ʱ�ȭ�� ���� �Լ� */
void DisplayInitScreen(void){
  LCD_Clear(RGB_WHITE); // ȭ�� Ŭ���� : �Ͼ��
  LCD_SetFont(&Gulim8); // ���� ��Ʈ : ���� 8
  LCD_SetBackColor(RGB_WHITE); //  ���� ���� : �Ͼ��
  LCD_SetTextColor(RGB_BLACK); // ���ڻ� : ������
  LCD_DisplayText(0, 0, "MC-Elevator(KJW)"); // (0,0)�� �� �Է� [Title]
  LCD_DisplayText(27, 9, ">"); // GLCD (27, 9)�� > Display
  
  LCD_SetTextColor(RGB_BLUE); // ���ڻ� : �Ķ���
  LCD_DisplayText(28, 4, "1"); // Left 1F
  LCD_DisplayText(27, 4, "2"); // Left 2F
  LCD_DisplayText(26, 4, "3"); // Left 3F
  LCD_DisplayText(25, 4, "4"); // Left 4F
  LCD_DisplayText(24, 4, "5"); // Left 5F
  LCD_DisplayText(23, 4, "6"); // Left 6F
  LCD_DisplayText(25, 8, "L-E"); // �ʱ� ���������� : L-E Display
  
  LCD_SetTextColor(RGB_GREEN); // ���ڻ� : �ʷϻ�
  LCD_DisplayText(28, 14, "1"); // Right 1F
  LCD_DisplayText(27, 14, "2"); // Right 2F
  LCD_DisplayText(26, 14, "3"); // Right 3F
  LCD_DisplayText(25, 14, "4"); // Right 4F
  LCD_DisplayText(24, 14, "5"); // Right 5F
  LCD_DisplayText(23, 14, "6"); // Right 6F
  
  LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
  LCD_DisplayText(23, 8, "FL"); // �ʱ� ���� ��� FL Display
  LCD_DisplayText(26, 9, "S"); //  ���� ���� ���� : STOP Display
  LCD_DisplayText(27, 8, "1"); // �ʱ� ���� ���������� ��ġ 1
  LCD_DisplayText(27, 10, "1"); // �ʱ� ���� ���������� ��ġ 1
  
  LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
  LCD_DrawFillRect(20, 107, 10, 13); // �� 10, ���� 13 ũ���� ���簢�� ����
  LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
  LCD_DrawFillRect(120, 107, 10, 13); // �� 10, ���� 13 ũ���� ���簢�� ����
  
  GPIOG->ODR &= ~0x00FF; // ��� LED OFF
  GPIOG->ODR |= 0x0080; // �ʱ� LED7 ON
}

/* EXTI �ʱ� ���� */
void _EXTI_Init(void){
  RCC->AHB1ENR |= 0x00000080; // RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR |= 0x00004000; // Enable System Configuration Controller Clock
  GPIOH->MODER &= ~0xFFFF0000; // GPIOH PIN8~PIN15 Input mode (reset state)
  
  /*EXTI8*/
  SYSCFG->EXTICR[2] |= 0x0007; // EXTI8�� ���� �ҽ� �Է��� GPIOH�� ����
  EXTI->RTSR |= 0x0100; // EXTI8: Rising Trigger Enable
  EXTI->IMR |= 0x0100; // EXTI8 ���ͷ�Ʈ mask (Interrupt Enable) ����
  NVIC->ISER[0] |= 1<<23; // Enable 'Global Interrupt EXTI8'
  
  /*EXTI15*/
  SYSCFG->EXTICR[3] |= 0x7000; // EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����
  EXTI->RTSR |= 0x8000; // EXTI15: Rising Trigger Enable
  EXTI->IMR |= 0x8000; // EXTI15 ���ͷ�Ʈ mask (Interrupt Enable) ����
  NVIC->ISER[1] |= 1<<8; // Enable 'Global Interrupt EXTI15'
}

/* EXTI8 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void){
  if(EXTI->PR & 0x0100){
    EXTI->PR |= 0x0100;
    Execute_mode = 1;
  } // if(EXTI->PR & 0x0100)
} // void EXTI9_5_IRQHandler(void)

/* EXTI15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){ // EXTI15 Interrupt Pending(�߻�) ����?
    EXTI->PR |= 0x8000; // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    if(Execute_mode == 0)
      Hold_mode = 0;
    else
      Hold_mode = 1; // Execute mode�� ���� ���� ���� Hold mode Flag �߻�
  } // if(EXTI->PR & 0x8000)
} // void EXTI15_10_IRQHandler(void)

/* ExecuteMode �Լ� */
void ExecuteMode(int Start_FL, int Des_FL){
  LCD_DisplayText(23, 8, "EX"); // Execute mode Text Display
  GPIOG->ODR &= ~0x0002; GPIOG->ODR &= ~0x0004; GPIOG->ODR &= ~0x0040; GPIOG->ODR &= ~0x0080; // Led 1, 2, 6, 7 OFF
  GPIOG->ODR |= 0x0001; // Led 0 ON
  if(abs(Left_E - Start_FL) > abs(Right_E-Start_FL)){ // ������� ������ ���������Ϳ� �� ����� ��� ������ ���������� �̵�
    MOVE_Right_E(Start_FL, Des_FL);
  } // if(abs(Left_E - SW1_Flag) > abs(Right_E-SW1_Flag))
  else if(abs(Left_E-Start_FL) <= abs(Right_E-Start_FL)){ // ������� ���� ���������Ϳ� �� ����� ��� ���� ���������� �̵�
    MOVE_Left_E(Start_FL, Des_FL);
  } // else if(abs(Left_E-SW1_Flag) <= abs(Right_E-SW1_Flag))
} // void ExecuteMode(int Des_FL)

/* HoldMode �Լ� */
void HoldMode(void){
  LCD_DisplayText(23, 8, "HD"); // LCD�� HD���� ��ȯ������ Display
  GPIOG->ODR &= ~0x0001; GPIOG->ODR &= ~0x0080; // LED0, 7 OFF
  GPIOG->ODR |= 0x0040; // LED6 ON
  for(int i = 0; i < 10; i++){ // 0.5�� �������� 10ȸ ���� �߻�
    BEEP();
    DelayMS(500);
  } // for(int i = 0; i < 10; i++)
  LCD_DisplayText(23, 8, "EX"); // LCD�� EX���� ��ȯ������ Display
  GPIOG->ODR &= ~0x0040; GPIOG->ODR &= ~0x0080; // LED6, 7 OFF
  GPIOG->ODR |= 0x0001; // LED0 ON
  Hold_mode = 0; // Hold mode Flag 0���� ����
} // void HoldMode(void)

/* ���� ���������� ���� �Լ� */
void MOVE_Left_E(int Start_FL, int Des_FL) {
  LCD_SetTextColor(RGB_BLUE); // ���ڻ� : �Ķ���
  LCD_DisplayText(25, 8, "L-E"); // LCD�� Left Elevator �������� Display
  
  // ���� ���� ��� ���� ���� ��� 1�� �� ��ǥ ������ �̵�
  if (Left_E == Start_FL) {
    DelayMS(1000);  
    if(Left_E < Des_FL){ // ��� ����
      for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Left_E = floor; // ���� �� Left_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
        left += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
        if (Left_E == Des_FL) { // ��ǥ���� ������ ���
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop Display
          DelayMS(500); // 0.5�� ���
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
    } // if(Left_E < Des_FL)
    
    else if(Left_E > Des_FL){ // �ϰ� ����
      for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Left_E = floor; // ���� �� Left_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "D"); // Down Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
        left -= 13; // Elevator ���� ����� ��ǥ Update
        if (Left_E == Des_FL) { // ��ǥ���� ������ ���
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop Display
          DelayMS(500); // 0.5�� ���
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
    } // else if(Left_E > Des_FL)
  } // if (Left_E == SW1_Flag)
  
  // ���� �����ٸ� �� �Է� �� ����
  else if (Left_E != Start_FL){
    LCD_SetTextColor(RGB_BLUE); // ���ڻ� : �Ķ���
    LCD_DisplayText(25,8,"L-E"); // LCD�� Left Elevator �������� Display
    
    if(Left_E < Des_FL){ // ��� ����
      if(Left_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Left_E + 1; floor <= Start_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26,9,"U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
          left += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26,9,"S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          left -= 13; // Elevator ���� ����� ��ǥ Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26,9,"S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Left_E = floor; // ���� �� Left_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
        left += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Left_E == Des_FL)
       } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
    } // if(Left_E < Des_FL)
    
    /* �ϰ� ���� */
    else if(Left_E > Des_FL){
       if(Left_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor =Left_E + 1; floor <= Start_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
          left += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          left -= 13; // Elevator ���� ����� ��ǥ Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      
      for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Left_E = floor; // ���� �� Left_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "D"); // Down ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
        left -= 13; // Elevator ���� ����� ��ǥ Update
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Left_E == Des_FL)
      } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
    } // else if(Left_E > Des_FL)
    
    /* Left_E == Des_FL*/
    else {
      if(Left_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor =Left_E + 1; floor <= Start_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
          left += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26,9,"S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == Start_FL)
        } // for (int floor =Left_E + 1; floor <= SW1_Flag; floor++)
      } //if(Left_E < SW1_Flag)
      
      else if (Left_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Left_E - 1; floor >= Start_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          left -= 13; // Elevator ���� ����� ��ǥ Update
          if (Left_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == SW1_Flag)
        } // for (int floor = Left_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Left_E > SW1_Flag)
      
      if(Left_E < Des_FL) {
        for (int floor = Left_E + 1; floor <= Des_FL; floor++){ // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�    
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Left_E = floor; // ���� �� Left_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_BLUE); // ������ : �Ķ���
        left += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
        if (Left_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Left_E == Des_FL)
       } // for (int floor = Left_E + 1; floor <= Des_FL; floor++)
      } // if(Left_E < Des_FL)
      
      else if(Left_E > Des_FL) {
        for (int floor = Left_E - 1; floor >= Des_FL; floor--) { // ���� �� Left_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Left_E = floor; // ���� �� Left_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(20, 107 - left, 10, 13); // Elevator ���� ����� Draw
          left -= 13; // Elevator ���� ����� ��ǥ Update
          if (Left_E == Des_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Left_E == Des_FL)
        } // for (int floor = Left_E - 1; floor >= Des_FL; floor--)
      } // else if(Left_E > Des_FL)
    } // else
  } // else if (Left_E != SW1_Flag)
  Fram_Write(2023, Left_E); // FRAM WRITE
  LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
  LCD_DisplayText(26, 9, "S"); // Stop ���� Display
  LCD_DisplayText(23, 8, "FL"); // �� ���� ��� ���� (EX->FL)
  GPIOG->ODR &= ~0x00FF; // ��� LED OFF
  GPIOG->ODR |= 0x0080; // LED7 ON
  BEEP(); DelayMS(500); BEEP(); DelayMS(500); BEEP(); DelayMS(500); // ���� 3ȸ �⸲
  Execute_mode = 0; // Execute mode Flag 0
} // void MOVE_Left_E(int Des_FL)

/* ������ ���������� ���� �Լ� */
void MOVE_Right_E(int Start_FL, int Des_FL){
  LCD_SetTextColor(RGB_GREEN); // ���ڻ� : �ʷϻ�
  LCD_DisplayText(25, 8, "R-E");
  
  // ���� ���� ��� ���� ���� ��� 1�� �� ��ǥ ������ �̵�
  if (Right_E == Start_FL) {
    DelayMS(1000);  
    if(Right_E < Des_FL){ // ��� ����
      for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Right_E = floor; // ���� �� Right_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
        right += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E+ 1; floor <= Des_FL; floor++)
    } // if(Right_E< Des_FL)
    
    else if(Right_E > Des_FL){ // �ϰ� ����
      for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Right_E = floor; // ���� �� Right_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "D"); // Down ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
        right -= 13; // Elevator ���� ����� ��ǥ Update
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E- 1; floor >= Des_FL; floor--)
    } // else if(Right_E> Des_FL)
  } // if (Right_E == SW1_Flag)
  
  // �ٸ� �� �Է� �� ����
  else if (Right_E != Start_FL){
    if(Right_E < Des_FL){ // ��� ����
      if(Right_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
          right += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          right -= 13; // Elevator ���� ����� ��ǥ Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Right_E = floor; // ���� �� Right_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
        right += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E + 1; floor <= Des_FL; floor++)
    } // if(Right_E < Des_FL)
    
    /* �ϰ� ���� */
    else if(Right_E > Des_FL){
       if(Right_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
          right += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          right -= 13; // Elevator ���� ����� ��ǥ Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Right_E = floor; // ���� �� Right_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "D"); // Down ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
        right -= 13; // Elevator ���� ����� ��ǥ Update
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Right_E == Des_FL)
      } // for (int floor = Right_E - 1; floor >= Des_FL; floor--)
    } // else if(Right_E > Des_FL)
    
    /* Right_E == Des_FL */
    else {
      if(Right_E < Start_FL) { // �Է������� ���� ��ġ�� ���� ���
        for (int floor =Right_E + 1; floor <= Start_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "U"); // Up ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
          right += 13; // Elevator ���� ����� ��ǥ Update
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor =Right_E + 1; floor <= SW1_Flag; floor++)
      } //if(Right_E < SW1_Flag)
      
      else if (Right_E > Start_FL){ // �Է������� ���� ��ġ�� ���� ���
        for (int floor = Right_E - 1; floor >= Start_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          right -= 13; // Elevator ���� ����� ��ǥ Update
          if (Right_E == Start_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == SW1_Flag)
        } // for (int floor = Right_E - 1; floor >= SW1_Flag; floor--)
      } // else if (Right_E > SW1_Flag)
      
      if(Right_E < Des_FL) { // �������� ��ǥ������ ���� ���
        for (int floor = Right_E + 1; floor <= Des_FL; floor++){ // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
        if(Hold_mode) // Hold mode �߻� �� Hold mode ����
          HoldMode();
        Right_E = floor; // ���� �� Right_E ������Ʈ
        LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
        LCD_DisplayText(26, 9, "U"); // Up ���� Display
        DelayMS(500); // 0.5�� ���
        LCD_SetBrushColor(RGB_GREEN); // ������ : �ʷϻ�
        right += 13; // Elevator ���� ����� ��ǥ Update
        LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
        if (Right_E == Des_FL) {
          DelayMS(500); // 0.5�� ���
          LCD_DisplayText(26, 9, "S"); // Stop ���� Display
          DelayMS(500); // 0.5�� ���
        } // if (Right_E == Des_FL)
       } // for (int floor = Right_E + 1; floor <= Des_FL; floor++)
      } // if(Right_E < Des_FL) left Right_E
      
      else if(Right_E > Des_FL) { // �������� ��ǥ������ ���� ���
        for (int floor = Right_E - 1; floor >= Des_FL; floor--) { // ���� �� Right_E���� ��ǥ �� Des_FL���� �Ʒ� ������ �����ϸ� �� ���� �̵�
          if(Hold_mode) // Hold mode �߻� �� Hold mode ����
            HoldMode();
          Right_E = floor; // ���� �� Right_E ������Ʈ
          LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
          LCD_DisplayText(26, 9, "D"); // Down ���� Display
          DelayMS(500); // 0.5�� ���
          LCD_SetBrushColor(RGB_WHITE); // ������ : �Ͼ��
          LCD_DrawFillRect(120, 107 - right, 10, 13); // Elevator ���� ����� Draw
          right -= 13; // Elevator ���� ����� ��ǥ Update
          if (Right_E == Des_FL) {
            DelayMS(500); // 0.5�� ���
            LCD_DisplayText(26, 9, "S"); // Stop ���� Display
            DelayMS(500); // 0.5�� ���
          } // if (Right_E == Des_FL)
        } // for (int floor = Right_E - 1; floor >= Des_FL; floor--)
      } // else if(Right_E > Des_FL)
    } // else
  } // else if (Right_E != SW1_Flag)
  Fram_Write(2024, Right_E); // FRAM WRITE
  LCD_SetTextColor(RGB_RED); // ���ڻ� : ������
  LCD_DisplayText(26, 9, "S"); // Stop ���� Display
  LCD_DisplayText(23, 8, "FL"); // �� ���� ��� ���� (EX->FL)
  GPIOG->ODR &= ~0x00FF; // ��� LED OFF
  GPIOG->ODR |= 0x0080; // LED7 ON
  BEEP(); DelayMS(500); BEEP(); DelayMS(500); BEEP(); DelayMS(500); // ���� 3ȸ �⸲
  Execute_mode = 0; // Execute mode Flag 0
} // void MOVE_Right_E(int Des_FL)

/* Switch�� �ԷµǾ����� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */ 
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