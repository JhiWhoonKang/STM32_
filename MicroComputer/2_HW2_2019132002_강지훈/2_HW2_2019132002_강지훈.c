/////////////////////////////////////////////////////////////
// ������: �����ı���2023_HW2_Ŀ�����Ǳ�
// ��������: *******************************
// *******************************
// ����� �ϵ����(���): GPIO, Joy-stick, EXTI, GLCD ...
// ������: 2023. 5. 09
// ������ Ŭ����: *���Ϲ�
// �й�: 2019132002
// �̸�: ������
///////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

void _GPIO_Init(void);
void BlackCoffee();     //Black Coffee �Լ� ����
void SugarCoffee();     //Sugar Coffee �Լ� ����
void MixCoffee();     //Mix Coffee �Լ� ����

uint16_t KEY_Scan(void);
uint8_t SW0_Flag, SW1_Flag, SW2_Flag, SW3_Flag; // Switch Flag ���� ����

void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int main(void){
   _GPIO_Init();       // GPIO (LED, SW, Buzzer) �ʱ�ȭ
   GPIOG->ODR &= ~0x00FF;   // LED �ʱⰪ: LED0~7 Off
   SW0_Flag=0,SW1_Flag=0,SW2_Flag=0,SW3_Flag=0;         // Switch Flag ���� �ʱ�ȭ
   
   while(1)
   {
       //coin �Է� ����
       switch (KEY_Scan())   // �Էµ� Switch ���� �з� 
       {
       //Coin �Է¿���
       case 0xFE00:    //SW0
           if (SW0_Flag == 0) {
               GPIOG->BSRRL |= 0x0001;  // Coin LED ON      
               BEEP();                  // BUZ 1ȸ ����
               SW0_Flag = 1;            // COIN Flag = 1
               SW1_Flag = 0;            //Black Coffee Flag = 0���� �ʱ�ȭ
               SW2_Flag = 0;            //Sugar Coffee Flag = 0���� �ʱ�ȭ
               SW3_Flag = 0;            //Mix Coffee Flag = 0���� �ʱ�ȭ
           }
           break;

       //Black Coffee ����
       case 0xFD00: //SW1
           if (SW0_Flag == 1) {         // ���ǹ�: Coin �Է��� ���� ���� ����
               GPIOG->ODR |= 0x0002;        // Black LED ON
               BEEP();                      // BUZ 1ȸ ����
               SW1_Flag = 1;            // Black Coffee Flag 1�� ��ȯ
           }
            break;

       //Sugar Coffee ����
       case 0xFB00: //SW2
           if (SW0_Flag == 1) {         // ���ǹ�: Coin �Է��� ���� ���� ����
               GPIOG->ODR |= 0x0004;        // Sugar LED ON
               BEEP();                      // BUZ 1ȸ ����
               SW2_Flag = 1;            // Sugar Coffee Flag 1�� ��ȯ
           }
           break;

       //Mix Coffee ����
       case 0xF700: //SW3
           if (SW0_Flag == 1) {         // ���ǹ�: Coin �Է��� ���� ���� ����
               GPIOG->ODR |= 0x0008;        // Mix LED ON
               BEEP();                      // BUZ 1ȸ ����
               SW3_Flag = 1;            // Mix Coffee Flag 1�� ��ȯ
           }
           break;
       }  // switch(KEY_Scan())

       //Coin �Է� �� ���� ���ǹ�
       if (SW0_Flag == 1) {         // ���ǹ�: Coin �Է��� ���� ���� ����
           if (SW1_Flag == 1)        //Black Coffee Flag�� 1�� ���
               BlackCoffee();          //BlackCoffee �Լ� ����
           
           else if (SW2_Flag == 1) //Sugar Coffee Flag�� 1�� ���
               SugarCoffee();          //Sugar Coffee �Լ� ����
           
           else if (SW3_Flag == 1)   //Mix Coffee Flag�� 1�� ���
               MixCoffee();                 //Mix Coffee �Լ� ����
       }//      if (SW0_Flag == 1)
  }//   while(1)
}//     int main(void)

void _GPIO_Init(void){
  // LED (GPIO G) ���� : Output mode
   RCC->AHB1ENR   |=  0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
   GPIOG->MODER    |=  0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
   GPIOG->OTYPER   &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
   GPIOG->OSPEEDR    |=  0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed

   // SW (GPIO H) ���� : Input mode 
   RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
   GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
   GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
        
  // Buzzer (GPIO F) ���� : Output mode
   RCC->AHB1ENR   |=  0x00000020;   // RCC_AHB1ENR : GPIOF(bit#5) Enable                     
   GPIOF->MODER    |=  0x00040000;   // GPIOF 9 : Output mode (0b01)                  
   GPIOF->OTYPER    &= ~0x0200;   // GPIOF 9 : Push-pull     
   GPIOF->OSPEEDR    |=  0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed
}

void BlackCoffee() {
    //Cup LED 1ȸ ����
    DelayMS(1000);               //1�� �Ŀ� Cup LED ON
    GPIOG->ODR |= 0x0080;        // Cup LED ON
    DelayMS(500);                //0.5�� ���
    GPIOG->ODR &= ~0x0080;       //Cup LED OFF

    //Water/Coffee LED 3ȸ ����
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C

    //0.5�� ���� 3ȸ BUZ
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    
    //�ٽ� ���� ���
    DelayMS(1000);
    GPIOG->ODR &= ~0x00FF;   // LED0~7 Off
    SW0_Flag = 0;       //Coin Flag 0���� �ʱ�ȭ
}

void SugarCoffee() {
    //Cup LED 1ȸ ����
    DelayMS(1000);               //1�� �Ŀ� Cup LED ON
    GPIOG->ODR |= 0x0080;        // Cup LED ON
    DelayMS(500);                //0.5�� ���
    GPIOG->ODR &= ~0x0080;       //Cup LED OFF

    //Sugar LED 2ȸ ����
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0040;     //LED6 ON Sugar
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0040;     //LED6 OFF Sugar     
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0040;     //LED6 ON Sugar
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0040;     //LED6 OFF Sugar

    //Water/Coffee LED 3ȸ ����
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C

    //0.5�� ���� 3ȸ BUZ
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    
    //�ٽ� ���� ���
    DelayMS(1000);
    GPIOG->ODR &= ~0x00FF;   // LED0~7 Off
    SW0_Flag = 0;       //Coin Flag 0���� �ʱ�ȭ
}

void MixCoffee() {
    //Cup LED 1ȸ ����
    DelayMS(1000);               //1�� �Ŀ� Cup LED ON
    GPIOG->ODR |= 0x0080;        // Cup LED ON
    DelayMS(500);                //0.5�� ���
    GPIOG->ODR &= ~0x0080;       //Cup LED OFF

    //Sugar LED 2ȸ ����
    DelayMS(500);
    GPIOG->ODR |= 0x0040;     //LED6 ON Sugar
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0040;     //LED6 OFF Sugar 
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0040;     //LED6 ON Sugar
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0040;     //LED6 OFF Sugar

    //Cream LED 2ȸ ����
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0020;     //LED5 ON Cream
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0020;     //LED5 OFF Cream 
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0020;     //LED5 ON Cream
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0020;     //LED5 OFF Cream

    //Water/Coffee LED 3ȸ ����
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR |= 0x0010;     //LED4 ON W/C
    DelayMS(500);               //0.5�� ���
    GPIOG->ODR &= ~0x0010;     //LED4 OFF W/C

    //0.5�� ���� 3ȸ BUZ
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    DelayMS(500);
    BEEP();
    
    //�ٽ� ���� ���
    DelayMS(1000);
    GPIOG->ODR &= ~0x00FF;   // LED0~7 Off
    SW0_Flag = 0;       //Coin Flag 0���� �ʱ�ȭ
}

/* Switch�� �ԷµǾ������� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */ 
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)   // input key SW0 - SW7 
{ 
   uint16_t key;
   key = GPIOH->IDR & 0xFF00;   // any key pressed ?
   if(key == 0xFF00)      // if no key, check key off
   {   if(key_flag == 0)
         return key;
      else
      {   DelayMS(10);
         key_flag = 0;
         return key;
      }
   }
   else            // if key input, check continuous key
   {   if(key_flag != 0)   // if continuous key, treat as no key input
         return 0xFF00;
      else         // if new key,delay for debounce
      {   key_flag = 1;
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