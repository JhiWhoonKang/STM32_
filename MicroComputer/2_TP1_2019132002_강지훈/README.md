# STM32F407IG_Elevator
## System Overview
- 빌딩(0~6층)에 설치된 엘리베이터에서 목표 층 스위치를 입력하여 목표 층까지 이동하도록 제어하는 프로그램 작성
- 필요한 Resources:
  - 층 입력: SW 7개(SW0 ~ 6(PH8 ~ 14): GPIO)
  - 현재 층과 목표 층 표시: GLCD
  - 이동 상황 표시: LED 7개(LED0 ~ 6)
  - Holding 동작: SW 1개(SW7(PH15): EXTI15)

## System Diagram
![Image](https://github.com/user-attachments/assets/931fdbd2-6f6b-4d4a-9200-96c52d1d3db9)

## System Sequence
### ✅엘리베이터 프로그램 동작 순서 - 무한 반복
1. 최초: 0층 (LED0 ON)
2. 층 선택 스위치(SW) 입력(SW0 ~ 6: GPIO, 0층 ~ 6층)
   <br/>(1) SW0~6 입력 즉시, Buzzer ON (Beep() 1회)
   <br/>(2) GLCD : 목표 층(‘**Des FL:-**’) 층 숫자가 GLCD에 표시
   <br/>(예) 목표 층으로 5층 SW를 입력하면 ‘**DesFL:5**’ 표시
3. 이동(LED 상태 변화)
   <br/>(1) LED : 선택된 SW에해당하는층까지점멸되며이동
   <br/>(예) 현재 0층에 있고, 목표 층 5층 SW를 누르면 1초 후에 0층(LED0) OFF, 1층 (LED1) ON-> 1초후에1층(LED1) OFF, 2층 (LED2) ON-> …-> 1초 후에 4층(LED4) OFF, 5층 (LED5) ON
4. 목표층 도착
   <br/>(1) GLCD : 현재층(‘Cur FL: ?’)과 목표층(‘Des FL: ?’)이 update
   <br/>(예) 목표층5층에 도착하면,‘CurFL:5’, ‘Des FL:-’ 표시
   <br/>(2) Buzzer 3회 울림 (Beep() 3회))
5. [**2 단계**]로 이동하여 SW 입력을 기다림
   ※ 현재 층과 같은 SW를 누르면 변화 없음

### ✅HOLDING 동작
(1) 엘리베이터가 이동중 SW7(EXTI15)를 누르면 EXTI15 인터럽트가 발생하여 엘리베이터 이동 동작이 5초간 멈춤(LED7ON,Beep()2회)
<br/>(2) 즉, EXTI15_IRQHandler()루틴에 들어가서 부저 2회 후 5초 delay 함수를 실행 함
<br/>(3) 5초 후 Handler에서 return하여 이동 동작 계속 수행함(handler에서 **`return`** 전 LED7 OFF, **`Beep()`** 2회 실행)

### ✅GLCD초기화면
![image](https://github.com/user-attachments/assets/f88b530f-664b-4251-8118-20f567326be0)
- 바탕화면 색
  - 노란색
- 'MECHA Elevator(KJW)'
  - 파란색
  - 초기화 시 1회만 표시
- 'Cur FL:', 'Des FL:'
  - 검정색
  - 초기화 시 1회만 표시
- '0', '-'
  - 빨간색
  - 층 입력 시 마다 변화

### ✅LED 초기 상태 0층
![image](https://github.com/user-attachments/assets/54a701b8-6b88-4e47-a1b1-861fd431f773)

### ✅예제 (0F → 5F → 3F)
#### GLCD 변화 과정
![image](https://github.com/user-attachments/assets/aeccf1d3-6aa8-4e2a-8323-fd16f93f4a1e)

#### LED 변화 과정
![image](https://github.com/user-attachments/assets/a2b7cdb3-4874-4607-ac57-42c32f571888)
