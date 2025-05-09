# STM32F407IG_Elevator
## System Overview
- 6층 건물에 두 대의 엘리베이터(Left엘리베이터(L-E), Right엘리베이터(R-E)) 설치
- 출발 층에서 목표 층 스위치(1~6층)를 선택하면 엘리베이터가 이동하도록 프로그램 작성
  <br/>※ 현재 층: 현 상태에서의 엘리베이터가 위치한 층
  <br/>※ 출발 층: 탑승자가 위치한 층
  <br/>※ 목표 층: 탑승자가 이동하여 가고자 하는 층
- 이동은 이동 막대의 크기 변화로 표시
- 동작 모드 종류
  - **층 선택** 모드
  - **실행** 모드
  - **중단** 모드
- 필요한 Resources:
  - 층 입력: SW 7개(SW0 ~ 6(PH8 ~ 14): GPIO)
  - 현재 층과 목표 층 표시: GLCD
  - 이동 상황 표시: LED 7개(LED0 ~ 6)
  - Holding 동작: SW 1개(SW7(PH15): EXTI15)

## System Diagram
![image](https://github.com/user-attachments/assets/4385967e-bf52-4a53-a7c8-41f899c6e4f5)


## System Sequence
### ✅모드 간 상관관계
![image](https://github.com/user-attachments/assets/f5158343-0483-4f64-b145-0f85d709a4ae)
### ✅GLCD 화면 구성
![image](https://github.com/user-attachments/assets/808186d8-6bf4-4431-ba9a-cd6ca1b1eb16)
### ✅초기 상태
- 초기 엘리베이터: **L-E**
- 초기 모드: 층 선택 모드(**'FL'**)
- 현재 층: 1층(L-E, R-E 모두), '1>1'(단, FRAM 사용 시 다른 층 표시)
- 동작 상태: 정지(**'S'**) 상태
- 초기 LED 상태: LED7(ON), LED0~6(OFF)  
### 1️⃣ 층 선택(Floor) 모드
- 초기 상태 또는 실행 모드 종료되면 동작하는 모드
- **층 선택 모드 진입** 시: **'FL'** 표시, **LED0(OFF)**, **LED7(ON)**, **LED1~6(OFF)**
- **출발 층 선택** 방법: **SW1 입력**
  - **SW2을 누를 때마다 현재 값에서 +1씩 증가**하여 GLCD에 표시(1~6까지 변화, **6 이후에는 다시 1**)
  - **'출발 층' 위치**에 층 숫자 표시
  - SW1을 누르면 LED1(ON, 다른 SW 누를 때까지 ON 유지), LED2(OFF), BUZ(1회)
- **목표 층 선택** 방법: **SW2 입력**
  - **SW1을 누를 때마다 현재 값에서 +1씩 증가**하여 GLCD에 표시(1~6까지 변화, **6 이후에는 다시 1**)
  - **'목표 층' 위치**에 층 숫자 표시
  - SW2를 누르면 LED2(ON, 다른 SW 누를 때까지 ON 유지), LED1(OFF), BUZ(1회)

<br/>※ 층 입력 시 LED0, 7, 6은 변동 없이 기존 상태 유지
<br/>※ **'출발 층 == 목표 층'인 경우**, 실행 모드에 들어가면 엘리베이터가 현재 층에서 출발 층으로 이동 후 1초 후 종료(도착)로 처리(동작 막대 無변화)
<br/>※ 층 선택 모드에서는 **실행 모드로만 전환(SW0(EXTI8) 입력) 가능**
        ⇒ 즉, **중단 모드로는 전환 불가**
### 2️⃣ 실행(Execute) 모드
- 층 선택 모드에서 SW0(EXTI8) 입력 시 실행
- 실행 모드 진입 시:
  - 'EX' 표시
  - LED0(ON), LED7(OFF), LED1, 2, 6(OFF)
  - BUZ(1회)
- 실행 모드가 되면 엘리베이터 동작
- 층 입력 모드에서 입력된 **출발 층과 가장 가까운 엘리베이터가 움직여서 출발 층으로 이동**
  - 두 엘리베이터의 현 위치(현재 층)가 출발 층과 거리가 같을 경우 Left 엘리베이터(L-E) 우선으로 동작
  - 엘리베이터가 현재 층에서 출발층으로 이동하고 출발 층에서 멈추었다가 목표 층으로 이동
    - 즉, GLCD의 동작 막대가 층에 따라 증감
  - **'현재 층 == 출발 층'인 경우**: 1초 후 목표 층으로 이동
- 한 층 사이 움직이는 시간 및 각 단계 사이 시간: 0.5초
- 실행이 종료(목표 층에 도착된 상태, 즉, LCD의 동작 상태 표시가 'S'인 상태)가 되면,
  - 층 선택 모드로 변경
  - BUZ(3회)
  - LED0(OFF), LED7(ON), LED1~6(OFF)
#### 동작 예 - Left(or Right) Elevator가 움직이는 경우
① (현재 층 != 출발 층) - (현재 층 → 출발 층 → 목표 층)
<br/>Up(or Down) → ... → Stop → Up(or Down)
<br/>② (현재 층 == 출발 층) - (출발 층 → 목표 층)
<br/>Up(or Down) → ... → Stop
### 3️⃣ 중단(HOLD) 모드
- SW7(EXTI15) 입력 시 실행
- 중단 모드 진입 시:
  - 'EX' → 'HD'
  - LED0(OFF), LED7(OFF), LED6(ON)
  - 층 표시는 현 상태 유지
  - 실행 모드(EX) 중에만 중단 모드로 전환 가능
  - 중단 모드가 되면, 엘리베이터 동작이
    - 5초간 중단
    - 중단된 5초간 BUZ 0.5초 간격으로 울림(On-Off 반복)
- 5초 후 중단 모드에서 빠져나와 중단 모드로 전환되기 직전의 상태에서 엘리베이터가 동작 실행
- 실행 모드로 다시 전환되면,
  - 'HD' → 'EX'
  - LED0(ON0
  - LED7(OFF)
  - LED6(OFF)
### ✅ 목표 층을 FRAM에 저장
- 실행 모드에서 목표 층에 도착하면 목표 층 번호를 FRAM에 저장
  - L-E의 목표층: 2023번지
  - R-E의 목표층: 2024번지
- Reset 후 FRAM의 저장된 값을 Read
  - 각 FRAM에서 Read한 값은 각 엘리베이터의 현재 층 정보가 됨
  - LCD 화면은 FRAM Read 값(현재 층) 정보를 기반으로 다음과 같이 표시
    - 초기 엘리베이터: L-E
    - 초기 모드: 층 선택 모드('FL')
    - 현재 층: FRAM 2023번지 값
      - 즉, **'FRAM 2023번지 값 > FRAM 2023번지 값'** 표시
    - 동작 막대: 현재 층(L-E(2023번지), R-E(2024번지))에 맞게 좌우 엘리베이터의 막대 표시
    - 동작 상태: 정지('S') 상태
    - 초기 LED 상태: LED7(ON), LED0~6(OFF)      
### ✅예제 (초기 상태에서 2F(출발), 4F(목표) 선택 → 1F(출발), 2F(목표) 선택)
![image](https://github.com/user-attachments/assets/52cbf2bd-875b-4072-a1e9-6dd10c7c5b34)
![image](https://github.com/user-attachments/assets/abd72da2-80e0-4588-ac7e-7ff2924bca8e)

