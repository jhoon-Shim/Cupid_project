# 사랑이 싹트는 42 — 프로젝트 아키텍처

---

## 한 줄 정의

> 42 클러스터 안에서, 터틀봇이 익명 메시지를 실제 좌석까지 직접 배달하는 시스템

---

## 접근 방식

### 1단계 — 문제 정의

42 클러스터는 매일 수십 명이 같은 공간에서 코딩한다.  
말 걸기 어색한 분위기, 차마 못 한 말들.  
→ 로봇이 대신 가면 어떨까?

핵심 질문 두 가지:
- **"어디로 가야 하는가?"** → 클러스터 실제 도면 기반 좌표 매핑
- **"어떻게 메시지를 전달하는가?"** → 웹 UI → WebSocket → ROS2 → 로봇

---

### 2단계 — 기술 스택 선택

| 레이어 | 선택 | 이유 |
|---|---|---|
| 로봇 플랫폼 | TurtleBot3 Burger | ROS2 공식 지원, 라이다 내장 |
| 자율주행 | ROS2 Nav2 | 경로계획 + 장애물 회피 통합 |
| 지도 | 실측 SLAM 맵 (.pgm/.yaml) | 클러스터 실제 도면 |
| 통신 | WebSocket (8765) | 브라우저 ↔ ROS2 실시간 연결 |
| UI | 순수 HTML/JS | 별도 프레임워크 없이 즉시 실행 |
| 카메라 | v4l2_camera + CompressedImage | Raspberry Pi 카메라 모듈 |

---

## 시스템 아키텍처

```
+------------------------------------------------------------------+
|  LAPTOP                                                          |
|                                                                  |
|  +--------------------+  WebSocket  +------------------------+  |
|  | cupid42_demo.html  |<----------->|    web_bridge.py       |  |
|  |                    |             |                        |  |
|  | - Seat selection   |             | HTTP :8080 / WS :8765  |  |
|  | - Message input    |             | NavigateToPose Client  |  |
|  | - ETA countdown    |             | Camera subscriber      |  |
|  | - Minimap / Camera |             | nav_feedback broadcast |  |
|  +--------------------+             +------------+-----------+  |
|                                                  |              |
+--------------------------------------------------|--------------+
                                                   |
                                       ROS2 (same Wi-Fi)
                                                   |
                                                   v
+------------------------------------------------------------------+
|  TURTLEBOT3                                                      |
|                                                                  |
|  +-----------------------------+  +----------------------------+ |
|  | camera_robot.launch.py      |  | cupid_demo.launch.py       | |
|  | (runs ON TurtleBot)         |  | (runs on Laptop)           | |
|  |                             |  |                            | |
|  | - turtlebot3_bringup        |  | - map_server  (PGM map)    | |
|  | - ld08_driver  (LiDAR)      |  | - amcl        (localize)   | |
|  | - turtlebot3_ros  (OpenCR)  |  | - planner     (path plan)  | |
|  | - v4l2_camera_node          |  | - controller  (DWB track)  | |
|  |   /camera/.../compressed    |  | - bt_navigator             | |
|  |                             |  | - lifecycle_manager        | |
|  +-----------------------------+  +----------------------------+ |
+------------------------------------------------------------------+
```

- 노트북과 터틀봇은 동일한 Wi-Fi에 연결되어 ROS2 DDS로 통신
- `cupid_demo.launch.py`는 노트북에서 실행되지만 터틀봇의 `/scan`, `/odom`, `/cmd_vel` 토픽을 직접 구독·발행

---

## 데이터 흐름

```
사용자 입력 (좌석 선택 + 메시지 작성)
         |
         v
  WebSocket send  ->  { type: "goto", seat: 3 }
         |
         v
  web_bridge.py 수신
  -> navigate(seat=3) 호출
  -> NavigateToPose Action Goal 전송
  -> SOFAS[3] 좌표 { x: 8.29, y: 4.61 } -> map frame
         |
         +---> Nav2 경로 계산 시작
         |           |
         |     피드백 수신 (estimated_time_remaining, distance_remaining)
         |           |
         |     WebSocket broadcast -> { type: "nav_feedback", eta_sec: 45 }
         |           |
         |     브라우저 ETA 카운트다운 + 미니맵 봇 이동 업데이트
         |
         +---> 도착 완료
               -> WebSocket broadcast -> { type: "arrived" }
               -> 브라우저: BGM 재생 + 도착 화면 전환

병렬 스트림 (카메라):
  TurtleBot v4l2_camera_node
  -> /camera/image_raw/compressed  (CompressedImage, BEST_EFFORT QoS)
  -> web_bridge.py  ->  base64 JPEG 인코딩
  -> WebSocket broadcast  ->  { type: "camera", data: "<base64>" }
  -> 브라우저 <img> 실시간 업데이트  (~10 fps)
```

---

## 핵심 구현 포인트

### 좌석 좌표 매핑

클러스터 실제 도면으로 SLAM 맵을 생성한 뒤,  
RViz의 2D Pose Estimate로 각 소파 위치를 직접 찍어 map frame 좌표 추출.

```python
SOFAS = {
    1: {'x': 8.46, 'y': 1.04, ...},   # 소파 1
    2: {'x': 8.27, 'y': 2.68, ...},   # 소파 2
    3: {'x': 8.29, 'y': 4.61, ...},   # 소파 3
    4: {'x': 8.19, 'y': 6.63, ...},   # 소파 4
    5: {'x': 6.40, 'y': 6.82, ...},   # 소파 5
}
```

### ROS2 ↔ 브라우저 브리지

`asyncio` + `threading` 이중 구조:
- ROS2 spin → 별도 스레드 (`threading.Thread`)
- WebSocket 서버 → asyncio 이벤트 루프
- 두 세계 연결: `asyncio.run_coroutine_threadsafe()`

### 실시간 ETA 표시

Nav2 `NavigateToPose` 액션의 피드백 콜백에서 `estimated_time_remaining` 직접 수신.  
첫 메시지 도달 전엔 거리 기반 추정값(속도 0.20 m/s 가정)으로 선행 표시 후 실제값으로 보정.

---

## 개발 순서

```
1. SLAM 맵 제작
   - TurtleBot + Cartographer -> 42 클러스터 2F 도면 .pgm 생성

2. 기본 이동 프로토타입 (go_to_sofa.py)
   - 터미널에서 번호 입력 -> NavigateToPose 전송 -> 도착 확인

3. 웹 UI + WebSocket 브리지 구축 (web_bridge.py)
   - HTTP 서버(8080) + WS 서버(8765) 단일 프로세스로 통합

4. 실시간 피드백 연동
   - Nav2 feedback -> WS broadcast -> 브라우저 ETA/미니맵 업데이트

5. 카메라 스트리밍 추가
   - v4l2_camera(터틀봇) -> CompressedImage -> base64 -> 브라우저

6. 실제 로봇 테스트 & 파라미터 튜닝
   - AMCL 초기 위치 설정, DWB 속도/허용오차 조정
```

---

## 테스트 환경 및 한계

### 테스트 환경

실제 클러스터 내부가 아닌 **클러스터 외부 공간에서 시연 테스트**를 진행했으며,  
자율주행 · 카메라 스트리밍 · ETA 표시 · BGM 재생 전 과정이 정상 동작함을 확인하였다.  
클러스터 도면 기반 SLAM 맵을 그대로 사용하므로, **클러스터 내부에서도 동일하게 동작할 것으로 기대**한다.

### 하드웨어 한계 — 바닥 몰딩 문제

TurtleBot3 Burger는 지상고(바퀴 높이)가 낮아 바닥 몰딩(문지방, 경계선 단차)을 넘지 못한다.  
→ 라이다는 수평 스캔이라 낮은 단차를 장애물로 인식하지 못함

**해결 방법:**  
몰딩 위에 **의자 등 물체를 의도적으로 배치**하여 라이다가 장애물로 감지하도록 했다.  
Nav2 costmap이 해당 구역을 회피 경로로 계획하여 충돌 없이 우회 주행에 성공하였다.

```
    [바닥 몰딩 구간]
    ================  <- 터틀봇이 넘지 못하는 단차
    
    해결: 의자를 놓아 라이다 감지 범위 안으로 올림
    
        [의자]
          |
    ======+=========  <- 라이다가 의자 다리를 장애물로 인식
          |
    Nav2가 자동으로 우회 경로 계획
```

---

## 파일 구조

```
Cupid/
+-- cupid_demo.launch.py     # 노트북 측 Nav2 전체 런치
+-- web_bridge.py            # ROS2 <-> WebSocket/HTTP 브리지
+-- cupid42_demo.html        # 발신자 웹 UI
+-- go_to_sofa.py            # 초기 프로토타입 (터미널 버전)
+-- camera_robot.launch.py   # 터틀봇 측 bringup + 카메라 런치
+-- 2026_0602_1518_2F.pgm    # 클러스터 실측 SLAM 맵
+-- 2026_0602_1518_2F.yaml   # 맵 메타데이터
+-- config/
    +-- nav2_params.yaml     # Nav2 파라미터 (DWB, costmap 등)
    +-- amcl_params.yaml     # AMCL 파라미터
```

---

*TurtleBot3 + ROS2 Humble + Nav2 | 42 클러스터 2F | 2026*
