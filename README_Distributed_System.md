# EPIC 분산 시스템 실행 가이드

## 개요
EPIC 탐색 알고리즘을 ARM 타겟 보드와 호스트 PC로 분리하여 실행하는 방법을 설명합니다.

- **호스트 PC**: 시뮬레이션, 시각화, ROS Master
- **ARM 타겟 보드**: 핵심 탐색 알고리즘, 경로 계획

## 시스템 구성

### 호스트 PC (x86_64)
- ROS Master 실행
- MARS 시뮬레이션 환경
- RViz 시각화
- 포인트클라우드 및 오도메트리 데이터 제공

### ARM 타겟 보드 (aarch64)
- 핵심 탐색 알고리즘 (`exploration_node`)
- 궤적 서버 (`traj_server`)
- 로컬 경로 계획 (`fast_planner_node`)

## 실행 방법

### 1. 호스트 PC에서 시뮬레이션 실행

```bash
# 호스트 PC에서
cd ~/catkin_ws
source devel/setup.bash

# 시뮬레이션 및 시각화 실행
roslaunch epic_planner host_pc_simulation.launch map_name:=garage
```

### 2. ARM 타겟 보드에서 탐색 알고리즘 실행

```bash
# ARM 타겟 보드에서
cd ~/catkin_ws

# 간편 실행 스크립트 사용
./src/src/EPIC/run_target_board.sh <HOST_PC_IP> [map_name]

# 예시
./src/src/EPIC/run_target_board.sh 192.168.1.100 garage
```

또는 수동으로:

```bash
# ARM 타겟 보드에서
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
source devel/setup.bash

roslaunch epic_planner target_board_core.launch map_name:=garage
```

## 네트워크 설정

### 방화벽 설정 (호스트 PC)
```bash
# ROS 통신용 포트 허용
sudo ufw allow 11311  # ROS Master
sudo ufw allow 33691:65000/tcp  # ROS 동적 포트 범위
sudo ufw allow 33691:65000/udp
```

### IP 주소 확인
```bash
# 호스트 PC IP 확인
hostname -I

# ARM 보드 IP 확인
hostname -I
```

## 주요 토픽

### 입력 토픽 (호스트 PC → ARM 보드)
- `/quad_0/lidar_slam/odom` - 로봇 오도메트리
- `/quad0_pcl_render_node/cloud` - 포인트클라우드 데이터

### 출력 토픽 (ARM 보드 → 호스트 PC)
- `/planning/pos_cmd` - 위치 명령
- `/planning/bspline` - B-spline 궤적
- `/exploration_finish` - 탐색 완료 신호

## 성능 최적화

### ARM 보드 설정
- 시각화 비활성화 (`view_graph: false`)
- 타임아웃 값 증가 (ARM 성능 고려)
- 궤적 길이 및 속도 제한 조정

### 네트워크 최적화
- 유선 연결 권장
- 대역폭 모니터링: `iftop` 또는 `nload` 사용

## 문제 해결

### 연결 문제
```bash
# ROS Master 연결 테스트
rostopic list

# 특정 토픽 확인
rostopic echo /quad_0/lidar_slam/odom -n 1
```

### 성능 문제
```bash
# ARM 보드 리소스 모니터링
htop

# 네트워크 대역폭 확인
iftop -i eth0
```

### 로그 확인
```bash
# 노드별 로그 확인
rosnode list
rosnode info /exploration_node
```

## 지원 맵
- `garage` - 차고 환경
- `cave` - 동굴 환경  
- `factory` - 공장 환경

각 맵에 대해 최적화된 설정 파일이 제공됩니다:
- `target_board_garage.yaml`
- `target_board_cave.yaml`
- `target_board_factory.yaml`

## 추가 기능

### 수동 목표점 설정
RViz에서 "2D Nav Goal"을 사용하여 수동으로 목표점을 설정할 수 있습니다.

### 실시간 모니터링
```bash
# 궤적 명령 모니터링
rostopic echo /planning/pos_cmd

# 탐색 상태 확인
rostopic echo /exploration_finish
```

이 분산 시스템을 통해 ARM 하드웨어의 제한된 리소스를 효율적으로 활용하면서도 강력한 탐색 성능을 얻을 수 있습니다. 