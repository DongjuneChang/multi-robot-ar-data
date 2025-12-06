# Lite6 Foxglove Studio 표시 확인 가이드

**Date:** 2025-12-03  
**목적:** Lite6 로봇이 Foxglove Studio에서 3D mesh와 토픽이 제대로 표시되는지 확인

## 사전 준비

### 1. Docker 컨테이너 확인

```bash
# 컨테이너 상태 확인
docker ps | grep serial-robot-control

# 없으면 시작
cd shared_data/docker/ros2/serial-robot
docker compose --env-file ../.env up -d
```

### 2. Foxglove Bridge 버전 확인 및 설정

**문제:** apt로 설치하면 v3.2.2가 설치되지만, Lichtblick과 호환 안 됨

**해결책: v0.7.2 라이브러리 복사**

```bash
# 1. husarion 이미지에서 0.7.2 라이브러리 추출
mkdir -p /tmp/foxglove-bridge-072
docker create --name temp-husarion husarion/foxglove-bridge:humble
docker cp temp-husarion:/opt/ros/humble/lib/foxglove_bridge /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_component.so /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_base.so /tmp/foxglove-bridge-072/
docker rm temp-husarion

# 2. serial-robot-control에 복사
docker cp /tmp/foxglove-bridge-072/foxglove_bridge/foxglove_bridge serial-robot-control:/opt/ros/humble/lib/foxglove_bridge/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_base.so serial-robot-control:/opt/ros/humble/lib/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_component.so serial-robot-control:/opt/ros/humble/lib/
```

### 3. Foxglove Studio (Lichtblick) 실행

```bash
cd shared_data/docker/ros2/foxglove

# Lichtblick 이미지 빌드 (처음만)
docker build -f Dockerfile.studio -t foxglove-studio:local .

# 실행
docker run -d -p 8080:8080 --name foxglove-studio foxglove-studio:local

# 또는 docker-compose 사용 (서비스 추가 필요)
```

---

## 단계별 확인

### Step 1: Lite6 MoveIt Launch

```bash
# serial-robot-control 컨테이너에서
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py show_rviz:=false"
```

**확인:**
```bash
# 프로세스 확인
docker exec serial-robot-control bash -c "ps aux | grep moveit | grep -v grep"

# 로그 확인
docker exec serial-robot-control bash -c "ros2 node list | grep move_group"
```

**예상 출력:**
```
/move_group
```

---

### Step 2: /robot_description 토픽 확인

**중요:** Lichtblick은 `/robot_description` 토픽에서 URDF를 읽습니다.

```bash
# 토픽 존재 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic list | grep robot_description"
```

**예상 출력:**
```
/robot_description
```

**없다면:**
```bash
# 토픽 내용 확인 (처음 몇 줄)
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /robot_description --once" | head -30
```

**예상 내용:**
```xml
<?xml version="1.0"?>
<robot name="lite6">
  <link name="link_base">
    ...
  </link>
  ...
</robot>
```

---

### Step 3: Foxglove Bridge 실행

```bash
# 기존 프로세스 종료
docker exec serial-robot-control bash -c "pkill -f foxglove_bridge"

# v0.7.2로 실행
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
```

**확인:**
```bash
# 프로세스 확인
docker exec serial-robot-control bash -c "ps aux | grep foxglove_bridge | grep -v grep"

# 로그 확인 (버전 확인)
docker exec serial-robot-control bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765" 2>&1 | head -5 &
sleep 2
docker exec serial-robot-control bash -c "pkill -f foxglove_bridge"
```

**예상 출력 (v0.7.2):**
```
Starting foxglove_bridge (humble, 0.7.2@) with WebSocket++/0.8.2
[WS] Server running without TLS
[WS] WebSocket server listening at ws://0.0.0.0:8765
```

**포트 확인:**
```bash
docker exec serial-robot-control bash -c "netstat -tuln | grep 8765"
```

---

### Step 4: WebSocket 연결 테스트

**브라우저에서:**
1. `http://localhost:8080` 접속 (Lichtblick)
2. 좌측 상단 "Open connection" 클릭
3. "Foxglove WebSocket" 선택
4. URL 입력: `ws://localhost:8765` (또는 `ws://<host-ip>:8765`)
5. "Open" 클릭

**연결 성공 시:**
- 좌측 패널에 Topics 목록 표시
- `/robot_description`, `/tf`, `/joint_states` 등 표시

**연결 실패 시:**
- 브라우저 콘솔 (F12) 확인
- 에러 메시지 확인:
  - `Protocol mismatch` → foxglove-bridge 버전 문제 (v0.7.2 필요)
  - `Connection refused` → 포트/네트워크 문제

---

### Step 5: 3D Mesh 표시 확인

**Lichtblick에서:**
1. 좌측 패널에서 "Add panel" → "3D" 선택
2. 3D 패널에서:
   - 좌측 상단 "Add layer" 클릭
   - "URDF" 선택
   - Source: "Topic" 선택
   - Topic: `/robot_description` 선택
   - "Add" 클릭

**예상 결과:**
- ✅ Lite6 로봇 3D 모델 표시
- ✅ TF 프레임 표시 (link_base, link1, ..., link6)
- ✅ Joint states에 따라 로봇 움직임

**문제 발생 시:**
- ❌ 로봇이 보이지 않음 → `/robot_description` 토픽 확인
- ❌ 메시가 보이지 않음 → 메시 파일 경로 문제 (package:// 해석 실패)
- ❌ TF만 보이고 로봇 안 보임 → URDF layer 설정 확인

---

### Step 6: 토픽 데이터 확인

**Lichtblick에서:**
1. 좌측 패널에서 Topics 목록 확인
2. `/joint_states` 토픽 확인:
   - 6개 관절 데이터 (joint1~joint6)
   - position, velocity, effort 값

**확인 명령어:**
```bash
# ROS2에서 직접 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /joint_states --once" | head -20
```

**예상 출력:**
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- 'joint1'
- 'joint2'
- 'joint3'
- 'joint4'
- 'joint5'
- 'joint6'
position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

---

## 종합 확인 스크립트

```bash
#!/bin/bash
# check_lite6_foxglove.sh

echo "=== Lite6 Foxglove Studio 확인 ==="

# 1. 컨테이너 확인
echo -e "\n[1] 컨테이너 상태:"
docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "(serial-robot|foxglove)" || echo "❌ 컨테이너 없음"

# 2. MoveIt 확인
echo -e "\n[2] MoveIt 실행:"
docker exec serial-robot-control bash -c "ros2 node list | grep move_group" && echo "✅ MoveIt 실행 중" || echo "❌ MoveIt 없음"

# 3. /robot_description 확인
echo -e "\n[3] /robot_description 토픽:"
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 1 ros2 topic list | grep robot_description" && echo "✅ 존재" || echo "❌ 없음"

# 4. foxglove-bridge 확인
echo -e "\n[4] foxglove-bridge:"
docker exec serial-robot-control bash -c "ps aux | grep foxglove_bridge | grep -v grep" && echo "✅ 실행 중" || echo "❌ 실행 안 됨"

# 5. 포트 확인
echo -e "\n[5] 포트 8765:"
docker exec serial-robot-control bash -c "netstat -tuln | grep 8765" && echo "✅ 열림" || echo "❌ 닫힘"

# 6. 주요 토픽 확인
echo -e "\n[6] 주요 ROS2 토픽:"
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic list | grep -E '(tf|joint_states|robot_description)'"

echo -e "\n=== 확인 완료 ==="
echo "다음 단계: http://localhost:8080 에서 Lichtblick 접속"
```

---

## 예상 문제 및 해결

### 문제 1: "Protocol mismatch" 에러

**원인:** foxglove-bridge v3.2.2 사용 중

**해결:** v0.7.2 라이브러리 복사 (위 "Foxglove Bridge 버전 확인" 참조)

---

### 문제 2: `/robot_description` 토픽 없음

**원인:** MoveIt launch 파일이 토픽을 발행하지 않음

**해결:**
```bash
# robot_state_publisher가 /robot_description을 발행하는지 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 param list | grep robot_description"
```

---

### 문제 3: 메시 파일이 보이지 않음

**원인:** URDF의 `package://xarm_description/...` 경로를 해석하지 못함

**해결:**
- foxglove-bridge가 `serial-robot-control` 내부에서 실행되므로 패키지 경로 접근 가능해야 함
- xarm_description 패키지 확인:
  ```bash
  docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 pkg prefix xarm_description"
  ```

---

## 성공 기준

✅ **성공:**
- Lichtblick에서 Lite6 3D 모델 표시
- TF 프레임 표시 (link_base, link1~link6)
- `/joint_states` 토픽 데이터 표시
- 로봇이 joint states에 따라 움직임

❌ **실패:**
- 연결은 되지만 로봇이 보이지 않음
- TF만 보이고 메시가 없음
- 토픽이 표시되지 않음

---

## 참고

- ARCHITECTURE: `ARCHITECTURE_2025-12-01.md`
- TROUBLESHOOTING: `TROUBLESHOOTING.md`
- Layout 설정: `layouts/lite6_default.json`



