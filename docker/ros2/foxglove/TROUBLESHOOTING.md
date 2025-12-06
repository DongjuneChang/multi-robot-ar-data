# Foxglove/Lichtblick 3D Mesh & Topic 문제 해결 가이드

**Date:** 2025-12-03  
**Issue:** 다른 PC에서 3D mesh와 토픽이 표시되지 않음

## 문제 진단 체크리스트

### 1. Foxglove Bridge 버전 및 프로토콜 호환성

**문제:**
- `Dockerfile.robot`에서 `apt-get install ros-humble-foxglove-bridge`로 설치하면 **v3.2.2**가 설치됨
- v3.2.2는 `foxglove.sdk.v1` 프로토콜 사용
- Lichtblick은 `foxglove.websocket.v1`만 지원 → **호환 안 됨**

**확인 방법:**
```bash
# serial-robot-control 컨테이너에서
docker exec serial-robot-control bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765" 2>&1 | head -5
```

**예상 출력:**
- v3.2.2: 프로토콜 정보 없음 (기본적으로 `foxglove.sdk.v1`)
- v0.7.2: `Starting foxglove_bridge (humble, 0.7.2@) with WebSocket++/0.8.2`

**해결책: v0.7.2 라이브러리 복사**

```bash
# 1. husarion 이미지에서 0.7.2 라이브러리 추출
mkdir -p /tmp/foxglove-bridge-072
docker create --name temp-husarion husarion/foxglove-bridge:humble
docker cp temp-husarion:/opt/ros/humble/lib/foxglove_bridge /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_component.so /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_base.so /tmp/foxglove-bridge-072/
docker rm temp-husarion

# 2. serial-robot-control에 복사 (apt 버전 덮어쓰기)
docker cp /tmp/foxglove-bridge-072/foxglove_bridge/foxglove_bridge serial-robot-control:/opt/ros/humble/lib/foxglove_bridge/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_base.so serial-robot-control:/opt/ros/humble/lib/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_component.so serial-robot-control:/opt/ros/humble/lib/

# 3. foxglove-bridge 재시작
docker exec serial-robot-control bash -c "pkill -f foxglove_bridge"
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
```

---

### 2. `/robot_description` 토픽 확인

**문제:**
- Lichtblick은 `/robot_description` 토픽에서 URDF를 읽음
- MoveIt launch 파일이 이 토픽을 발행하지 않으면 3D mesh가 표시되지 않음

**확인 방법:**
```bash
# serial-robot-control 컨테이너에서
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic list | grep robot_description"
```

**예상 출력:**
```
/robot_description
```

**없다면:**
```bash
# 토픽 내용 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /robot_description --once" | head -20
```

**해결책:**
- MoveIt launch 파일이 자동으로 `/robot_description`을 발행하는지 확인
- `xarm_moveit_config` 패키지의 launch 파일 확인:
  ```bash
  docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 pkg prefix xarm_moveit_config"
  # 출력된 경로에서 launch 파일 확인
  ```

---

### 3. 메시 파일 경로 문제

**문제:**
- URDF에 `package://xarm_description/meshes/...` 경로가 있음
- foxglove-bridge가 이 경로를 해석하지 못하면 메시가 표시되지 않음

**확인 방법:**
```bash
# URDF 내용 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /robot_description --once" | grep -i "package://" | head -5
```

**예상 출력:**
```
<mesh filename="package://xarm_description/meshes/lite6/link1.stl"/>
```

**해결책:**
- foxglove-bridge가 `serial-robot-control` 컨테이너 내부에서 실행되므로 ROS 패키지 경로 접근 가능해야 함
- `xarm_description` 패키지가 설치되어 있는지 확인:
  ```bash
  docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 pkg prefix xarm_description"
  ```

---

### 4. 토픽 구독 확인

**문제:**
- Lichtblick이 토픽을 구독하지 못함
- WebSocket 연결은 되지만 데이터가 전송되지 않음

**확인 방법:**
```bash
# foxglove-bridge 로그 확인
docker exec serial-robot-control bash -c "ps aux | grep foxglove_bridge"
docker logs serial-robot-control 2>&1 | grep -i foxglove | tail -20
```

**WebSocket 연결 테스트:**
```bash
# 브라우저 콘솔에서:
# 1. WebSocket 연결 확인
# 2. Topics 목록 확인
# 3. /tf, /joint_states 구독 확인
```

**Lichtblick 브라우저 콘솔 확인:**
- F12 → Console 탭
- 에러 메시지 확인:
  - `WebSocket connection failed` → 포트/네트워크 문제
  - `Protocol mismatch` → 버전 문제 (v3.2.2 vs v0.7.2)
  - `Topic not found` → 토픽 발행 안 됨

---

### 5. 네트워크/포트 접근 확인

**문제:**
- 다른 PC에서 접근 시 포트가 막혀있음
- Docker 포트 매핑 문제

**확인 방법:**
```bash
# 호스트에서 포트 확인
netstat -tuln | grep 8765
# 또는
ss -tuln | grep 8765

# Docker 포트 매핑 확인
docker port serial-robot-control | grep 8765
```

**해결책:**
- `serial-robot-control` 컨테이너가 `--network=host`로 실행되어야 함
- 또는 포트 매핑: `-p 8765:8765`

---

## 종합 진단 스크립트

```bash
#!/bin/bash
# foxglove_diagnosis.sh

echo "=== Foxglove Bridge 진단 ==="

# 1. 컨테이너 확인
echo -e "\n[1] serial-robot-control 컨테이너 상태:"
docker ps | grep serial-robot-control

# 2. foxglove-bridge 프로세스 확인
echo -e "\n[2] foxglove-bridge 프로세스:"
docker exec serial-robot-control bash -c "ps aux | grep foxglove_bridge | grep -v grep" || echo "❌ foxglove-bridge 실행 안 됨"

# 3. 버전 확인
echo -e "\n[3] foxglove-bridge 버전:"
docker exec serial-robot-control bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765" 2>&1 | head -3 &
sleep 2
pkill -f foxglove_bridge

# 4. /robot_description 토픽 확인
echo -e "\n[4] /robot_description 토픽:"
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 1 ros2 topic list | grep robot_description" && echo "✅ 존재" || echo "❌ 없음"

# 5. ROS2 토픽 목록
echo -e "\n[5] 주요 ROS2 토픽:"
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic list | grep -E '(tf|joint_states|robot_description)'"

# 6. 포트 확인
echo -e "\n[6] 포트 8765 상태:"
docker exec serial-robot-control bash -c "netstat -tuln | grep 8765" || echo "❌ 포트 열려있지 않음"

# 7. xarm_description 패키지 확인
echo -e "\n[7] xarm_description 패키지:"
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 pkg prefix xarm_description 2>&1" && echo "✅ 설치됨" || echo "❌ 없음"

echo -e "\n=== 진단 완료 ==="
```

---

## 빠른 해결 방법

### 방법 1: v0.7.2 라이브러리 복사 (권장)

위의 "1. Foxglove Bridge 버전" 섹션의 스크립트 실행

### 방법 2: Lichtblick PR #772 적용

```bash
cd shared_data/docker/ros2/foxglove/lichtblick
# PR #772 패치가 이미 있으면
git apply pr772.patch
docker build -f ../Dockerfile.studio -t foxglove-studio:pr772 ..
```

### 방법 3: embed.foxglove.dev 사용 (클라우드)

`docker-compose.yml`에 따르면 현재는 embed.foxglove.dev를 사용하도록 변경됨:
- 장점: 프로토콜 호환성 문제 없음
- 단점: 로그인 필요할 수 있음

---

## 예상 원인 우선순위

1. **프로토콜 호환성 문제** (v3.2.2 vs v0.7.2) - 90%
2. `/robot_description` 토픽 없음 - 5%
3. 메시 파일 경로 문제 - 3%
4. 네트워크/포트 문제 - 2%

---

## 참고

- ARCHITECTURE 문서: `shared_data/docker/ros2/foxglove/ARCHITECTURE_2025-12-01.md`
- Layout 설정: `shared_data/docker/ros2/foxglove/layouts/lite6_default.json`



