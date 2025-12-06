# Lite6 Foxglove Studio 빠른 테스트 가이드

**Date:** 2025-12-04  
**상태:** Lichtblick 실행 중

## 현재 상태

✅ **실행 중:**
- `serial-robot-control` - ROS2 컨테이너
- `foxglove-bridge` - WebSocket 서버 (포트 8765)
- `foxglove-studio` - Lichtblick (포트 8080)
- MoveIt - lite6 실행 중

✅ **토픽 확인됨:**
- `/robot_description` - URDF 데이터
- `/tf`, `/tf_static` - Transform 데이터
- `/joint_states` - 관절 상태

## 브라우저에서 테스트

### 1. Lichtblick 접속

```
http://localhost:8080
```

### 2. Foxglove Bridge 연결

1. 좌측 상단 "Open connection" 클릭
2. "Foxglove WebSocket" 선택
3. URL 입력: `ws://localhost:8765`
4. "Open" 클릭

### 3. 3D 패널 설정

1. "Add panel" → "3D" 선택
2. 좌측 상단 "Add layer" 클릭
3. "URDF" 선택
4. Source: "Topic"
5. Topic: `/robot_description` 선택
6. "Add" 클릭

### 4. 예상 결과

✅ **성공:**
- Lite6 로봇 3D 모델 표시
- TF 프레임 표시 (link_base, link1~link6)
- Joint states에 따라 로봇 움직임

❌ **실패 시:**
- "Protocol mismatch" → foxglove-bridge 버전 문제 (v3.2.2 → v0.7.2 필요)
- 로봇이 보이지 않음 → `/robot_description` 토픽 확인
- 연결 안 됨 → 포트/네트워크 확인

## 버전 호환성 문제 해결

**현재 문제:**
- foxglove-bridge: v3.2.2 (foxglove.sdk.v1 프로토콜)
- Lichtblick: foxglove.websocket.v1 프로토콜만 지원

**해결책: v0.7.2 라이브러리 복사**

```bash
# 1. husarion 이미지에서 0.7.2 라이브러리 추출
mkdir -p /tmp/foxglove-bridge-072
docker create --name temp-husarion husarion/foxglove-bridge:humble
docker cp temp-husarion:/opt/ros/humble/lib/foxglove_bridge /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_component.so /tmp/foxglove-bridge-072/
docker cp temp-husarion:/opt/ros/humble/lib/libfoxglove_bridge_base.so /tmp/foxglove-bridge-072/
docker rm temp-husarion

# 2. foxglove-bridge 컨테이너에 복사
docker cp /tmp/foxglove-bridge-072/foxglove_bridge/foxglove_bridge foxglove-bridge:/opt/ros/humble/lib/foxglove_bridge/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_base.so foxglove-bridge:/opt/ros/humble/lib/
docker cp /tmp/foxglove-bridge-072/libfoxglove_bridge_component.so foxglove-bridge:/opt/ros/humble/lib/

# 3. foxglove-bridge 재시작
docker restart foxglove-bridge
```

## 빠른 확인 명령어

```bash
# 1. 컨테이너 상태
docker ps | grep -E "(serial-robot|foxglove)"

# 2. MoveIt 확인
docker exec serial-robot-control bash -c "ros2 node list | grep move_group"

# 3. 토픽 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic list | grep robot_description"

# 4. foxglove-bridge 확인
docker exec foxglove-bridge bash -c "ps aux | grep foxglove_bridge | grep -v grep"
```

## 다음 단계

1. 브라우저에서 `http://localhost:8080` 접속
2. WebSocket 연결: `ws://localhost:8765`
3. 3D 패널에서 URDF layer 추가
4. Lite6 로봇 표시 확인

**문제 발생 시:**
- 브라우저 콘솔 (F12) 확인
- `TROUBLESHOOTING.md` 참고
- 버전 호환성 문제 해결 스크립트 실행



