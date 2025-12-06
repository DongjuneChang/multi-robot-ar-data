# Lite6 Foxglove 테스트 - 현재 상태

**Date:** 2025-12-04  
**Status:** `/robot_description` 토픽 발행 성공 ✅

## 현재 상태

✅ **성공:**
- `/robot_description` 토픽 발행 중 (URDF 데이터 확인됨)
- MoveIt 실행 중 (`publish_robot_description:=true`로 재시작)
- foxglove-bridge 컨테이너 실행 중
- foxglove-studio (Lichtblick) 컨테이너 실행 중

## 브라우저에서 테스트

### 1. Lichtblick 접속
```
http://localhost:8080
```

### 2. WebSocket 연결
1. 좌측 상단 "Open connection" 클릭
2. "Foxglove WebSocket" 선택
3. URL 입력: `ws://localhost:8765`
4. "Open" 클릭

**연결 성공 시:**
- 좌측 패널에 Topics 목록 표시
- `/robot_description`, `/tf`, `/joint_states` 등 표시

**연결 실패 시:**
- 브라우저 콘솔 (F12) 확인
- 에러 메시지 확인:
  - `Protocol mismatch` → foxglove-bridge 버전 문제 (v3.2.2 → v0.7.2 필요)
  - `Connection refused` → 포트/네트워크 문제

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
- 로봇이 보이지 않음 → 프로토콜 호환성 문제 가능
- TF만 보이고 메시 없음 → 메시 파일 경로 문제

## 프로토콜 호환성 문제 해결

**문제 발생 시:**
- 브라우저 콘솔에 "Protocol mismatch" 에러
- foxglove-bridge v3.2.2와 Lichtblick 호환 안 됨

**해결:**
`TROUBLESHOOTING.md`의 "v0.7.2 라이브러리 복사" 섹션 참조

## 빠른 확인 명령어

```bash
# 1. /robot_description 발행 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /robot_description --once | head -3"

# 2. MoveIt 실행 확인
docker exec serial-robot-control bash -c "ps aux | grep move_group | grep -v grep"

# 3. foxglove-bridge 확인
docker exec foxglove-bridge bash -c "ps aux | grep foxglove_bridge | grep -v grep"

# 4. 컨테이너 상태
docker ps | grep -E "(serial-robot|foxglove)"
```

## 다음 단계

1. 브라우저에서 `http://localhost:8080` 접속
2. WebSocket 연결 테스트
3. 3D 패널에서 URDF layer 추가
4. Lite6 로봇 표시 확인

**문제 발생 시:**
- 브라우저 콘솔 에러 확인
- `CURRENT_STATUS.md` 참조
- `TROUBLESHOOTING.md` 참조



