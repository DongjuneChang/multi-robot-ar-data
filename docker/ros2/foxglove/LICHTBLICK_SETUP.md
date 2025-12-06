# Lichtblick(L 아저씨) 설정 완료 가이드

**Date:** 2025-12-04  
**목적:** Lichtblick과 호환되는 foxglove-bridge v0.7.2 설정

## 핵심 변경사항

### Dockerfile.robot 수정

**변경 전:**
- apt로 foxglove-bridge v3.2.2 설치
- `foxglove.sdk.v1` 프로토콜 사용
- Lichtblick과 호환 안 됨

**변경 후:**
- apt로 의존성 설치 (v3.2.2)
- 빌드 시점에 husarion 이미지에서 v0.7.2 복사
- `foxglove.websocket.v1` 프로토콜 사용
- Lichtblick과 호환 ✅

## Dockerfile 구조

```dockerfile
# Multi-stage build: Get v0.7.2 from husarion image
FROM husarion/foxglove-bridge:humble AS foxglove-bridge-072

# Main build stage
FROM robot-base:humble

# ... (기존 설정) ...

# Install apt version (for dependencies)
RUN apt-get install -y ros-humble-foxglove-bridge

# Replace with v0.7.2 (Lichtblick compatible)
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/foxglove_bridge/foxglove_bridge /opt/ros/humble/lib/foxglove_bridge/foxglove_bridge
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/libfoxglove_bridge_base.so /opt/ros/humble/lib/libfoxglove_bridge_base.so
COPY --from=foxglove-bridge-072 /opt/ros/humble/lib/libfoxglove_bridge_component.so /opt/ros/humble/lib/libfoxglove_bridge_component.so
```

## 빌드 및 실행

### 1. 이미지 재빌드

```bash
cd shared_data/docker/ros2/serial-robot
docker compose --env-file ../.env build serial-robot-control
```

### 2. 컨테이너 재시작

```bash
docker compose --env-file ../.env up -d
```

### 3. foxglove-bridge 실행

```bash
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
```

### 4. 버전 확인

```bash
docker exec serial-robot-control bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 2>&1 | head -3"
```

**예상 출력 (v0.7.2):**
```
[INFO] [xxx] [foxglove_bridge]: Starting foxglove_bridge (humble, 0.7.2@) with WebSocket++/0.8.2
[WS] Server running without TLS
[WS] WebSocket server listening at ws://0.0.0.0:8765
```

**v3.2.2 출력 (호환 안 됨):**
```
[INFO] [xxx] [foxglove_bridge]: Starting foxglove_bridge (humble, 3.2.2@)
```

## Lichtblick 연결 테스트

### 1. Windows 포트 포워딩 (WSL2)

```powershell
# PowerShell (관리자 권한)
$wsl_ip = (wsl hostname -I).Trim()
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wsl_ip
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip
```

### 2. Lichtblick 접속

```
http://localhost:8080
```

### 3. WebSocket 연결

- "Open connection" → "Foxglove WebSocket"
- URL: `ws://localhost:8765`
- "Open" 클릭

### 4. 성공 확인

✅ **성공:**
- WebSocket 연결 성공
- Topics 목록 표시
- 3D 패널에서 URDF 표시 가능

❌ **실패:**
- "Protocol mismatch" 에러 → v0.7.2가 아닌 경우
- "Connection refused" → 포트 포워딩 문제

## 프로토콜 호환성

| 컴포넌트 | 버전 | 프로토콜 | Lichtblick 호환 |
|---------|------|----------|----------------|
| foxglove-bridge (apt) | v3.2.2 | `foxglove.sdk.v1` | ❌ |
| foxglove-bridge (husarion) | v0.7.2 | `foxglove.websocket.v1` | ✅ |
| Lichtblick | latest | `foxglove.websocket.v1` | ✅ |

## 장점

1. **빌드 시점 설정:** 런타임 복사 불필요
2. **Lichtblick 호환:** v0.7.2로 자동 설정
3. **husarion 이미지:** 빌드 단계에서만 사용 (런타임 의존성 없음)
4. **Windows 호환:** `/tmp` 경로 문제 없음

## 참고

- `WSL2_SETUP_COMPLETE.md` - WSL2 포트 포워딩 설정
- `ARCHITECTURE_2025-12-01.md` - 아키텍처 설계
- `README_OPENSOURCE.md` - 오픈소스 버전 가이드



