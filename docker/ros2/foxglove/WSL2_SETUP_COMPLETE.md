# WSL2에서 Foxglove Bridge 설정 완료 가이드

**Date:** 2025-12-04  
**환경:** WSL2 + Windows 10/11 + Docker Desktop

## 문제 상황

1. **포트 접근 문제:** WSL2에서 `network_mode: "host"` 사용 시 Windows에서 포트 접근 불가
2. **프로토콜 호환성 문제:** foxglove-bridge v3.2.2는 `foxglove.sdk.v1` 사용, Lichtblick은 `foxglove.websocket.v1`만 지원

## 해결 방법

### 1. Windows 포트 포워딩 설정

PowerShell을 **관리자 권한**으로 실행:

```powershell
# WSL2 IP 확인
$wsl_ip = (wsl hostname -I).Trim()
Write-Host "WSL2 IP: $wsl_ip"

# 포트 포워딩 설정
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wsl_ip
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip

# 확인
netsh interface portproxy show all

# 삭제 (필요시)
# netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0
# netsh interface portproxy delete v4tov4 listenport=8080 listenaddress=0.0.0.0
```

**주의:** WSL2 재시작 시 IP가 변경될 수 있으므로, 재시작 후 다시 설정 필요

### 2. foxglove-bridge v0.7.2로 변경 (프로토콜 호환성)

**현재 문제:**
```
Check that the WebSocket server at ws://localhost:8765 is reachable 
and supports protocol version foxglove.websocket.v1.
```

**원인:**
- foxglove-bridge v3.2.2 (apt 설치) → `foxglove.sdk.v1` 프로토콜
- Lichtblick → `foxglove.websocket.v1` 프로토콜만 지원
- 프로토콜 불일치로 연결 실패

**해결: v0.7.2 라이브러리 복사**

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

# 3. foxglove-bridge 재시작
docker exec serial-robot-control bash -c "pkill -f foxglove_bridge"
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
```

**확인:**
```bash
# v0.7.2로 시작되었는지 확인 (로그에 버전 표시)
docker exec serial-robot-control bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 2>&1 | head -3"
# 예상 출력: Starting foxglove_bridge (humble, 0.7.2@)
```

### 3. MoveIt 실행 (publish_robot_description=true)

```bash
# 기존 MoveIt 종료
docker exec serial-robot-control bash -c "pkill -f moveit"

# 재시작 (publish_robot_description=true 필수)
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py show_rviz:=false publish_robot_description:=true"

# 확인
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 3 ros2 topic echo /robot_description --once | head -5"
```

---

## 전체 실행 순서

### 1단계: Windows 포트 포워딩 설정

```powershell
# PowerShell (관리자 권한)
$wsl_ip = (wsl hostname -I).Trim()
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wsl_ip
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip
```

### 2단계: foxglove-bridge v0.7.2 설정

```bash
# WSL2 터미널에서
# (위의 "v0.7.2 라이브러리 복사" 섹션 참조)
```

### 3단계: 컨테이너 실행

```bash
# serial-robot-control 시작
cd shared_data/docker/ros2/serial-robot
docker compose --env-file ../.env up -d

# foxglove-studio (Lichtblick) 시작
cd ../foxglove
docker run -d -p 8080:8080 --name foxglove-studio ghcr.io/lichtblick-suite/lichtblick:latest
```

### 4단계: MoveIt 및 foxglove-bridge 실행

```bash
# MoveIt 실행
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py show_rviz:=false publish_robot_description:=true"

# foxglove-bridge 실행 (v0.7.2)
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
```

### 5단계: 브라우저에서 접속

1. **Lichtblick 접속:**
   ```
   http://localhost:8080
   ```

2. **WebSocket 연결:**
   - "Open connection" → "Foxglove WebSocket"
   - URL: `ws://localhost:8765`
   - "Open" 클릭

3. **3D 패널 설정:**
   - "Add panel" → "3D"
   - "Add layer" → "URDF"
   - Source: "Topic"
   - Topic: `/robot_description`
   - "Add" 클릭

---

## 확인 사항

### 포트 포워딩 확인

```powershell
# Windows PowerShell
netsh interface portproxy show all
```

**예상 출력:**
```
ipv4 수신 대기:             ipv4에 연결:
주소            포트        주소            포트
--------------- ----------  --------------- ----------
0.0.0.0         8765        130.202.162.87  8765
0.0.0.0         8080        130.202.162.87  8080
```

### foxglove-bridge 실행 확인

```bash
docker exec serial-robot-control bash -c "ps aux | grep foxglove_bridge | grep -v grep"
```

**예상 출력:**
```
root      1660  ... /usr/bin/python3 /opt/ros/humble/bin/ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
root      1661  ... /opt/ros/humble/lib/foxglove_bridge/foxglove_bridge --ros-args -p port:=8765
```

### /robot_description 토픽 확인

```bash
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 2 ros2 topic echo /robot_description --once | head -3"
```

**예상 출력:**
```
data: <?xml version="1.0" ?><!-- =================================================================================== --><!-- |    This...
```

---

## 문제 해결

### 문제 1: "WebSocket connection failed"

**원인:** 프로토콜 불일치 (v3.2.2 vs v0.7.2)

**해결:** v0.7.2 라이브러리 복사 (위 "foxglove-bridge v0.7.2로 변경" 참조)

### 문제 2: "No coordinate frames found"

**원인:** `/robot_description` 토픽이 발행되지 않음

**해결:** MoveIt을 `publish_robot_description:=true`로 재시작

### 문제 3: 포트 포워딩이 작동하지 않음

**원인:** WSL2 IP가 변경됨

**해결:** WSL2 IP 재확인 후 포트 포워딩 재설정

```powershell
$wsl_ip = (wsl hostname -I).Trim()
Write-Host "New WSL2 IP: $wsl_ip"
# 기존 포워딩 삭제 후 재설정
```

### 문제 4: WSL2 재시작 후 포트 포워딩 사라짐

**해결:** 자동화 스크립트 생성

**Windows 스크립트 (`setup-port-forward.ps1`):**
```powershell
# 관리자 권한으로 실행 필요
$wsl_ip = (wsl hostname -I).Trim()
Write-Host "Setting up port forwarding for WSL2 IP: $wsl_ip"

# 기존 포워딩 삭제
netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0
netsh interface portproxy delete v4tov4 listenport=8080 listenaddress=0.0.0.0

# 새 포워딩 설정
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wsl_ip
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip

Write-Host "Port forwarding configured:"
netsh interface portproxy show all
```

---

## 참고 문서

- `WSL2_FIX.md` - WSL2 네트워크 문제 해결
- `TROUBLESHOOTING.md` - 일반적인 문제 해결
- `ARCHITECTURE_2025-12-01.md` - 아키텍처 설계 문서
- `QUICK_TEST.md` - 빠른 테스트 가이드

---

## 요약

✅ **성공 기준:**
1. Windows 포트 포워딩 설정 완료
2. foxglove-bridge v0.7.2 실행 중
3. MoveIt 실행 중 (`publish_robot_description:=true`)
4. `/robot_description` 토픽 발행 중
5. 브라우저에서 `http://localhost:8080` 접속 성공
6. WebSocket 연결 성공 (`ws://localhost:8765`)
7. 3D 패널에서 Lite6 로봇 표시

❌ **실패 시:**
- 브라우저 콘솔 (F12) 확인
- 위의 "문제 해결" 섹션 참조
- 각 단계별 확인 사항 점검



