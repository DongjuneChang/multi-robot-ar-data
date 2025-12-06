# WSL2에서 Foxglove Bridge 연결 문제 해결

**문제:** WSL2에서 `network_mode: "host"`를 사용하면 Windows에서 포트 매핑이 무시됨

## 해결 방법

### 방법 1: network_mode 제거하고 포트 매핑 사용 (권장)

`docker-compose.yml`에서 `network_mode: "host"`를 제거하고 포트 매핑 사용:

```yaml
services:
  serial-robot-control:
    # network_mode: "host"  # 이 줄 제거 또는 주석 처리
    ports:
      - "8765:8765"  # foxglove-bridge
```

**주의:** ROS2 통신을 위해 같은 네트워크에 있어야 함

### 방법 2: WSL2 IP 주소 직접 사용

브라우저에서 WSL2 IP 주소 사용:

1. WSL2 IP 확인:
   ```bash
   wsl hostname -I
   # 예: 130.202.162.87
   ```

2. 브라우저에서 접속:
   ```
   http://130.202.162.87:8080
   ws://130.202.162.87:8765
   ```

### 방법 3: Windows 포트 포워딩 설정

PowerShell을 관리자 권한으로 실행:

```powershell
# WSL2 IP 확인
wsl hostname -I

# 포트 포워딩 설정 (예: WSL2 IP가 130.202.162.87인 경우)
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=130.202.162.87
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=130.202.162.87

# 확인
netsh interface portproxy show all

# 삭제 (필요시)
# netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0
```

**주의:** WSL2 재시작 시 IP가 변경될 수 있으므로 스크립트로 자동화 필요

---

## 현재 상황

- WSL2 IP: `130.202.162.87`
- 컨테이너 IP: `192.168.65.6` (Docker bridge 네트워크)

## 빠른 테스트

1. **WSL2 IP로 접속:**
   ```
   http://130.202.162.87:8080
   ws://130.202.162.87:8765
   ```

2. **또는 localhost 사용 (포트 포워딩 설정 후):**
   ```
   http://localhost:8080
   ws://localhost:8765
   ```

---

## 권장 해결책

**가장 간단한 방법:** 브라우저에서 WSL2 IP 주소 직접 사용

1. WSL2 IP 확인: `wsl hostname -I`
2. Lichtblick 접속: `http://<WSL2_IP>:8080`
3. WebSocket 연결: `ws://<WSL2_IP>:8765`

**영구적 해결:** `network_mode: "host"` 제거하고 포트 매핑 사용 (ROS2 통신 확인 필요)



