# 현재 상태 및 문제점 (2025-12-04)

## 현재 실행 중인 컨테이너

✅ **실행 중:**
- `serial-robot-control` - ROS2 컨테이너
- `foxglove-bridge` - WebSocket 서버 (포트 8765)
- `foxglove-studio` (Lichtblick) - 포트 8080

## 확인된 문제

### 1. `/robot_description` 토픽 발행 문제

**상황:**
- 토픽은 존재하지만 데이터가 발행되지 않음
- `ros2 topic echo /robot_description --once` → 출력 없음
- `ros2 topic hz /robot_description` → 응답 없음

**원인:**
- `/move_group`의 `publish_robot_description` 파라미터가 `False`로 설정됨
- 이미 `true`로 변경했지만 여전히 문제가 있음

**해결 방법:**

#### 방법 1: MoveIt Launch 파일 수정 (영구적)

MoveIt launch 파일에서 `publish_robot_description:=true` 추가:

```bash
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py show_rviz:=false publish_robot_description:=true
```

#### 방법 2: 별도 노드로 발행

`robot_state_publisher`가 `/robot_description`을 발행하도록 설정:

```bash
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\$(ros2 param get /move_group robot_description) -p publish_robot_description:=true"
```

#### 방법 3: Python 스크립트로 수동 발행

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.pub = self.create_publisher(String, '/robot_description', 10)
        
        # Get robot_description from parameter server
        result = subprocess.run(
            ['ros2', 'param', 'get', '/move_group', 'robot_description'],
            capture_output=True, text=True
        )
        urdf = result.stdout.strip()
        
        # Publish every 1 second
        self.timer = self.create_timer(1.0, lambda: self.pub.publish(String(data=urdf)))
        self.get_logger().info('Publishing /robot_description')

def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 2. Foxglove Bridge 버전 호환성 문제

**상황:**
- foxglove-bridge가 v3.2.2일 가능성 (Lichtblick과 호환 안 됨)
- 프로토콜: `foxglove.sdk.v1` vs `foxglove.websocket.v1`

**확인 방법:**
```bash
docker exec foxglove-bridge bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 2>&1 | head -3"
```

**예상 출력:**
- v3.2.2: 프로토콜 정보 없음
- v0.7.2: `Starting foxglove_bridge (humble, 0.7.2@)`

**해결 방법:**
v0.7.2 라이브러리 복사 (TROUBLESHOOTING.md 참조)

---

### 3. WebSocket 연결 문제

**확인 사항:**
1. 브라우저에서 `http://localhost:8080` 접속
2. WebSocket 연결: `ws://localhost:8765`
3. 브라우저 콘솔 (F12) 에러 확인

**예상 에러:**
- `Protocol mismatch` → 버전 문제
- `Connection refused` → 포트/네트워크 문제
- `Topic not found` → `/robot_description` 발행 안 됨

---

## 빠른 해결 체크리스트

### Step 1: MoveIt 재시작 (publish_robot_description=true)

```bash
# 기존 MoveIt 종료
docker exec serial-robot-control bash -c "pkill -f moveit"

# 재시작 (publish_robot_description=true 추가)
docker exec -d serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py show_rviz:=false publish_robot_description:=true"
```

### Step 2: /robot_description 확인

```bash
# 5초 대기 후 확인
sleep 5
docker exec serial-robot-control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 3 ros2 topic echo /robot_description --once | head -5"
```

### Step 3: Foxglove Bridge 버전 확인

```bash
docker exec foxglove-bridge bash -c "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 2>&1 | head -3"
```

### Step 4: 브라우저에서 테스트

1. `http://localhost:8080` 접속
2. WebSocket 연결: `ws://localhost:8765`
3. 3D 패널 → URDF layer 추가
4. Topic: `/robot_description` 선택

---

## 다음 단계

1. MoveIt을 `publish_robot_description:=true`로 재시작
2. `/robot_description` 토픽 발행 확인
3. 브라우저에서 Lichtblick 연결 테스트
4. 문제 지속 시 foxglove-bridge 버전 확인 및 v0.7.2로 변경



