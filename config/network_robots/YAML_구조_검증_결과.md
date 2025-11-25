# Network Robots YAML 구조 검증 결과

## 검증 일시
2025-01-24

## 검증 대상 파일
1. xarm5.yaml (기준 파일)
2. lite6.yaml
3. ur3e.yaml
4. ur5e.yaml
5. ur16e.yaml
6. fr3.yaml
7. kinova_gen3_7dof.yaml
8. iiwa7.yaml
9. med14.yaml
10. med7.yaml

## 검증 결과

### ✅ 최상위 구조 일치
모든 파일이 다음 최상위 구조를 준수합니다:
- `robot` (필수)
- `frame_hierarchy` (필수)
- `ros2` (필수)
- `visualization_config` (선택)
- `ros2_visualization` (선택)

### ✅ robot 섹션 구조
모든 파일이 다음 필드를 포함합니다:
- `model`: 로봇 모델 이름
- `manufacturer`: 제조사 이름
- `id_pattern`: ID 패턴 (예: "{model}_{anchor}")

### ✅ frame_hierarchy 섹션 구조
모든 파일이 다음 필드를 포함합니다:
- `qr_frame_pattern`: QR 프레임 패턴 (예: "QR_{anchor}")
- `root_frame`: 루트 프레임 설정
  - `type`: "NetworkTF"
  - `id`: "{robot_id}"
  - `parent`: "QR_{anchor}"
- `network_frames`: NetworkTF 프레임 딕셔너리
  - `{robot_id}_base`: base 프레임
  - `{robot_id}_eef`: end-effector 프레임
  - `{robot_id}_work`: work 프레임 (critical: true)
- `local_frames`: 로컬 프레임 리스트 (시각화용)

### ✅ ros2 섹션 구조
모든 파일이 다음 필드를 포함합니다:
- `namespace`: ROS2 네임스페이스 (빈 문자열)
- `topics`: 토픽 설정
  - `tf`: "/tf"
  - `tf_static`: "/tf_static"
  - `joint_states`: "/joint_states"
- `prefab_links`: Prefab GameObject 매핑 딕셔너리
- `external_frames`: 외부 프레임 딕셔너리
  - `WorkFrame`: critical 프레임
  - `robot_origin`: 로봇 원점
- `subscribe_frames`: 구독할 프레임 리스트
- `joints`: 조인트 설정
  - `names`: 조인트 이름 리스트
  - `limits`: 조인트 제한 딕셔너리 (degrees)

### ✅ ros2_visualization 섹션 구조
모든 파일이 다음 필드를 포함합니다:
- `reference_frame`: 참조 프레임 (로봇별 base link)
- `features`: 시각화 기능
  - `show_current_pose`: true
  - `show_target_pose`: true
  - `show_path`: true
- `markers`: 마커 설정
  - `sphere_scale`: 0.05
  - `text_scale`: 0.03
  - `path_line_width`: 0.02
- `colors`: 색상 딕셔너리
  - `current`: [1.0, 0.0, 0.0, 1.0] (빨간색)
  - `target`: [0.0, 0.0, 1.0, 1.0] (파란색)
  - `path`: [0.0, 1.0, 0.0, 0.8] (녹색)
  - `success`: [0.0, 1.0, 0.0, 0.6] (녹색)
  - `text`: [1.0, 1.0, 1.0, 1.0] (흰색)

## 로봇별 특이사항

### UFACTORY (xarm5, lite6)
- `ros_frame`: `link_base`, `link_eef`
- `local_frames`: `link1`, `link2`, ... (DOF에 따라)
- `joints.names`: `joint1`, `joint2`, ...

### Universal Robots (ur3e, ur5e, ur16e)
- `ros_frame`: `base_link`, `tool0`
- `local_frames`: `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`
- `joints.names`: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`

### Franka (fr3)
- `ros_frame`: `fr3_link0`, `fr3_link8`
- `local_frames`: `fr3_link1`, `fr3_link2`, ..., `fr3_link7`
- `joints.names`: `fr3_joint1`, `fr3_joint2`, ..., `fr3_joint7`

### Kinova (kinova_gen3_7dof)
- `ros_frame`: `base_link`, `end_effector_link`
- `local_frames`: `shoulder_link`, `half_arm_1_link`, `half_arm_2_link`, `forearm_link`, `spherical_wrist_1_link`, `spherical_wrist_2_link`, `bracelet_link`
- `joints.names`: `joint_1`, `joint_2`, ..., `joint_7`

### KUKA (iiwa7, med14, med7)
- `ros_frame`: `{model}_link_0`, `{model}_link_ee`
- `local_frames`: `{model}_link_1`, `{model}_link_2`, ..., `{model}_link_7`
- `joints.names`: `{model}_A1`, `{model}_A2`, ..., `{model}_A7`

## Unity 로드 경로 확인

### AppDataManager.cs 로드 프로세스
1. `file_manifest.yaml`에서 `robot_configs` 섹션 읽기
2. 각 `network_robots/*.yaml` 파일을 `NetworkRobotData` 타입으로 deserialize
3. `robotData.Model`을 키로 하여 `networkRobots` 딕셔너리에 저장
4. `GetNetworkRobot(string model)` 메서드로 접근 가능

### 필수 조건
- ✅ 모든 파일이 `file_manifest.yaml`의 `robot_configs`에 등록되어 있음
- ✅ 모든 파일이 `type: "NetworkRobotData"`로 설정되어 있음
- ✅ 모든 파일이 `robot.model` 필드를 포함하고 있음
- ✅ 모든 파일이 NetworkRobotData C# 클래스 구조와 일치함

## 결론

**✅ 모든 YAML 파일이 xarm5.yaml의 구조를 준수하며, Unity에서 정상적으로 로드될 수 있습니다.**

### 검증 완료 항목
- [x] 최상위 구조 일치
- [x] 필수 필드 존재
- [x] 필드 이름 일치 (snake_case)
- [x] 데이터 타입 일치
- [x] file_manifest.yaml 등록 확인
- [x] Unity 로드 경로 확인

