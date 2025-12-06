# Multi-Robot AR Data Configuration

HoloLens 2 AR 멀티유저 로봇 제어 시스템의 **설정 파일 저장소**

## 📁 파일 구조 (v3.0)

```
multi-robot-ar-data/
├── config/                    # 모든 설정 파일
│   ├── network_config.yaml    # 네트워크/서버 설정
│   ├── device_config.yaml     # HoloLens 디바이스 설정
│   ├── robot_qr_mapping.yaml  # QR 코드-로봇 매핑
│   │
│   ├── network_robots/        # 로봇별 설정
│   │   ├── _defaults.yaml
│   │   └── overrides/         # lite6, xarm5, ur3e, ur5e, etc.
│   │
│   ├── ai/                    # AI 설정 (v2.3+)
│   │   ├── _defaults.yaml
│   │   ├── providers/         # ollama, openai, anthropic, google
│   │   ├── knowledge/         # Expert profiles & domains
│   │   └── ai_user_preferences/  # 사용자별 AI 선호도
│   │
│   ├── users/                 # 사용자 설정
│   │   ├── _defaults.yaml
│   │   └── overrides/         # 사용자별 오버라이드
│   │
│   ├── unit_patterns/        # 움직임 패턴 (unit-based)
│   ├── ros2_interface/        # ROS2 토픽/서비스 정의
│   ├── policies/             # 전략/정책 설정
│   └── projects/             # 프로젝트 정의
│
├── file_manifest.yaml         # 파일 인덱스 및 메타데이터
└── README.md                  # 이 파일
```

## 🔧 주요 설정 파일

### Core Configs
- **network_config.yaml**: 서버 엔드포인트 (web, ai, ros2, photon), 스트리밍, 타임아웃
- **device_config.yaml**: HoloLens Device Portal 접속 정보, 디바이스별 설정
- **robot_qr_mapping.yaml**: QR 코드와 로봇 타입 매핑

### Robot Configs
- **network_robots/**: 로봇별 설정 (lite6, xarm5, ur3e, ur5e, ur16e, fr3, iiwa7, med7, med14, kinova_gen3_7dof)
- Joint angles는 **DEGREES** 단위 (ROS2에서 radians로 변환)

### AI Configs (v2.3+)
- **ai/providers/**: AI 제공자 설정 (ollama, openai, anthropic, google)
- **ai/knowledge/**: Expert 프로필 및 도메인 지식
- **ai/ai_user_preferences/**: 사용자별 AI 선호도

### Patterns
- **unit_patterns/**: Unit-based 좌표 (-1 to 1 범위)
- 실제 위치 = `unit_value * defaults.size`

## 🔄 Unity에서 자동 다운로드

```csharp
// GitHubUtility3_2 사용
var gitHub = new GitHubUtility3_2("main_v3.0");
var robotConfig = await gitHub.DownloadAndParseJsonAsync<RobotConfig>("config/network_robots/overrides/lite6.yaml");
var networkConfig = await gitHub.DownloadAndParseJsonAsync<NetworkConfig>("config/network_config.yaml");
```

## 📚 상세 문서

- [config/README.md](config/README.md) - 설정 구조 상세 설명
- [file_manifest.yaml](file_manifest.yaml) - 전체 파일 인덱스

## 📅 Last Updated

2025-12-05 by Dongjune Chang (v3.0 - 간소화된 구조로 재구성)
