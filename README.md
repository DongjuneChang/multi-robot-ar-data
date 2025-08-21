# Multi-Robot AR Data Configuration

HoloLens 2 AR 멀티유저 로봇 제어 시스템의 **설정 파일 저장소**

## 📁 파일 구조

```
multi-robot-ar-data/
├── map_data.yaml         # QR 코드 및 사용자 등록 정보
├── device_config.yaml    # HoloLens 2 디바이스 설정
├── streaming_config.yaml # MRC 스트리밍 최적화 설정
└── README.md
```

## 🔧 설정 파일 설명

### map_data.yaml
- QR 코드 등록 정보 (위치, ID)
- 사용자 권한 및 디바이스 정보
- 스폰된 로봇 초기 설정

### device_config.yaml
- HoloLens Device Portal 접속 정보
- 각 사용자별 디바이스 IP/이름
- 스트리밍 품질 프리셋

### streaming_config.yaml
- MRC 딜레이 최적화 설정
- 품질별 프리셋 (low_latency, standard, high_quality)

## 🔄 Unity에서 자동 다운로드

```csharp
// GitHubUtility3_2 사용
var gitHub = new GitHubUtility3_2("main");
var mapData = await gitHub.DownloadAndParseJsonAsync<MapData>("map_data.yaml");
```

## 📝 업데이트 방법

1. 파일 수정 후 commit & push
2. Unity/HoloLens 앱이 시작 시 자동으로 최신 버전 다운로드
3. 로컬 캐시와 비교하여 변경사항만 적용

## ⚠️ 주의사항

- **비밀번호나 민감한 정보는 절대 commit하지 마세요**
- 실시간 tracking_data.json은 이 레포에 포함하지 않음
- Python 서버 코드는 별도 private 레포에서 관리

## 📅 Last Updated

2025-08-21 by Dongjune Chang