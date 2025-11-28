# Server API 폴더 구조 리팩토링 (2025-11-27)

## 목적
- API 파일들을 `api/` 폴더로 정리
- `RoutesManager` 싱글톤으로 경로 관리 통합
- 하드코딩된 상대경로 제거
- **최종 목표: docker/web으로 배포**

---

## 1. 구조 변경 완료 ✅

### Before (이전)
```
server/
├── main.py
├── config_api.py        # 루트에 흩어져 있음
├── ros_api.py
├── tracking_api.py
├── ai/
│   └── riley_api.py
└── __pycache__/
```

### After (현재) ✅
```
server/
├── main.py              # RoutesManager 통합 완료
├── routes_manager.py    # 경로 관리 싱글톤
├── requirements.txt
├── REFACTORING_2025-11-27.md
└── api/                 # API 핸들러들 모음
    ├── __init__.py
    ├── config_api.py    # /api/config
    ├── ros_api.py       # /api/ros2
    ├── tracking_api.py  # /api/tracking
    └── ai/
        ├── __init__.py
        └── riley_api.py # /api/ai (예정)
```

---

## 2. 완료된 작업 체크리스트

### 2.1 폴더 생성 ✅
- [x] `server/api/` 폴더 생성
- [x] `server/api/__init__.py` 생성
- [x] `server/api/ai/__init__.py` 생성

### 2.2 파일 이동 ✅
- [x] `config_api.py` → `api/config_api.py`
- [x] `ros_api.py` → `api/ros_api.py`
- [x] `tracking_api.py` → `api/tracking_api.py`
- [x] `ai/riley_api.py` → `api/ai/riley_api.py`
- [x] `ai/` 폴더 삭제

### 2.3 import 수정 ✅
- [x] `main.py`의 import 경로 수정
  ```python
  from api.config_api import ConfigAPIHandler, RobotConfigAPIHandler
  from api.tracking_api import TrackingAPIHandler
  from api.ros_api import ROS2APIHandler
  from routes_manager import RoutesManager
  ```

### 2.4 RoutesManager 통합 ✅
- [x] `main.py`에서 RoutesManager.initialize() 사용
- [x] WebServer가 RoutesManager에서 경로 가져옴
- [x] `/api/routes` blueprint 등록

### 2.5 테스트 ✅
- [x] Import 테스트 통과
- [x] WebServer 초기화 테스트 통과

---

## 3. 진행 상황

| 단계 | 상태 | 비고 |
|------|------|------|
| routes_manager.py 생성 | ✅ 완료 | 싱글톤 패턴 |
| api/ 폴더 구조 생성 | ✅ 완료 | __init__.py 포함 |
| 파일 이동 | ✅ 완료 | 4개 파일 이동 |
| import 수정 | ✅ 완료 | main.py 수정 |
| 테스트 | ✅ 완료 | 초기화 성공 |
| docker/web 배포 | 🔄 진행중 | 다음 단계 |

---

## 4. 다음 단계: Docker 배포

### 4.1 docker/web 구조 확인 필요
- [ ] `docker/web/` 폴더 현재 상태 확인
- [ ] Dockerfile 수정 (새 api/ 구조 반영)
- [ ] docker-compose.yml 수정

### 4.2 경로 설정
- `SHARED_DATA_DIR` 환경변수 사용
- Docker 내부: `/app/shared_data`
- 로컬: `Path(__file__).parent.parent`

---

## 5. 새로운 API 엔드포인트

`/api/routes/paths` - 프론트엔드에서 모든 API 경로 조회 가능

```json
{
  "api_paths": {
    "config": "/api/config",
    "robots": "/api/robots",
    "ros2_interface": "/api/config/ros2_interface",
    "tracking": "/api/tracking",
    ...
  },
  "static_routes": { ... },
  "shared_data_dir": "/app/shared_data"
}
```

---

## 6. 주의사항

1. **디버깅 지옥 영역** - import 경로 오류는 찾기 어려움
2. **한 단계씩** - 이동 후 즉시 확인
3. **조급증 금지** - 천천히, 확실하게
4. **Docker 테스트** - 로컬 성공 후 Docker에서도 테스트

---

## 7. 관련 파일

- `routes_manager.py` - 경로 관리 싱글톤 ✅
- `main.py` - WebServer 클래스 ✅
- `api/*.py` - API 핸들러들 ✅
- `docker/web/` - Docker 배포 (다음 단계)
