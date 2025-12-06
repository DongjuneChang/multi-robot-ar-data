# Foxglove Studio 오픈소스 버전 사용 가이드

**목적:** 로그인 없이 사용 가능한 오픈소스 Foxglove Studio 사용

## 옵션 비교

| 방법 | 장점 | 단점 | 추천도 |
|------|------|------|--------|
| **Lichtblick** (현재) | 최신 업데이트, 활발한 유지보수 | Foxglove 원본과 약간 다를 수 있음 | ⭐⭐⭐⭐ |
| **ghcr.io/foxglove/studio:1.84.0** | 간단, 빠른 빌드 | 22개월 전 버전 (오래됨) | ⭐⭐⭐ |
| **소스 빌드** | 최신 오픈소스 기능, 완전한 제어 | 빌드 시간 오래 걸림 | ⭐⭐⭐⭐⭐ |

## 방법 1: Lichtblick 사용 (현재 설정)

이미 코드베이스에 있음:
```bash
cd shared_data/docker/ros2/foxglove
docker build -f Dockerfile.studio -t foxglove-studio:local .
docker run -d -p 8080:8080 foxglove-studio:local
```

**장점:**
- ✅ 로그인 불필요
- ✅ 최신 업데이트 (2025-11-14)
- ✅ iframe 호환
- ✅ 활발한 커뮤니티 유지보수

---

## 방법 2: 오래된 Foxglove Studio 버전 사용

**husarion이 사용한 방법:**
```dockerfile
FROM ghcr.io/foxglove/studio:1.84.0
```

**빌드:**
```bash
docker build -t foxglove-studio:1.84.0 - <<EOF
FROM ghcr.io/foxglove/studio:1.84.0
EXPOSE 8080
EOF

docker run -d -p 8080:8080 foxglove-studio:1.84.0
```

**주의:**
- ⚠️ 22개월 전 버전 (2023년 초)
- ⚠️ 보안 업데이트 없음
- ⚠️ 최신 기능 없음

---

## 방법 3: GitHub에서 소스 빌드 (권장)

**Dockerfile 사용:**
```bash
cd shared_data/docker/ros2/foxglove
docker build -f Dockerfile.studio.opensource -t foxglove-studio:opensource .
docker run -d -p 8080:8080 foxglove-studio:opensource
```

**특정 버전 지정:**
```bash
docker build -f Dockerfile.studio.opensource \
  --build-arg FOXGLOVE_VERSION=v1.84.0 \
  -t foxglove-studio:1.84.0 .
```

**최신 오픈소스 버전:**
```bash
# 최신 main 브랜치 (로그인 요구 전 커밋)
docker build -f Dockerfile.studio.opensource \
  --build-arg FOXGLOVE_VERSION=main \
  -t foxglove-studio:latest .
```

**장점:**
- ✅ 완전한 제어
- ✅ 원하는 버전 선택 가능
- ✅ 로그인 요구 전 커밋 사용 가능
- ✅ 커스터마이징 가능

---

## 로그인 요구 전 버전 찾기

**GitHub에서 확인:**
1. https://github.com/foxglove/studio/releases
2. 로그인 요구가 추가된 커밋 이전 버전 찾기
3. 해당 태그/커밋 해시 사용

**예상 버전:**
- `v1.84.0` - husarion이 사용 (2023년 초)
- `v1.85.0`, `v1.86.0` 등 - 이후 버전들
- 특정 커밋 해시 사용 가능

---

## docker-compose.yml 설정

```yaml
services:
  foxglove-studio:
    build:
      context: .
      dockerfile: Dockerfile.studio.opensource
      args:
        FOXGLOVE_VERSION: v1.84.0  # 또는 원하는 버전
    container_name: foxglove-studio
    ports:
      - "8080:8080"
    restart: unless-stopped
```

---

## 권장 사항

**현재 상황:**
- Lichtblick이 이미 잘 작동 중
- 로그인 불필요
- 최신 업데이트

**추천:**
1. **Lichtblick 계속 사용** (가장 간단)
2. 원본이 필요하면 **소스 빌드** (방법 3)
3. 빠른 테스트용으로 **v1.84.0** (방법 2)

---

## 참고

- Foxglove Studio GitHub: https://github.com/foxglove/studio
- Lichtblick GitHub: https://github.com/lichtblick-suite/lichtblick
- ARCHITECTURE 문서: `ARCHITECTURE_2025-12-01.md`



