# 실시간 시뮬레이션 업그레이드 완료

## 개요
미사일 식별 게임을 기존 저장된 .npz 파일 방식에서 **실시간 6DOF 시뮬레이션 + ML 분석** 방식으로 고도화했습니다.

## 주요 변경 사항

### 1. `main_visualization.py` - 프로그래밍 방식 시뮬레이션 지원

**추가된 함수:**
```python
def run_simulation_programmatic(missile_type, launch_angle_deg, azimuth_deg, seed=0)
```

**기능:**
- `game_launcher.py`에서 호출 가능한 프로그래밍 방식 시뮬레이션 실행
- 사용자 입력 없이 자동으로 시뮬레이션 실행 및 NPZ 저장
- 반환값: 생성된 NPZ 파일 경로

**파라미터:**
- `missile_type`: 'SCUD-B', 'Nodong', 'KN-23' 중 선택
- `launch_angle_deg`: 발사각 (도)
- `azimuth_deg`: 방위각 (도)
- `seed`: 시뮬레이션 시드

---

### 2. `game_launcher.py` - 실시간 시뮬레이션 통합

**변경된 함수: `select_random_missile()`**

**이전 방식:**
```python
# results_6dof/ 폴더에서 기존 .npz 파일 랜덤 선택
npz_file = random.choice(available[missile_type])
```

**새로운 방식:**
```python
# 1. 미사일 종류 랜덤 선택 (SCUD-B, Nodong, KN-23)
missile_type = random.choice(missile_types)

# 2. 고각 및 방위각 랜덤 설정
launch_angle = random.uniform(30, 60)  # 30~60도
azimuth = random.uniform(80, 110)      # 80~110도

# 3. 실시간 시뮬레이션 실행
npz_path = run_simulation_programmatic(
    missile_type=missile_type,
    launch_angle_deg=launch_angle,
    azimuth_deg=azimuth,
    seed=seed
)
```

**Fallback 메커니즘:**
- 실시간 시뮬레이션 실패 시 기존 NPZ 파일 사용
- `main_visualization` 모듈 import 실패 시에도 기존 방식으로 동작

**추가된 import:**
```python
from main_visualization import run_simulation_programmatic
```

---

### 3. `GameApp.js` - 동시 API 호출 구현

**변경된 함수: `handleAnalyze()`**

**이전 방식:**
```javascript
// 순차 실행
const response = await fetch(`${API_BASE}/api/analyze`);
// ...
fetch(`${API_BASE}/api/visualize`);  // 비동기 호출
```

**새로운 방식:**
```javascript
// 병렬 실행 (Promise.all 사용)
const [analyzeResponse, visualizeResponse] = await Promise.all([
    fetch(`${API_BASE}/api/analyze`),
    fetch(`${API_BASE}/api/visualize`)
]);
```

**효과:**
- `/api/analyze`: ML 모델로 실시간 생성된 데이터 분석
- `/api/visualize`: Python 그래프 창 띄우기
- **두 작업이 동시에 실행되어 응답 속도 향상**

**UI 업데이트:**
- 게임 방법 설명에 "실시간 6DOF 시뮬레이션" 명시
- 실시간 생성 파라미터 정보 추가 (고각 30~60°, 방위각 80~110°)

---

## 실행 흐름

### 전체 프로세스

```
1. 사용자: "시뮬레이션 시작" 버튼 클릭
   ↓
2. Frontend: /api/start 호출
   ↓
3. Backend (game_launcher.py):
   - select_random_missile() 실행
   - 미사일 종류 랜덤 선택 (SCUD-B/Nodong/KN-23)
   - 고각(30~60°), 방위각(80~110°) 랜덤 생성
   - run_simulation_programmatic() 호출
   ↓
4. main_visualization.py:
   - MissileVisualization6DOF 객체 생성
   - True6DOFSimulator 또는 KN23Depressed 시뮬레이터 실행
   - 결과를 NPZ 파일로 저장 (results_6dof/)
   - NPZ 경로 반환
   ↓
5. Backend: GameState.npz_path에 저장
   ↓
6. 사용자: "예측하기" 버튼 클릭
   ↓
7. Frontend: Promise.all로 동시 호출
   - /api/analyze: 15개 특징 추출 + ML 모델 예측
   - /api/visualize: main_visualization.py --file <npz> 실행
   ↓
8. Backend:
   - analyze: extract_15_features() → ML_MODEL.predict()
   - visualize: subprocess로 Python 그래프 창 띄우기
   ↓
9. Frontend: 분석 결과 UI 표시 + 그래프 창 팝업
   ↓
10. 사용자: 그래프 분석 후 미사일 종류 선택
```

---

## 기술적 세부사항

### 시뮬레이션 파라미터
- **고각 범위**: 30~60도 (현실적인 탄도 미사일 발사각)
- **방위각 범위**: 80~110도 (남쪽 방향 편차)
- **시뮬레이션 시간**: 600초 (자동)
- **시드**: 0~9999 랜덤 (재현 가능성)

### ML 특징 추출 (15개)
**레이더 시그니처 (12개):**
1. max_altitude_km
2. final_range_km
3. impact_angle_deg
4. total_flight_time
5. max_velocity
6. terminal_velocity
7. max_mach
8. velocity_loss_ratio
9. max_deceleration
10. ground_track_curvature
11. path_efficiency
12. energy_ratio

**6DOF 기동성 (3개):**
13. alpha_std_deg (받음각 표준편차)
14. q_max_deg_s (최대 피치율)
15. alpha_q_correlation (받음각-피치율 상관계수)

### NPZ 파일 저장 위치
```
final/results_6dof/
├── SCUD-B_<timestamp>_<seed>.npz
├── Nodong_<timestamp>_<seed>.npz
└── KN-23_<timestamp>_<seed>.npz
```

---

## 장점

### 1. **데이터 다양성 증가**
- 매 게임마다 새로운 궤적 생성
- 고각/방위각 랜덤화로 무한한 시나리오

### 2. **ML 모델 일반화 성능 검증**
- 학습 데이터에 없는 새로운 궤적으로 테스트
- 실전 배치 환경 시뮬레이션

### 3. **물리 엔진 신뢰성**
- True6DOFSimulator (Quaternion 기반)
- KN23Depressed (3DOF 편평 탄도)
- 실시간 검증 가능

### 4. **사용자 경험 향상**
- 매번 다른 시나리오로 재미 증가
- 실시간 생성 과정 투명성 확보

---

## 테스트 방법

### 1. 백엔드 서버 시작
```bash
cd final
python game_launcher.py
```

**확인 사항:**
- `✅ 실시간 시뮬레이션 완료: <npz_path>` 메시지
- `🎯 [HIDDEN] Selected missile: <type>` 출력

### 2. 프론트엔드 실행
```bash
cd lastpang
npm start
```

### 3. 게임 플레이
1. "시뮬레이션 시작" 클릭
2. "예측하기" 클릭
3. **확인:**
   - Python 그래프 창 자동 팝업
   - UI 우측 패널에 분석 결과 표시
   - 15개 특징값 확인

### 4. 로그 확인
**Backend 콘솔:**
```
============================================================
프로그래밍 방식 시뮬레이션 실행
미사일: KN-23, 고각: 45.3°, 방위각: 95.7°
============================================================
✓ 시뮬레이션 완료!
✓ NPZ 저장 완료: C:\...\results_6dof\KN-23_20241224_134512_1234.npz
```

**Frontend 콘솔:**
```
✓ Visualization launched successfully
```

---

## Fallback 메커니즘

### 시나리오 1: main_visualization import 실패
```python
if not _simulation_available:
    # 기존 NPZ 파일 사용
    npz_file = random.choice(available[missile_type])
```

### 시나리오 2: 시뮬레이션 실행 실패
```python
if not npz_path:
    print("⚠ 시뮬레이션 실패, 기존 NPZ 파일 사용")
    # Fallback to pre-saved files
```

### 시나리오 3: Backend 연결 실패
```javascript
catch (error) {
    // Frontend local identification
    const localIdent = generateLocalIdentification();
}
```

---

## 파일 변경 요약

| 파일 | 변경 내용 | 라인 수 |
|------|----------|---------|
| `final/main_visualization.py` | `run_simulation_programmatic()` 함수 추가 | +52 |
| `final/game_launcher.py` | `select_random_missile()` 재작성, import 추가 | +45 |
| `lastpang/src/GameApp.js` | `handleAnalyze()` Promise.all 적용, UI 업데이트 | +15 |

---

## 다음 단계 (선택 사항)

### 1. 성능 최적화
- 시뮬레이션 캐싱 (동일 파라미터 재사용)
- 백그라운드 시뮬레이션 (미리 생성)

### 2. 고급 기능
- 사용자 커스텀 파라미터 입력
- 시뮬레이션 진행률 표시 (WebSocket)

### 3. 데이터 분석
- 생성된 NPZ 파일 통계 분석
- ML 모델 재학습 파이프라인

---

## 문제 해결

### Q: "⚠ main_visualization 모듈 없음" 오류
**A:** `game_launcher.py`와 `main_visualization.py`가 같은 `final/` 폴더에 있는지 확인

### Q: 시뮬레이션이 너무 느림
**A:** 정상입니다. 6DOF 시뮬레이션은 5~10초 소요 (미사일 종류에 따라 다름)

### Q: NPZ 파일이 계속 쌓임
**A:** 주기적으로 `results_6dof/` 폴더 정리 권장 (또는 자동 삭제 스크립트 추가)

### Q: 그래프 창이 안 뜸
**A:** 
1. matplotlib backend 확인 (TkAgg)
2. Windows: CREATE_NEW_CONSOLE 플래그 확인
3. 수동 실행: `python main_visualization.py --file <npz_path>`

---

## 결론

✅ **실시간 시뮬레이션 통합 완료**
- 매 게임마다 새로운 궤적 생성
- ML 분석 + 시각화 동시 실행
- Fallback 메커니즘으로 안정성 확보

🚀 **이제 게임을 실행하면 실시간으로 미사일 궤적이 생성되고, ML 모델이 분석합니다!**
