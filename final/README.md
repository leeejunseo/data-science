# 6DOF 미사일 시뮬레이션 프로젝트

탄도 미사일(SCUD-B, Nodong, KN-23)의 6자유도 비행 시뮬레이션 및 시그니처 분석 시스템

## 📁 NPZ 저장 포맷 (표준화됨)

### 표준 포맷
**모든 trajectory NPZ 저장은 `trajectory_io.save_trajectory_unified()` 함수를 통해서만 수행됩니다.**

```python
from trajectory_io import save_trajectory_unified

# 시뮬레이션 결과 저장
filepath = save_trajectory_unified(
    results=sim_results,
    missile_type="SCUD-B",
    elevation=45.0,
    azimuth=90.0,
    seed=0
)
```

### 표준 키 세트
| 키 | 설명 | 단위 |
|----|------|------|
| `time` | 시간 배열 | s |
| `position_x` | X 위치 (North) | m |
| `position_y` | Y 위치 (East) | m |
| `position_z` | 고도 (Altitude) | m |
| `u`, `v`, `w` | Body Frame 속도 | m/s |
| `phi`, `theta`, `psi` | Euler 각도 (Roll, Pitch, Yaw) | rad |
| `p`, `q`, `r` | 각속도 | rad/s |
| `mass` | 질량 | kg |
| `V` | 속도 크기 | m/s |
| `gamma` | 비행경로각 | rad |
| `chi` | 방위각 | rad |

### 저장 경로 및 파일명 규칙
- **저장 폴더**: `./outputs/trajectories/`
- **파일명 형식**: `{missile_type}__elev{ELEV:.1f}__azi{AZI:.1f}__seed{SEED}__{YYYYMMDD_HHMMSS}.npz`
- **예시**: `KN-23__elev30.0__azi90.0__seed0__20251220_183000.npz`

---

## 🚀 실행 방법

### 1. 환경 설정
```bash
cd final
pip install -r requirements.txt
```

### 2. 시각화 실행 (main_visualization.py)
```bash
python main_visualization.py
```

### 3. 게임/서버 모드 (missile_manager.py)
```bash
python missile_manager.py --demo --angle 45
python missile_manager.py --server --port 5000
```

### 4. 배치 시뮬레이션 (run_kn23_batch.py)
```bash
python run_kn23_batch.py
```

### 5. NPZ 파일 확인 (view_npz.py)
```bash
python view_npz.py outputs/trajectories/SCUD-B__elev45.0__azi90.0__seed0__20251220_183000.npz
```

---

## 🤖 ML 학습 파이프라인

### 대량 학습 (train_large.py)

```bash
python train_large.py
```

**기능:**
- ~5000개 샘플 생성 (미사일 3종 × 33개 발사각 × 50 샘플/각도)
- 32차원 시그니처 특성 추출
- RandomForest 분류 모델 학습
- 혼동행렬/분류리포트/정확도 출력
- **중요 시그니처 TOP10** 출력

**출력 파일:**
| 파일 | 설명 |
|------|------|
| `signature_dataset/dataset_large.npz` | 학습 데이터셋 |
| `signature_dataset/model_rf.joblib` | 학습된 RandomForest 모델 |
| `signature_dataset/scaler.joblib` | StandardScaler |

### 발사각 그룹 평가 (eval_by_angle.py)

```bash
python eval_by_angle.py
```

**기능:**
- `dataset_large.npz` 로드
- **GroupShuffleSplit**으로 발사각(nominal_angle) 단위 train/test 분리
- 학습하지 않은 발사각에서의 일반화 성능 평가 (누수 방지)
- 테스트에 사용된 발사각 목록 출력

**평가 방식 비교:**

| 평가 방식 | 설명 | 목적 |
|-----------|------|------|
| 랜덤 Split | 샘플 단위 무작위 분할 | 기본 분류 성능 |
| 발사각 그룹 Split | 발사각 단위 분할 | 일반화 성능 (누수 방지) |

### 시그니처 특성 (32차원)

```
기하학적 (8개): max_altitude_km, final_range_km, altitude_range_ratio, ...
속도 (6개): max_velocity, burnout_velocity, terminal_velocity, ...
가속도 (4개): max_acceleration, max_deceleration, burn_time_ratio, ...
6DOF 고유 (10개): alpha_max_deg, q_max_deg_s, alpha_q_correlation, ...
파생 (4개): ballistic_coefficient, energy_ratio, glide_ratio, ...
```

---

## 📊 NPZ 파일 검증

```python
from trajectory_io import load_trajectory, validate_trajectory

# 로드 및 검증
data = load_trajectory("outputs/trajectories/example.npz")
is_valid = validate_trajectory(data)
```

---

## 📂 프로젝트 구조

```
final/
├── trajectory_io.py          # NPZ 저장/로드 표준 모듈 (핵심!)
├── missile_6dof_true.py      # True 6DOF 시뮬레이터 (Quaternion)
├── kn23_depressed.py         # KN-23 편평 탄도 시뮬레이터
├── main_visualization.py     # 시각화 시스템
├── missile_manager.py        # 게임/API 서버
├── signature_extractor.py    # 시그니처 추출
├── signature_generator.py    # 분류 데이터셋 생성
├── train_large.py            # 대량 학습 파이프라인 ★
├── eval_by_angle.py          # 발사각 그룹 평가 ★
├── view_npz.py               # NPZ 뷰어
├── config_6dof.py            # 6DOF 설정
├── outputs/
│   └── trajectories/         # 표준 NPZ 저장 폴더
├── signature_dataset/        # ML 학습 데이터/모델 ★
│   ├── dataset_large.npz
│   ├── model_rf.joblib
│   └── scaler.joblib
└── README.md
```

---

## ⚠️ 주의사항

1. **NPZ 저장은 반드시 `trajectory_io` 모듈을 통해** 수행하세요.
2. `np.savez` / `np.savez_compressed`를 직접 호출하지 마세요.
3. 시그니처 데이터는 `save_npz_generic()` 함수를 사용하세요.

---

*Last updated: 2024-12*
