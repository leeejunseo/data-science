# NPZ 궤적 데이터 포맷 통일 가이드

## 📋 문제 상황

조원이 지적한 대로, **3가지 NPZ 포맷이 혼재**하여 나중에 호환성 문제 발생 가능:

### 1️⃣ trajectory_io.py 표준 포맷
```python
# 필수 필드
position_x, position_y, position_z  # ← position_z
u, v, w  # Body Frame 속도
phi, theta, psi  # Euler 각도
p, q, r  # 각속도
mass, V, gamma, chi
```

### 2️⃣ missile_6dof.py (3DOF 기반)
```python
# 문제점
'altitude'  # ← position_z가 아님!
V, gamma, psi  # 있음
# u, v, w 없음! (3DOF라서)
```

### 3️⃣ missile_6dof_v2.py (6DOF)
```python
# 완벽한 Body Frame
position_x, position_y, position_z  # ✓
u, v, w  # ✓
phi, theta, psi, p, q, r  # ✓
# 하지만 저장 메서드가 없었음 (수정됨)
```

---

## ✅ 해결책: 어댑터 패턴

### 🔧 1. trajectory_format_adapter.py 사용 (자동 변환)

```python
from trajectory_format_adapter import TrajectoryFormatAdapter

# 어떤 포맷이든 자동 로드 + 표준화
adapter = TrajectoryFormatAdapter()
standard_data = adapter.load_and_convert("results/old_format.npz")

# 포맷 감지
fmt = adapter.detect_format(data)  # 'standard' | '3dof' | '6dof_v2'

# 검증
is_valid, missing = adapter.validate_standard_format(standard_data)
```

**지원 변환:**
- `altitude` → `position_z`
- `V, gamma, psi` → `u, v, w` (3DOF→Body Frame 근사)
- 누락 필드 자동 생성 (`phi=0`, `p/q/r=0` 등)

---

### 🔧 2. missile_6dof_v2.py 표준 저장 (권장!)

```python
from missile_6dof_v2 import Missile6DOF_Authentic

missile = Missile6DOF_Authentic("SCUD-B")
results = missile.simulate(elevation_deg=45, azimuth_deg=90)

# ✅ 표준 포맷으로 저장 (trajectory_io.py 호환)
missile.save_to_standard_npz(
    results=results,
    filepath="results_6dof/SCUD-B_45deg_standard.npz",
    launch_angle_deg=45
)
```

**자동 처리:**
- `gamma` 계산 (Body→Inertial 변환)
- `chi` = `psi` (방위각)
- `trajectory_io.save_trajectory()` 호출 → 표준 NPZ 생성

---

### 🔧 3. main_visualization.py (이미 호환)

```python
from main_visualization import MissileVisualization6DOF

viz = MissileVisualization6DOF("SCUD-B")

# 새 시뮬레이션
viz.run_simulation(launch_angle_deg=45)
npz_path = viz.save_to_npz()  # 이미 표준 포맷으로 저장

# 기존 NPZ 로드 (어댑터 없이도 작동)
viz.load_from_npz("results_6dof/old_file.npz")
```

---

## 📊 통합 워크플로우

### Case 1: 새 시뮬레이션 (권장)
```python
# 1. missile_6dof_v2.py로 시뮬레이션
from missile_6dof_v2 import Missile6DOF_Authentic

missile = Missile6DOF_Authentic("KN-23")
results = missile.simulate(elevation_deg=45, azimuth_deg=90)

# 2. 표준 NPZ 저장
missile.save_to_standard_npz(results, "results/KN23_45deg.npz", launch_angle_deg=45)

# 3. 시각화
from main_visualization import MissileVisualization6DOF
viz = MissileVisualization6DOF("KN-23")
viz.load_from_npz("results/KN23_45deg.npz")
viz.plot_comprehensive()
```

### Case 2: 기존 NPZ 변환 (레거시 호환)
```python
# 1. 어댑터로 기존 파일 로드 + 변환
from trajectory_format_adapter import TrajectoryFormatAdapter

adapter = TrajectoryFormatAdapter()
standard_data = adapter.load_and_convert("old_results/professor_format.npz")

# 2. 표준 포맷으로 재저장
from trajectory_io import save_trajectory
save_trajectory(
    filepath="results/converted_standard.npz",
    **standard_data  # 이미 표준 필드명
)

# 3. 시각화 (이제 호환됨)
viz = MissileVisualization6DOF("SCUD-B")
viz.load_from_npz("results/converted_standard.npz")
viz.plot_comprehensive()
```

### Case 3: predict() 함수용 (ML/분석)
```python
# predict() 함수에서 어댑터 사용
def predict(npz_path):
    from trajectory_format_adapter import TrajectoryFormatAdapter
    
    # 자동 포맷 감지 + 표준화
    adapter = TrajectoryFormatAdapter()
    data = adapter.load_and_convert(npz_path)
    
    # 이제 안전하게 접근 가능
    q_signal = data['q']  # 항상 존재
    position_z = data['position_z']  # altitude 아님!
    
    # 분석 로직...
    return prediction
```

---

## 🎯 결론 (조원 지적 반영)

### ✅ 해결된 문제들

1. **포맷 통일**: `trajectory_io.py` 표준 포맷 확립
2. **자동 변환**: `trajectory_format_adapter.py`로 레거시 지원
3. **표준 저장**: `missile_6dof_v2.save_to_standard_npz()`
4. **호환성**: 기존 코드 수정 없이 어댑터만 추가

### 📝 권장사항

**✅ 새 코드 작성 시:**
```python
# 항상 missile_6dof_v2.py + save_to_standard_npz() 사용
missile.save_to_standard_npz(results, filepath, launch_angle_deg)
```

**✅ 기존 파일 처리 시:**
```python
# 어댑터로 자동 변환
adapter = TrajectoryFormatAdapter()
standard_data = adapter.load_and_convert(legacy_npz)
```

**✅ predict() / 분석 함수:**
```python
# 함수 첫 줄에 어댑터 적용
data = TrajectoryFormatAdapter.load_and_convert(npz_path)
# 이후 안전하게 data['position_z'], data['u'] 등 사용
```

---

## 🔍 필드명 매핑 참고표

| 표준 (trajectory_io) | 3DOF (missile_6dof.py) | 6DOF v2 | 비고 |
|---------------------|------------------------|---------|------|
| `position_z` | `altitude` | `position_z` | ⚠️ 주의! |
| `u, v, w` | ❌ 없음 | `u, v, w` | Body Frame 속도 |
| `V` | `V` | `V` | 속도 크기 |
| `gamma` | `gamma` | 계산 필요 | 비행경로각 |
| `chi` | `psi` | `psi` | 방위각 |
| `phi, theta, psi` | 부분적 | 완전 지원 | Euler 각도 |
| `p, q, r` | 부분적 | 완전 지원 | 각속도 |

---

**작성자**: Cascade AI  
**날짜**: 2025-12-19  
**목적**: 조원 지적사항 반영 - NPZ 포맷 통일 솔루션
