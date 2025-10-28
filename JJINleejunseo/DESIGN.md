# 🏗️ 시스템 아키텍처 문서

## 목차
- [시스템 아키텍처](#시스템-아키텍처)
- [주요 클래스/함수 설명](#주요-클래스함수-설명)
- [데이터 흐름](#데이터-흐름)
- [데이터 구조](#데이터-구조)

---

# 🎯 시스템 아키텍처

## 1. 전체 시스템 구조

### 시스템 개요
6DOF 미사일 궤적 시뮬레이션 시스템은 물리 기반 시뮬레이션 엔진으로, 미사일의 병진 운동과 회전 운동을 동시에 계산합니다.

### 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                    User Interface Layer                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │CLI Input │  │  Config  │  │Script API│                  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                  │
└───────┼─────────────┼─────────────┼────────────────────────┘
        └─────────────┴─────────────┘
                      ▼
┌─────────────────────────────────────────────────────────────┐
│            Application Layer (main_fixed.py)                 │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         MissileSimulation6DOF (Controller)             │ │
│  │  - initialize_simulation()                             │ │
│  │  - run_simulation()           (모드 2: 상세 분석)      │ │
│  │  - run_simulation_realtime()  (모드 1: 실시간 3D)      │ │
│  │  - dynamics_*_6dof()          (4단계 동역학)          │ │
│  │  - plot_results_6dof()        (12-패널 시각화)        │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
        │                 │                 │
        ▼                 ▼                 ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   Physics    │  │ Aerodynamics │  │  Propulsion  │
│   Engine     │  │    Module    │  │    Module    │
│              │  │              │  │              │
│ - ODE solver │  │ - Drag/Lift  │  │ - Thrust     │
│ - Kinematics │  │ - Moments    │  │ - Fuel burn  │
└──────────────┘  └──────────────┘  └──────────────┘
        │                 │                 │
        └─────────────────┴─────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                      Data Layer                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │ Config   │  │  State   │  │ Results  │                  │
│  │ (6DOF)   │  │ Vector   │  │   Dict   │                  │
│  └──────────┘  └──────────┘  └──────────┘                  │
└─────────────────────────────────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  Visualization Layer                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │Matplotlib│  │PNG Files │  │Interactive│                 │
│  └──────────┘  └──────────┘  └──────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 각 계층/모듈의 역할

### User Interface Layer
- **역할**: 사용자 입력 수신 및 시뮬레이션 시작
- **구성**: CLI, Config Files, Script API

### Application Layer
- **역할**: 시뮬레이션 전체 흐름 제어
- **핵심 클래스**: `MissileSimulation6DOF`

### Physics Engine
- **역할**: 운동 방정식 수치 해석
- **도구**: scipy.integrate.solve_ivp

### Aerodynamics Module
- **역할**: 공력 및 모멘트 계산
- **출력**: 힘/모멘트 벡터

### Data Layer
- **역할**: 설정 및 결과 데이터 저장

### Visualization Layer
- **역할**: 결과 그래프 생성

---

## 3. 모듈 간 통신 방식

### 동기 호출
```python
drag, lift = self.calculate_aerodynamic_forces(V, alpha, rho)
```

### 콜백 패턴
```python
sol = solve_ivp(self.equations_of_motion, [0, sim_time], state0)
```

### 데이터 공유
```python
import config_6dof as cfg
thrust = cfg.MISSILE_PARAMS[self.missile_type]["thrust"]
```

---

# 🔧 주요 클래스/함수 설명

## 클래스: MissileSimulation6DOF

### 목적
6DOF 미사일 궤적 시뮬레이션의 메인 컨트롤러

### 주요 속성
```python
missile_type: str           # 미사일 타입
results: Dict[str, List]    # 시계열 결과
states: List[np.ndarray]    # 상태 벡터 히스토리
alpha_list: List[float]     # 받음각 히스토리
roll_list: List[float]      # 롤각 히스토리
```

### 주요 메서드

#### `__init__(missile_type, apply_errors)`
- **목적**: 시뮬레이션 객체 초기화
- **Parameters**:
  - `missile_type` (str): 미사일 타입
  - `apply_errors` (bool): 오차 적용 여부
- **동작**: 결과 딕셔너리 초기화, 파라미터 로드

#### `initialize_simulation(launch_angle_deg, azimuth_deg, sim_time)`
- **목적**: 초기 조건 설정
- **Parameters**:
  - `launch_angle_deg` (float): 발사각 (0-90도)
  - `azimuth_deg` (float): 방위각 (0-360도)
  - `sim_time` (float): 시뮬레이션 시간 (초)
- **동작**: 초기 상태 벡터 생성, 각도 변환

#### `run_simulation()` (모드 2)
- **목적**: 상세 분석용 시뮬레이션 실행
- **동작**: 4단계 순차 ODE 적분, 결과 저장
- **예시**:
```python
sim = MissileSimulation6DOF()
sim.initialize_simulation(launch_angle_deg=45, sim_time=1500)
results = sim.run_simulation()
sim.plot_results_6dof()
```

#### `run_simulation_realtime()` (모드 1) 🆕
- **목적**: 실시간 3D 시각화와 함께 시뮬레이션 실행
- **동작**: matplotlib 대화형 모드로 궤적 애니메이션
- **특징**: 
  - plt.ion() 활용한 실시간 업데이트
  - 4단계 비행 과정 실시간 표시
  - 78km 멈춤 버그 수정 (sim_time=1500초)
- **예시**:
```python
sim = MissileSimulation6DOF()
sim.run_simulation_realtime()  # 자동으로 초기화 및 실행
```

#### 동역학 함수들 (4단계 비행 프로그램)

**`dynamics_vertical_6dof(t, state)`** - 수직 상승 단계
- 발사 후 수직 상승 (0~5초)
- 피치각 일정 유지

**`dynamics_pitch_6dof(t, state)`** - 피치 전환 단계
- 목표 피치각으로 자세 변경 (5~15초)
- 양력 및 제어 모멘트 적용

**`dynamics_constant_6dof(t, state)`** - 등자세 비행 단계
- 일정 자세 유지하며 추력 비행 (15~60초)

**`dynamics_midcourse_6dof(t, state)`** - 중간단계 비행
- 추력 종료 후 관성 비행 (~1500초)
- 대기권 재진입 효과 포함

**공통 계산 순서**:
  1. 대기 특성 계산 (밀도, 온도, 음속)
  2. 공력 계산 (항력, 양력, 모멘트)
  3. 추력 계산 (연소 시간 고려)
  4. 가속도 및 각가속도 계산
  5. 상태 미분 반환

#### `plot_results_6dof()`
- **목적**: 12-패널 통합 결과 시각화
- **출력**: PNG 파일 1개 (고해상도 300 DPI)
  - `results_6dof/6dof_results_YYYYMMDD_HHMMSS.png`
- **패널 구성**:
  1. 3D 궤적
  2. 속도 vs 시간
  3. 고도 vs 시간
  4. 비행경로각 vs 시간
  5. 롤각 (φ) vs 시간
  6. 피치각 (θ) vs 시간
  7. 요각 (ψ) vs 시간
  8. 롤 각속도 (p) vs 시간
  9. 피치 각속도 (q) vs 시간
  10. 요 각속도 (r) vs 시간
  11. 질량 vs 시간
  12. 사거리 vs 고도

#### `calculate_aerodynamic_moments(state, q_dynamic)` 🆕
- **목적**: 공력 모멘트 계산 (안정화 버전)
- **Parameters**:
  - `state`: 14차원 상태 벡터
  - `q_dynamic`: 동압 (0.5·ρ·V²)
- **Returns**: 
  - `L_aero`: 롤 모멘트 (N·m)
  - `M_aero`: 피치 모멘트 (N·m)
  - `N_aero`: 요 모멘트 (N·m)
  - `alpha`: 받음각 (rad)
  - `beta`: 측면 받음각 (rad)
- **안정화 기법**:
  - 받음각 제한: ±45도
  - 각속도 정규화: ±10 rad/s
  - 모멘트 제한: ±1e6 N·m
  - 스무딩 적용

#### `calculate_euler_rates(phi, theta, p, q, r)` 🆕
- **목적**: 오일러 각도 변화율 계산 (짐벌락 방지)
- **Parameters**:
  - `phi, theta`: 현재 오일러 각도 (rad)
  - `p, q, r`: 각속도 (rad/s)
- **Returns**:
  - `dphi_dt, dtheta_dt, dpsi_euler_dt`: 각도 변화율 (rad/s)
- **안정화 기법**:
  - cos(theta) ≈ 0 예외 처리
  - 각도 변화율 제한: ±5 rad/s

---

## 클래스: ConfigManager (config_6dof.py)

### 목적
미사일 파라미터 및 시뮬레이션 설정 관리

### 주요 데이터 구조

#### MISSILE_PARAMS
```python
MISSILE_PARAMS = {
    "SCUD-B": {
        "mass_initial": 6370,      # kg
        "mass_fuel": 3800,         # kg
        "diameter": 0.88,          # m
        "thrust": 250000,          # N
        "inertia_xx": 58729.12,    # kg·m²
        "cl_alpha": 0.5,           # 양력 기울기
        "cm_alpha": -0.15,         # 피칭 모멘트
        # ...
    }
}
```

### 주요 상수
- `SIM_TIME`: 시뮬레이션 시간 (300초)
- `DT_MAX`: 최대 시간 간격 (0.5초)
- `G`: 중력 가속도 (9.81 m/s²)

---

## 함수: get_atmosphere_properties(altitude)

### 목적
고도에 따른 대기 특성 계산 (ISA 모델)

### Parameters
- `altitude` (float): 고도 (m)

### Returns
- `rho` (float): 공기 밀도 (kg/m³)
- `pressure` (float): 압력 (Pa)
- `temperature` (float): 온도 (K)
- `speed_of_sound` (float): 음속 (m/s)

### 구현
```python
def get_atmosphere_properties(h):
    T0, P0, rho0 = 288.15, 101325, 1.225
    
    if h < 11000:  # 대류권
        T = T0 - 0.0065 * h
        P = P0 * (T / T0) ** 5.2561
    elif h < 25000:  # 성층권
        T = 216.65
        P = 22632 * np.exp(-0.0001577 * (h - 11000))
    else:  # 상층
        T = 216.65
        P = 2488 * np.exp(-0.0001262 * (h - 25000))
    
    rho = P / (287.05 * T)
    a = np.sqrt(1.4 * 287.05 * T)
    return rho, P, T, a
```

---

# 📊 데이터 흐름

## 1. 입력 → 처리 → 출력

### 입력 단계
```
사용자 입력
  ↓
초기 조건 (발사각, 방위각, 시간)
  ↓
config_6dof.py (미사일 파라미터)
```

### 처리 단계
```
초기화
  ↓
상태 벡터 생성 (14차원)
  ↓
ODE 적분 루프
  ├─ 대기 특성 계산
  ├─ 공력 계산
  ├─ 추력 계산
  ├─ 운동 방정식 해석
  └─ 상태 업데이트
  ↓
결과 수집
```

### 출력 단계
```
results 딕셔너리
  ↓
데이터 후처리
  ↓
시각화 (5개 그래프)
  ↓
PNG 파일 저장
```

---

## 2. 주요 데이터 구조

### 상태 벡터 (State Vector)
**타입**: `numpy.ndarray` (14,)

```python
state = [
    V,      # 0: 속도 (m/s)
    gamma,  # 1: 비행경로각 (rad)
    psi,    # 2: 방위각 (rad)
    x,      # 3: 수평 거리 (m)
    y,      # 4: 횡방향 거리 (m)
    h,      # 5: 고도 (m)
    M,      # 6: 질량 (kg)
    fuel,   # 7: 연료 질량 (kg)
    phi,    # 8: 롤각 (rad)
    theta,  # 9: 피치각 (rad)
    psi_e,  # 10: 요각 (rad)
    p,      # 11: 롤 각속도 (rad/s)
    q,      # 12: 피치 각속도 (rad/s)
    r       # 13: 요 각속도 (rad/s)
]
```

### Results 딕셔너리
**타입**: `Dict[str, List[float]]`

```python
results = {
    'time': [],       # 시간 (s)
    'velocity': [],   # 속도 (m/s)
    'gamma': [],      # 비행경로각 (rad)
    'psi': [],        # 방위각 (rad)
    'x': [],          # 수평 거리 (m)
    'y': [],          # 횡방향 거리 (m)
    'h': [],          # 고도 (m)
    'mass': [],       # 질량 (kg)
    'phi': [],        # 롤각 (rad)
    'theta': [],      # 피치각 (rad)
    'psi_euler': [],  # 요각 (rad)
    'p': [],          # 롤 각속도 (rad/s)
    'q': [],          # 피치 각속도 (rad/s)
    'r': [],          # 요 각속도 (rad/s)
    'alpha': [],      # 받음각 (rad)
    'beta': [],       # 측면 받음각 (rad)
    'CD': [],         # 항력계수
    'mach': [],       # 마하수
    'phase': []       # 비행 단계
}
```

### 미사일 파라미터 딕셔너리
**타입**: `Dict[str, Dict[str, float]]`

```python
MISSILE_PARAMS = {
    "SCUD-B": {
        # 기본 물리량
        "mass_initial": float,    # 초기 질량 (kg)
        "mass_fuel": float,       # 연료 질량 (kg)
        "diameter": float,        # 직경 (m)
        "length": float,          # 길이 (m)
        
        # 추진
        "thrust": float,          # 추력 (N)
        "isp": float,             # 비추력 (s)
        "burn_time": float,       # 연소 시간 (s)
        
        # 관성
        "inertia_xx": float,      # 피치/요 관성 (kg·m²)
        "inertia_yy": float,
        "inertia_zz": float,      # 롤 관성 (kg·m²)
        
        # 공력 계수
        "cd0": float,             # 기본 항력계수
        "cl_alpha": float,        # 양력 기울기
        "cm_alpha": float,        # 피칭 모멘트
        "cn_beta": float,         # 요잉 모멘트
        "cl_p": float,            # 롤 댐핑
        "cm_q": float,            # 피치 댐핑
        "cn_r": float             # 요 댐핑
    }
}
```

---

## 3. 데이터 흐름 상세

### 시뮬레이션 루프
```
t = 0
while t < sim_time:
    1. state 분해
       V, gamma, psi, x, y, h, M, fuel, phi, theta, psi_e, p, q, r = state
    
    2. 대기 계산
       rho, P, T, a = get_atmosphere(h)
    
    3. 공력 계산
       alpha = theta - gamma
       beta = psi_e - psi
       drag, lift = calculate_drag_lift(V, alpha, rho)
       L, M, N = calculate_moments(alpha, beta, p, q, r, rho, V)
    
    4. 추력 계산
       if t < burn_time:
           thrust = THRUST
           fuel_rate = thrust / (isp * g)
       else:
           thrust = 0
           fuel_rate = 0
    
    5. 힘의 합력
       F_x = thrust * cos(theta) - drag
       F_y = lift * sin(phi)
       F_z = -M * g + lift * cos(phi)
    
    6. 가속도
       dV_dt = F_x / M
       dgamma_dt = (F_z / (M * V)) - (g * cos(gamma) / V)
       dpsi_dt = F_y / (M * V * cos(gamma))
    
    7. 각가속도
       dp_dt = L / Ixx
       dq_dt = M / Iyy
       dr_dt = N / Izz
    
    8. 상태 업데이트
       state += dstate_dt * dt
       t += dt
```

---

# 📁 데이터베이스 설계

**참고**: 현재 시스템은 데이터베이스를 사용하지 않으며, 모든 데이터는 메모리와 파일로 관리됩니다.

## 향후 확장 시 제안 구조

### 테이블 1: Missiles
```sql
CREATE TABLE Missiles (
    missile_id INTEGER PRIMARY KEY,
    name VARCHAR(50) NOT NULL,
    mass_initial FLOAT,
    mass_fuel FLOAT,
    diameter FLOAT,
    length FLOAT,
    thrust FLOAT,
    isp FLOAT,
    burn_time FLOAT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### 테이블 2: Simulations
```sql
CREATE TABLE Simulations (
    sim_id INTEGER PRIMARY KEY,
    missile_id INTEGER,
    launch_angle FLOAT,
    azimuth FLOAT,
    sim_time FLOAT,
    final_range FLOAT,
    max_altitude FLOAT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (missile_id) REFERENCES Missiles(missile_id)
);
```

### 테이블 3: TrajectoryData
```sql
CREATE TABLE TrajectoryData (
    data_id INTEGER PRIMARY KEY,
    sim_id INTEGER,
    time FLOAT,
    velocity FLOAT,
    altitude FLOAT,
    x_position FLOAT,
    y_position FLOAT,
    roll_angle FLOAT,
    pitch_angle FLOAT,
    yaw_angle FLOAT,
    FOREIGN KEY (sim_id) REFERENCES Simulations(sim_id)
);
```

### 테이블 관계
```
Missiles (1) ──< (N) Simulations (1) ──< (N) TrajectoryData
```

### 샘플 데이터
```sql
-- Missiles
INSERT INTO Missiles VALUES 
(1, 'SCUD-B', 6370, 3800, 0.88, 11.25, 250000, 230, 60, NOW());

-- Simulations
INSERT INTO Simulations VALUES
(1, 1, 45.0, 90.0, 300.0, 332290, 735680, NOW());

-- TrajectoryData
INSERT INTO TrajectoryData VALUES
(1, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 90.0),
(2, 1, 1.0, 150.5, 145.2, 106.3, 0.0, 0.0, 44.8, 90.0);
```

---

## 부록: 수식 정리

### 운동 방정식
```
병진 운동:
  dV/dt = (T - D) / m - g·sin(γ)
  dγ/dt = (L·cos(φ) - m·g·cos(γ)) / (m·V)
  dψ/dt = L·sin(φ) / (m·V·cos(γ))

회전 운동:
  dp/dt = L_aero / I_xx
  dq/dt = M_aero / I_yy
  dr/dt = N_aero / I_zz
```

### 공력 계산
```
동압: q = 0.5·ρ·V²
항력: D = q·S·CD
양력: L = q·S·CL
모멘트: M = q·S·L_ref·Cm
```

---

**문서 버전**: 1.1.0  
**최종 수정일**: 2025-10-28 (main_fixed.py 반영)  
**작성자**: 이준서

## 주요 업데이트 (v1.1.0)
- ✅ 실시간 3D 시각화 모드 추가 (`run_simulation_realtime()`)
- ✅ 78km 멈춤 버그 수정 (sim_time 기본값 1500초)
- ✅ 공력 모멘트 계산 안정화 (과도한 값 제한)
- ✅ 오일러 각도 변환 안정화 (짐벌락 방지 강화)
- ✅ 4단계 동역학 함수 분리 및 최적화
