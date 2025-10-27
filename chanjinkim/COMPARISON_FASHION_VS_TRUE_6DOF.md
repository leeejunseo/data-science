# 🔍 패션 6DOF vs 진짜 6DOF 비교 분석

## 📊 핵심 차이점 요약

| 측면 | 패션 6DOF (`main_6dof.py`) | 진짜 6DOF (`main_true_6dof.py`) |
|------|---------------------------|--------------------------------|
| **상태 공간** | 14차원 (혼합) | 12차원 (일관성) |
| **좌표계** | 혼용 (γ, ψ + φ, θ, ψ) | 명확 구분 (Body/Inertial) |
| **병진-회전 커플링** | ❌ 없음 | ✅ 완전 커플링 |
| **좌표 변환** | ❌ 없음 | ✅ DCM 사용 |
| **물리적 정확성** | ⚠️ 낮음 | ✅ 높음 |
| **계산 복잡도** | 낮음 | 중간 |

---

## 🔬 상세 비교

### 1. 상태 벡터

#### 패션 6DOF (14차원)
```python
state = [V, gamma, psi,           # 3DOF 병진 변수
         x, y, h,                  # 위치
         phi, theta, psi_euler,    # 오일러각 (중복!)
         p, q, r,                  # 각속도
         M, fuel]                  # 질량
```

**문제점**:
- `psi`(방위각)와 `psi_euler`(요각) 중복
- `gamma`(비행경로각)와 `theta`(피치각) 혼용
- 좌표계 불명확

#### 진짜 6DOF (12차원)
```python
state = [u, v, w,      # Body Frame 속도
         p, q, r,      # Body Frame 각속도
         phi, theta, psi,  # 오일러각
         X, Y, Z]      # Inertial Frame 위치
```

**장점**:
- 모든 변수가 명확한 좌표계에 속함
- 중복 없음
- 표준 6DOF 표현

---

### 2. 병진 운동 방정식

#### 패션 6DOF
```python
def dynamics_vertical_6dof(self, t, state):
    V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q, r, M_t, fuel = state
    
    # 병진 운동 (각속도 무관!)
    dV_dt = (T - D) / M_safe - g * np.sin(gamma)
    dgamma_dt = 0.0  # 수직상승 단계
    dpsi_dt = 0.0
    
    # 위치 (gamma, psi 사용)
    dx_dt = V * np.cos(gamma) * np.cos(psi)
    dy_dt = V * np.cos(gamma) * np.sin(psi)
    dh_dt = V * np.sin(gamma)
```

**문제점**:
1. 각속도(p, q, r)가 병진에 **전혀 영향 없음**
2. `gamma`, `psi` 사용 (Body Frame과 무관)
3. 회전 자세(φ, θ, ψ)가 궤적에 무영향

#### 진짜 6DOF
```python
def dynamics_equations(self, t, state):
    u, v, w, p, q, r, phi, theta, psi, X, Y, Z = state
    
    # 병진 운동 (Body Frame)
    # 핵심: 각속도가 직접 영향!
    u_dot = r*v - q*w + F_x / m  # ← 각속도 항!
    v_dot = p*w - r*u + F_y / m  # ← 각속도 항!
    w_dot = q*u - p*v + F_z / m  # ← 각속도 항!
    
    # 위치 (좌표 변환)
    v_body = [u, v, w]
    v_inertial = DCM @ v_body  # Body → Inertial 변환
    
    X_dot = v_inertial[0]
    Y_dot = v_inertial[1]
    Z_dot = v_inertial[2]
```

**핵심 차이**:
- 각속도(p, q, r)가 **직접** 병진 속도(u, v, w)에 영향
- DCM을 통한 좌표 변환
- 자세가 궤적에 직접 영향

---

### 3. 회전 운동 방정식

#### 패션 6DOF
```python
# 공력 모멘트 계산
L_aero, M_aero, N_aero, alpha, beta = self.calculate_aerodynamic_moments(state, q)

# 회전 운동 (오일러 방정식)
dp_dt = (L_aero + (self.inertia_yy - self.inertia_zz) * q * r) / self.inertia_xx
dq_dt = (M_aero + (self.inertia_zz - self.inertia_xx) * p * r) / self.inertia_yy
dr_dt = (N_aero + (self.inertia_xx - self.inertia_yy) * p * q) / self.inertia_zz
```

**형식상 맞지만**:
- 계산된 공력 모멘트가 병진 운동에 **영향 없음**
- 각속도가 궤적과 **독립적**
- 회전 에너지와 병진 에너지 **분리**

#### 진짜 6DOF
```python
# 공력 모멘트 (각속도 댐핑 포함)
L_aero, M_aero, N_aero = self.aero.calculate_aerodynamic_moments(
    rho, V_mag, alpha, beta, p, q, r  # ← 각속도 영향!
)

# 회전 운동 (동일한 방정식)
dp_dt = (L_aero + (Iyy - Izz) * q * r) / Ixx
dq_dt = (M_aero + (Izz - Ixx) * p * r) / Iyy
dr_dt = (N_aero + (Ixx - Iyy) * p * q) / Izz
```

**차이점**:
- 공력 모멘트가 각속도(p, q, r)에 의존 → **댐핑 효과**
- 각속도가 병진 방정식에 **직접 영향**
- 에너지 **완전 커플링**

---

### 4. 공력 모델

#### 패션 6DOF
```python
def calculate_aerodynamic_moments(self, state, q_dynamic):
    V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q, r, M_t, fuel = state
    
    # 받음각 (theta와 gamma 차이)
    alpha = np.clip(theta - gamma, -np.pi/4, np.pi/4)
    beta = 0.0  # 항상 0
    
    # 공력 모멘트 계수
    Cl = self.cl_alpha * alpha + self.cl_p * p * (self.length / (2 * V))
    Cm = self.cm_alpha * alpha + self.cm_q * q * (self.length / (2 * V))
    Cn = self.cn_beta * beta + self.cn_r * r * (self.length / (2 * V))
```

**문제점**:
- `theta - gamma`는 **임의 정의**
- `beta = 0` 고정 (측면 받음각 무시)
- Body Frame 속도(v) 미사용

#### 진짜 6DOF
```python
def calculate_aerodynamic_forces(self, rho, V_mag, u, v, w, mach):
    # 받음각 (Body Frame 속도로부터 직접 계산)
    alpha = np.arctan2(w, u)  # 진짜 받음각!
    beta = np.arcsin(v / V_mag)  # 진짜 측면 받음각!
    
    # 공력 계수 (받음각 의존)
    C_D = self.get_cd_base(mach) + 0.1 * alpha**2
    C_L = self.cl_alpha * alpha
    C_Y = self.cy_beta * beta  # 측력 (v 성분)
    
    # Body Frame 힘
    F_aero_x = -D
    F_aero_y = Y
    F_aero_z = -L
```

**장점**:
- Body Frame 속도(u, v, w)로부터 **직접 계산**
- 측면 받음각 반영 (측력)
- 물리적으로 정확

---

### 5. 중력 처리

#### 패션 6DOF
```python
# 중력 (gamma 기반)
dV_dt = (T - D) / M_safe - g * np.sin(gamma)
```

**문제점**:
- 중력이 `gamma`(비행경로각)에만 의존
- 자세(φ, θ, ψ)와 무관
- Body Frame 미고려

#### 진짜 6DOF
```python
# 중력 (Inertial → Body 변환)
g_inertial = np.array([0, 0, -g])
g_body = DCM.T @ g_inertial  # Inertial → Body

F_gravity_x = m * g_body[0]
F_gravity_y = m * g_body[1]
F_gravity_z = m * g_body[2]

# 병진 방정식에 추가
u_dot = r*v - q*w + (F_thrust_x + F_aero_x + F_gravity_x) / m
```

**장점**:
- 자세(φ, θ, ψ)에 따라 중력 방향 변함
- Body Frame에서 일관되게 처리
- 물리적으로 정확

---

## 🎯 결과 비교 (예상)

### 테스트 케이스: SCUD-B, 45° 발사

| 항목 | 패션 6DOF | 진짜 6DOF | 차이 |
|------|-----------|----------|------|
| **최종 거리** | ~280 km | ~275 km | -2% |
| **최대 고도** | ~85 km | ~83 km | -2% |
| **비행 시간** | ~350 s | ~345 s | -1% |
| **회전 효과** | 없음 | 있음 | ✓ |
| **자세 안정성** | 분석 불가 | 분석 가능 | ✓ |

**주요 차이**:
1. **대부분의 경우 유사한 궤적** (병진 운동이 지배적)
2. **회전이 중요한 경우 차이 발생**:
   - 낮은 각도 발사 (< 15°)
   - 고속 회전 (spin-stabilized)
   - 바람/교란 존재 시
3. **진짜 6DOF만 가능한 분석**:
   - 자세 안정성 평가
   - 각운동량 분석
   - 회전 에너지 분석

---

## 💡 언제 어떤 것을 사용할까?

### 패션 6DOF 사용 가능 (main_6dof.py)
✓ 빠른 프로토타이핑  
✓ 단순 궤적 예측  
✓ 회전 효과 무시 가능  
✓ 계산 자원 제한  

**단점**: 물리적 정확성 낮음

### 진짜 6DOF 필수 (main_true_6dof.py)
✓ **정확한 시뮬레이션**  
✓ **자세 안정성 분석**  
✓ **회전 효과 중요**  
✓ **연구/논문용**  
✓ **제어 시스템 설계**  

**단점**: 계산 비용 2-3배

---

## 🔧 마이그레이션 가이드

### 패션 6DOF → 진짜 6DOF

#### 1. 미사일 설정 변환

```python
# 기존 (패션 6DOF)
from config_6dof import MISSILE_TYPES
missile_info = MISSILE_TYPES['SCUD-B']

# 변환 (진짜 6DOF)
missile_config = {
    'name': missile_info['name'],
    'launch_weight': missile_info['launch_weight'],
    'propellant_mass': missile_info['propellant_mass'],
    'diameter': missile_info['diameter'],
    'length': missile_info['length'],
    'reference_area': missile_info['reference_area'],
    'wingspan': missile_info['diameter'] * 2,  # 추가 필요
    'inertia_xx': missile_info['inertia_xx'],
    'inertia_yy': missile_info['inertia_yy'],
    'inertia_zz': missile_info['inertia_zz'],
    'isp_sea': missile_info['isp_sea'],
    'isp_vac': missile_info.get('isp_vac', missile_info['isp_sea'] * 1.1),
    'burn_time': missile_info['burn_time'],
    'cd_base': 0.25,
    'cl_alpha': 3.5,
    'cy_beta': -0.5,
    'cm_alpha': missile_info['cm_alpha'],
    'cn_beta': missile_info['cn_beta'],
    'cl_p': missile_info['cl_p'],
    'cm_q': missile_info['cm_q'],
    'cn_r': missile_info['cn_r']
}
```

#### 2. 시뮬레이션 실행

```python
# 기존
from main_6dof import MissileSimulation6DOF
sim = MissileSimulation6DOF(missile_type="SCUD-B")
sim.initialize_simulation(launch_angle_deg=45)
results = sim.run_simulation()

# 변환
from main_true_6dof import MissileSimulationTrue6DOF, get_scud_b_config
config = get_scud_b_config()
sim = MissileSimulationTrue6DOF(config)
results = sim.run_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=600)
```

#### 3. 결과 접근

```python
# 기존 (패션 6DOF)
velocity = results['velocity']  # 속도 크기
gamma = results['gamma']  # 비행경로각
altitude = results['h']

# 변환 (진짜 6DOF)
u = results['u']  # Body Frame X 속도
v = results['v']  # Body Frame Y 속도
w = results['w']  # Body Frame Z 속도
V_mag = results['V_mag']  # 속도 크기
altitude = results['Z']

# 비행경로각 계산 (필요 시)
from coordinate_transforms import CoordinateTransform
v_body = np.array([u, v, w])
v_inertial = CoordinateTransform.body_to_inertial(v_body, phi, theta, psi)
gamma = np.arctan2(v_inertial[2], np.sqrt(v_inertial[0]**2 + v_inertial[1]**2))
```

---

## 📈 성능 비교

### 계산 시간 (예상)

| 시뮬레이션 시간 | 패션 6DOF | 진짜 6DOF | 비율 |
|----------------|-----------|----------|------|
| **600초 궤적** | ~1-2초 | ~2-5초 | 2-3배 |
| **Monte Carlo 100회** | ~2-3분 | ~5-8분 | 2-3배 |

**병목**: 
- 좌표 변환 (DCM 계산)
- 공력 계산 복잡도
- ODE solver 스텝 수

**최적화 방법**:
1. DCM 캐싱
2. 벡터화
3. JIT 컴파일 (Numba)
4. GPU 가속

---

## 🏆 결론

### 패션 6DOF의 가치
- 교육용/프로토타입에 유용
- 3DOF에서 6DOF로 가는 **중간 단계**
- 빠른 결과 확인

### 진짜 6DOF의 필요성
- **물리적 정확성** 필수 시
- **자세 안정성** 분석 필요 시
- **회전 효과** 중요 시
- **연구/논문** 수준

### 권장 사항
1. **프로토타입**: 패션 6DOF로 시작
2. **검증/분석**: 진짜 6DOF로 전환
3. **최종 결과**: 진짜 6DOF 사용

---

**작성일**: 2025-10-27  
**버전**: 1.0  
**다음 AI의 평가**: "패션 6DOF" ✓ 정확함!
