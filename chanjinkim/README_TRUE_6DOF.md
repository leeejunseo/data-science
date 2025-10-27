# 🚀 진짜 6DOF 미사일 시뮬레이션

## 📋 개요

**물리적으로 정확한** 6자유도(6DOF) 미사일 궤적 시뮬레이션입니다.

### ✅ "진짜 6DOF"란?

- ✓ **Body Frame 기반** 동역학
- ✓ **병진-회전 완전 커플링**
- ✓ **좌표계 변환** (DCM/쿼터니언)
- ✓ **자세가 궤적에 직접 영향**
- ✓ **각속도가 공력 모멘트에 영향** (댐핑)

---

## 📁 파일 구조

```
leejunseo/
├── TRUE_6DOF_DESIGN.md           # 설계 문서
├── README_TRUE_6DOF.md            # 이 파일
│
├── coordinate_transforms.py       # 좌표계 변환 (DCM, 쿼터니언)
├── aerodynamics_true_6dof.py      # 진짜 6DOF 공력 모델
├── dynamics_true_6dof.py          # 진짜 6DOF 동역학
├── main_true_6dof.py              # 메인 시뮬레이션
│
└── config_6dof.py                 # 미사일 설정 (기존 호환)
```

---

## 🚀 사용 방법

### 1. 기본 실행

```bash
python main_true_6dof.py
```

### 2. 파이썬 스크립트에서 사용

```python
from main_true_6dof import MissileSimulationTrue6DOF, get_scud_b_config

# 미사일 설정
config = get_scud_b_config()

# 시뮬레이션 생성
sim = MissileSimulationTrue6DOF(config)

# 실행 (45도 발사, 동쪽 방향)
results = sim.run_simulation(
    launch_angle_deg=45,
    azimuth_deg=90,
    sim_time=600
)

# 시각화
sim.plot_results()
```

### 3. 커스텀 미사일 설정

```python
custom_config = {
    'name': 'Custom Missile',
    'launch_weight': 5000,       # kg
    'propellant_mass': 4000,     # kg
    'diameter': 0.8,             # m
    'length': 10.0,              # m
    'reference_area': 0.5,       # m²
    'wingspan': 1.6,             # m
    'inertia_xx': 50000,         # kg·m²
    'inertia_yy': 50000,         # kg·m²
    'inertia_zz': 1000,          # kg·m²
    'isp_sea': 240,              # s
    'isp_vac': 270,              # s
    'burn_time': 60,             # s
    'cd_base': 0.3,
    'cl_alpha': 3.0,
    'cy_beta': -0.4,
    'cm_alpha': -0.2,
    'cn_beta': 0.15,
    'cl_p': -0.6,
    'cm_q': -1.0,
    'cn_r': -1.0
}

sim = MissileSimulationTrue6DOF(custom_config)
```

---

## 📊 출력 결과

### 상태 변수 (12개)

**Body Frame 속도** (m/s):
- `u`: X축 (전방) 속도
- `v`: Y축 (우측) 속도
- `w`: Z축 (상방) 속도

**Body Frame 각속도** (rad/s):
- `p`: 롤 각속도
- `q`: 피치 각속도
- `r`: 요 각속도

**오일러각** (rad):
- `phi` (φ): 롤각
- `theta` (θ): 피치각
- `psi` (ψ): 요각

**Inertial Frame 위치** (m):
- `X`: 북쪽 위치
- `Y`: 동쪽 위치
- `Z`: 고도

### 추가 계산 변수

- `V_mag`: 속도 크기 (m/s)
- `alpha`: 받음각 (rad)
- `beta`: 측면 받음각 (rad)
- `mach`: 마하수
- `range_km`: 수평 거리 (km)

---

## 🔬 핵심 기능

### 1. Body Frame 동역학

```python
# 병진 운동 (각속도가 직접 영향!)
u_dot = r*v - q*w + F_x / m
v_dot = p*w - r*u + F_y / m
w_dot = q*u - p*v + F_z / m
```

### 2. 좌표계 변환

```python
from coordinate_transforms import CoordinateTransform

# Body → Inertial
v_inertial = CoordinateTransform.body_to_inertial(v_body, phi, theta, psi)

# Inertial → Body
v_body = CoordinateTransform.inertial_to_body(v_inertial, phi, theta, psi)
```

### 3. 자세 의존 공력

```python
# 받음각 계산 (Body Frame 속도로부터)
alpha = arctan2(w, u)
beta = asin(v / V_mag)

# 받음각에 따른 공력 계수
C_L = C_L_alpha * alpha
C_D = C_D_base + K * alpha²
```

### 4. 각속도 댐핑

```python
# 각속도가 공력 모멘트에 영향
C_m = C_m_alpha * alpha + C_m_q * (q * length / (2*V))
```

---

## ⚖️ 패션 6DOF vs 진짜 6DOF

| 항목 | 패션 6DOF | 진짜 6DOF |
|------|-----------|----------|
| **좌표계** | 혼용 (일관성 없음) | Body/Inertial 명확히 구분 |
| **병진 방정식** | `dV/dt = F/m - g*sin(γ)` | `du/dt = rv - qw + F_x/m + g_x` |
| **좌표 변환** | ❌ 없음 | ✅ DCM/쿼터니언 |
| **회전-병진 커플링** | ❌ 독립적 | ✅ 완전 커플링 |
| **자세 영향** | ❌ 궤적에 무영향 | ✅ 직접 영향 |
| **공력 모멘트** | 계산만 함 | ✅ 실제 사용 |
| **각속도 댐핑** | ❌ 없음 | ✅ 있음 |
| **물리적 타당성** | ⚠️ 낮음 | ✅ 높음 |

---

## 🧪 검증 방법

### 1. 단위 테스트

```bash
# 좌표계 변환 테스트
python coordinate_transforms.py

# 공력 모델 테스트
python aerodynamics_true_6dof.py

# 동역학 테스트
python dynamics_true_6dof.py
```

### 2. 물리적 검증

#### DCM 직교성
```python
from coordinate_transforms import CoordinateTransform

DCM = CoordinateTransform.euler_to_dcm(phi, theta, psi)
is_valid, errors = CoordinateTransform.verify_dcm(DCM)
# 직교성: DCM * DCM^T = I
# 행렬식: det(DCM) = 1
```

#### 에너지 보존 (테스트용)
```python
# 추력 에너지 = 운동 에너지 + 위치 에너지 + 손실
KE = 0.5 * m * V²
PE = m * g * h
```

### 3. 극한 케이스

- **수직 발사**: θ = 90° (짐벌락 테스트)
- **저각 발사**: θ < 15° (수치 안정성)
- **고속 회전**: p, q, r 큼 (자이로 효과)

---

## 🎯 예제

### 예제 1: SCUD-B 45° 발사

```python
from main_true_6dof import MissileSimulationTrue6DOF, get_scud_b_config

config = get_scud_b_config()
sim = MissileSimulationTrue6DOF(config)

results = sim.run_simulation(
    launch_angle_deg=45,
    azimuth_deg=90,
    sim_time=600
)

sim.plot_results()
```

**예상 결과**:
- 최대 고도: ~80-100 km
- 최대 거리: ~250-300 km
- 최대 속도: ~1500-1800 m/s

### 예제 2: 다양한 발사각 비교

```python
import matplotlib.pyplot as plt

angles = [15, 30, 45, 60, 75]
results_list = []

config = get_scud_b_config()

for angle in angles:
    sim = MissileSimulationTrue6DOF(config)
    results = sim.run_simulation(
        launch_angle_deg=angle,
        azimuth_deg=90,
        sim_time=600
    )
    results_list.append(results)

# 비교 플롯
plt.figure(figsize=(10, 6))
for i, results in enumerate(results_list):
    plt.plot(results['range_km'], results['Z']/1000, 
             label=f'{angles[i]}°', linewidth=2)

plt.xlabel('Range (km)')
plt.ylabel('Altitude (km)')
plt.title('Trajectory Comparison (True 6DOF)')
plt.legend()
plt.grid(True)
plt.show()
```

---

## 🛠️ 고급 사용법

### 커스텀 추력 프로파일

```python
def custom_thrust(t):
    """시간 의존 추력 프로파일"""
    if t < 10:
        return 300000  # 초기 부스트
    elif t < 60:
        return 200000  # 유지 추력
    else:
        return 0

config = get_scud_b_config()
config['thrust_profile'] = custom_thrust
```

### 풍속 효과 추가 (향후 구현)

```python
# aerodynamics_true_6dof.py에 풍속 추가
def calculate_aerodynamic_forces(self, rho, V_mag, u, v, w, mach, wind_vector):
    # 상대 풍속 계산
    u_rel = u - wind_vector[0]
    v_rel = v - wind_vector[1]
    w_rel = w - wind_vector[2]
    
    # 받음각 계산 (상대 풍속 기준)
    alpha, beta = self.calculate_angles_of_attack(u_rel, v_rel, w_rel, V_rel_mag)
    # ...
```

---

## ⚠️ 주의사항

### 1. 짐벌락 (Gimbal Lock)

- **문제**: θ = ±90°에서 오일러각 특이점
- **해결**: 
  - 쿼터니언 사용 (`Quaternion` 클래스)
  - 극한 각도 회피

### 2. 수치 안정성

- **stiff ODE**: 빠른 회전 운동
- **해결**: `solve_ivp` 파라미터 조정
  ```python
  sol = solve_ivp(
      dynamics,
      [0, t_end],
      state0,
      method='Radau',  # stiff solver
      rtol=1e-6,
      atol=1e-8,
      max_step=0.5
  )
  ```

### 3. 좌표계 일관성

- **중요**: 모든 벡터의 좌표계를 명확히
- **관례**:
  - Body Frame: (u, v, w), (p, q, r)
  - Inertial Frame: (X, Y, Z)
  - 주석으로 명시!

---

## 📚 참고 문헌

1. **Stevens & Lewis** - "Aircraft Control and Simulation" (3rd Ed.)
   - Chapter 1-2: 좌표계 및 동역학
   
2. **Zipfel** - "Modeling and Simulation of Aerospace Vehicle Dynamics"
   - Chapter 3: 6DOF 방정식
   
3. **Tewari** - "Atmospheric and Space Flight Dynamics"
   - Chapter 4: 회전 운동

4. **Fleeman** - "Tactical Missile Design" (2nd Ed.)
   - Chapter 8: 궤적 시뮬레이션

---

## 🤝 기여

개선 사항이나 버그 발견 시:
1. 이슈 등록
2. 테스트 케이스 추가
3. Pull Request

---

## 📝 라이선스

MIT License

---

## 🆚 비교: 기존 코드와 차이점

### `main_6dof.py` (패션 6DOF)
```python
# 병진 운동 (각속도 무관)
dV_dt = (T - D) / M_safe - g * np.sin(gamma)
dgamma_dt = 0.0  # 수직상승

# 회전 운동 (독립적)
dp_dt = L_aero / Ixx
```

### `main_true_6dof.py` (진짜 6DOF)
```python
# 병진 운동 (각속도 직접 영향!)
u_dot = r*v - q*w + F_x / m + g_body_x
v_dot = p*w - r*u + F_y / m + g_body_y

# 회전 운동 (자이로 커플링)
dp_dt = (L_aero + (Iyy - Izz) * q * r) / Ixx
```

**핵심 차이**: 각속도가 병진 속도 변화에 **직접 영향**!

---

## ✅ TODO

- [ ] 쿼터니언 기반 적분 (짐벌락 완전 제거)
- [ ] 풍속 효과 추가
- [ ] 질량 특성 변화 (연료 소모에 따른 무게중심 이동)
- [ ] 공력 계수 실험 데이터 반영
- [ ] GPU 가속 (대량 시뮬레이션용)
- [ ] 실시간 3D 시각화
- [ ] Monte Carlo 불확실성 분석

---

**작성일**: 2025-10-27  
**버전**: 1.0  
**작성자**: Cascade AI
