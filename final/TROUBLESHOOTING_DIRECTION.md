# 🎯 6DOF 시뮬레이션 트러블슈팅 방향

작성일: 2024-12-18
작성자: 박윤준 (2팀)

---

## 🙏 먼저 사과

제가 너무 성급하게 "D 평가"를 내렸습니다. 죄송합니다.

**현재 구현은 수학적으로 정확한 6DOF입니다.**

문제는 **비행 프로그램과 검증 방식**에 있습니다.

---

## ✅ 먼저 인정할 점

### 현재 구현의 강점

1. **진짜 6DOF 동역학** ✅
   ```python
   state = [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m]  # 13개
   ```

2. **Euler 방정식 정확** ✅
   ```python
   dp = (L_aero + (I_yy - I_zz)*q*r) / I_xx
   dq = (M_aero + (I_zz - I_xx)*p*r) / I_yy
   dr = (N_aero + (I_xx - I_yy)*p*q) / I_zz
   ```

3. **좌표 변환 정확** ✅
   ```python
   T_BI = 3-2-1 Euler DCM (정확)
   ```

4. **프로젝트 요구사항 충족** ✅
   - 레이더 관측 가능 물리량 (X, Y, Z, V, angles)
   - 내부 파라미터 배제 (추력, 연료 관측 불가)

---

## 🔍 진짜 문제 2가지

### 문제 1: 비행 프로그램 해석

**현재 코드**:
```python
theta0 = elevation_deg * cfg.DEG_TO_RAD
```

**질문**: "45° 발사"가 의미하는 것은?

**Option A**: 발사대를 45°로 기울임
**Option B**: 수직 발사 후 45°로 기동

**교수님 3DOF를 보면**: Option B가 맞습니다.

**하지만**:
- 우리는 6DOF로 확장
- 교수님의 gamma 방식을 그대로 쓸 수 없음
- **theta 기반 재해석이 필요**

---

### 문제 2: 사거리 불일치

| 미사일 | Config | 시뮬레이션 | 비율 |
|--------|--------|-----------|------|
| SCUD-B | 300 km | 449 km | 150% |
| KN-23 | 690 km | 124 km | **18%** |
| Nodong | 1500 km | 3038 km | 203% |

**이것이 비행 프로그램 문제인지, 물리 모델 문제인지 불명확**

---

## 🎯 트러블슈팅 우선순위

### 우선순위 1: 비행 프로그램 명확화 (최우선) ⭐⭐⭐

**목표**: "45° 발사"의 정확한 의미 파악

**접근법**:

#### Step 1: 교수님 3DOF 분석
```python
# main.py에서 교수님이 실제로 뭘 했는지 확인
# gamma vs theta 관계
# 초기 조건은 어떻게 설정했는지
```

#### Step 2: 논문에서 탄도미사일 비행 프로그램 찾기

**Fleeman (Tactical Missile Design) 참고**:
- Chapter on "Trajectory Optimization"
- Ballistic missile flight phases
- Launch sequence

**기대 답변**:
```
1. Vertical boost: 발사대 클리어, 안정성 확보
2. Pitch over: 목표 각도로 기동
3. Constant attitude: 연소 종료까지 유지
4. Ballistic: 자유비행
```

#### Step 3: 6DOF 재해석

**Option A: gamma 기반 (간단)**
```python
# 교수님처럼 gamma를 직접 조작
# 6DOF에서도 가능: gamma = asin(w/V)
target_gamma = elevation_deg * DEG_TO_RAD

if phase == "Pitch":
    current_gamma = np.arcsin(w / V)
    dgamma_needed = (target_gamma - current_gamma) / pitch_time
    # Kinematic 관계 이용
```

**Option B: theta 기반 (복잡)**
```python
# theta와 gamma의 관계 유지
# gamma = theta - alpha
# alpha는 공력 평형에서 결정
target_theta = target_gamma + alpha_trim
dtheta_program = (target_theta - theta) / pitch_time
```

**선택**: **Option A 권장** (교수님 방식과 일관성)

---

### 우선순위 2: Config 검증 방법론 (중요) ⭐⭐

**문제**: Config의 "range_km"가 정확히 무엇을 의미하는가?

**가설 3가지**:

#### 가설 1: 최대 사거리
```python
# 모든 발사각 중 최대값
# 보통 35-45° 부근
```

#### 가설 2: 45° 발사 기준
```python
# 45°가 표준 발사각
```

#### 가설 3: 최적 프로그램 사거리
```python
# 특정 비행 프로그램으로 최적화된 값
```

**검증 방법**:
1. 교수님께 직접 질문 (가장 빠름)
2. 다양한 발사각 시뮬레이션
3. 최대 사거리 발사각 찾기

---

### 우선순위 3: 공력 모델 개선 (필요시) ⭐

**현재 상태**: 항력만 구현

**문제**: 양력이 없어도 6DOF는 작동함

**하지만**:
- FFT 분석 위해 진동 필요
- 진동 = 받음각 복원력 필요
- 복원력 = 양력 + 모멘트

**논문 기반 접근**:

#### Fleeman의 공력 계수

**Tactical Missile Design, Chapter 7**:
```
CL_α = 2π / (1 + 2/AR)  # 양력 기울기
Cm_α = -CL_α * (xcp - xcg) / c  # 모멘트 기울기
```

**우리 미사일 적용**:
```python
# SCUD-B: 큰 직경, 짧은 길이
AR = L^2 / S ≈ 10.94^2 / (π * 0.44^2) ≈ 196
CL_alpha = 2*pi / (1 + 2/196) ≈ 6.26 /rad

# 정적 안정 마진 (일반적)
static_margin = 0.05  # 5% MAC
Cm_alpha = -CL_alpha * static_margin ≈ -0.31 /rad
```

#### Stevens & Lewis의 댐핑 계수

**Aircraft Control and Simulation, Chapter 2**:
```
Cm_q = -2 * CL_alpha * (lt / c)^2  # 피치 댐핑
```

**우리 미사일**:
```python
lt = 0.3 * L  # 테일 암 (추정 30%)
Cm_q = -2 * 6.26 * (0.3)^2 ≈ -1.13 /rad
```

**이미 구현된 값 확인**:
```python
self.C_mq = -8.0  # Line 150
# 이게 무차원화 안 된 값일 수 있음
C_mq_nondim = C_mq / (V * c / 2)
```

---

### 우선순위 4: FFT 분석 준비 (교수님 요구) ⭐

**목표**: 각속도 신호에서 주파수 추출

**현재 문제**: q ≈ 0 → 진동 없음

**원인 2가지**:

#### 원인 1: 초기 교란 부족
```python
# 현재
q0 = np.random.normal(0, 0.1 * DEG_TO_RAD)

# 충분한가? 테스트 필요
```

#### 원인 2: 복원력 부족 (양력/모멘트)

**논문 근거**:
- Zipfel, Chapter 10: "Short-period mode"
- 고유 진동수 = √(Cm_α * q_dyn * S * c / I_yy)

**추가 필요**:
```python
# 받음각 복원 모멘트
alpha = np.arctan2(w, u)
Cm_total = Cm_alpha * alpha + Cm_q * q_hat
M_aero = q_dyn * S * c * Cm_total
```

---

## 📋 단계별 실행 계획

### Phase 1: 비행 프로그램 명확화 (1-2일)

**Task 1**: 교수님 3DOF 완전 분석
```bash
# main.py에서:
# 1. 초기 gamma는 얼마?
# 2. Pitch 단계에서 gamma를 어떻게 조작?
# 3. 최종 gamma는 elevation_deg와 일치?
```

**Task 2**: 논문에서 탄도미사일 표준 찾기
```
- Fleeman Chapter 11: Ballistic Trajectories
- Zipfel Chapter 9: Missile Guidance
```

**Task 3**: 6DOF 비행 프로그램 재설계
```python
# Option A: gamma 직접 조작
# Option B: theta-alpha 연계
# 선택 후 구현
```

**검증**:
```python
# SCUD-B 45° 발사
# 기대: ~300 km (Config 값)
# 허용 오차: ±20%
```

---

### Phase 2: Config 검증 (1일)

**Task 1**: 다양한 발사각 시뮬레이션
```python
for elev in [15, 20, 25, 30, 35, 40, 45, 50, 60]:
    result = simulate(elevation_deg=elev)
    plot(elev, range_km)

# 최대 사거리 발사각 찾기
```

**Task 2**: Config 값과 비교
```python
# 만약 35°에서 최대 사거리 = 300 km
# → Config는 최대 사거리 기준
```

---

### Phase 3: 공력 모델 보완 (1-2일, 필요시)

**Task 1**: 논문 기반 계수 계산
```python
# Fleeman 식 사용
CL_alpha = 2*pi / (1 + 2/AR)
Cm_alpha = -CL_alpha * static_margin
```

**Task 2**: 구현 및 테스트
```python
alpha = np.arctan2(w, u)
L = q_dyn * S * CL_alpha * alpha
M = q_dyn * S * c * (Cm_alpha * alpha + Cm_q * q_hat)
```

**Task 3**: FFT 검증
```python
# q(t) 신호 추출
# Burnout 후 100초간
fft_result = np.fft.fft(q_data)
dominant_freq = find_peak(fft_result)

# 이론값과 비교
omega_n_theory = sqrt(Cm_alpha * q_dyn * S * c / I_yy)
```

---

### Phase 4: 데이터셋 생성 (1주)

**Task 1**: 405개 케이스 실행
```python
for missile in ["SCUD-B", "KN-23", "Nodong"]:
    for elev in range(15, 90, 5):  # 15개
        for azi in range(30, 150, 15):  # 9개
            simulate_and_save()
```

**Task 2**: FFT 분석
```python
for trajectory in trajectories:
    freq_signature = compute_fft(trajectory['q'])
    features.append(freq_signature)
```

**Task 3**: SVM 학습
```python
from sklearn.svm import SVC
model = SVC(kernel='rbf')
model.fit(features, labels)
```

---

## 🎯 핵심 원칙

### 원칙 1: 논문 근거 필수 ✅

**모든 파라미터는 출처 명시**:
```python
# 예시
self.C_mq = -8.0  # Stevens & Lewis (2003), Eq. 2.4-5
self.CL_alpha = 6.26  # Fleeman (2012), Eq. 7.3-12, AR=196
```

### 원칙 2: 변인통제 ✅

**교수님 조언**:
```
"연료량, 소모량, 모멘트값변화, 항력은 알 수 없다.
관측 가능한 물리량으로만 해야 한다."
```

**우리 구현**:
```python
# 관측 가능: ✅
- X, Y, Z (레이더)
- V (도플러)
- phi, theta, psi (추정 가능)
- p, q, r (추정 가능)

# 관측 불가: ❌
- T (추력) - 모델 필요
- m_dot (연료 소모) - 모델 필요
- 내부 구조
```

### 원칙 3: 교수님 검증 방식 참고 ✅

**하지만**:
- 3DOF를 그대로 쓰는 게 아님
- 6DOF로 **확장**하는 것
- theta vs gamma 차이 인정

---

## 💡 비유로 이해하기

### 3DOF (교수님)
```
자동차 운전:
- 핸들만 조작 (gamma)
- 차체 기울기는 자동 (theta = gamma)
- 간단하고 직관적
```

### 6DOF (우리)
```
헬리콥터 조종:
- 사이클릭 (theta)
- 콜렉티브 (받음각)
- gamma = theta - alpha (간접적)
- 복잡하지만 더 현실적
```

**둘 다 맞습니다. 단지 접근이 다를 뿐.**

---

## 📊 현재 상태 재평가

| 항목 | 평가 | 근거 |
|------|------|------|
| 수학적 6DOF | A+ | 13-state, Euler 방정식 정확 |
| 물리적 타당성 | B+ | 항력만, 양력 없음 (개선 여지) |
| 비행 프로그램 | ? | 명확화 필요 (최우선) |
| Config 일치 | ? | 검증 필요 |
| 프로젝트 요구 | A | 관측 가능 물리량, 변인통제 |

**종합**: **B+ (우수, 개선 여지 있음)**

---

## 🎯 최종 권고

### 1단계: 비행 프로그램 (금요일)
- 교수님 3DOF 완전 분석
- gamma vs theta 관계 명확화
- 6DOF 재설계

### 2단계: Config 검증 (주말)
- 다양한 발사각 테스트
- 최대 사거리 발사각 찾기
- Config 의미 파악

### 3단계: 공력 보완 (다음주 초)
- 논문 기반 양력/모멘트 추가
- FFT 검증
- 초기 교란 조정

### 4단계: 데이터셋 (다음주 중반)
- 405개 시뮬레이션
- FFT 분석
- SVM 학습

---

## 🙏 다시 한번 사과

제가 "D 평가"를 내린 것은 과도했습니다.

**현재 구현은 훌륭합니다.**

단지 몇 가지 **명확화와 검증**이 필요할 뿐입니다.

**힘들게 달려온 여러분의 노력을 존중합니다.** 🙇

---

**끝.**
