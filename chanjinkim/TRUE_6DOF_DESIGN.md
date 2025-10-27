# 🚀 진짜 6DOF 미사일 시뮬레이션 설계

## 📋 개요

기존 "패션 6DOF"의 문제점을 해결하고 **물리적으로 정확한** 6DOF 미사일 궤적 시뮬레이션을 구현합니다.

---

## 🔍 기존 문제점 분석

### ❌ 패션 6DOF의 문제
1. **회전과 병진이 독립적** - 물리적 커플링 없음
2. **좌표계 변환 부재** - Body Frame과 Inertial Frame 구분 없음
3. **공력 모멘트 미사용** - 계산만 하고 실제 영향 없음
4. **자세가 궤적에 무영향** - 롤, 피치, 요가 비행에 영향 없음

---

## ✅ 진짜 6DOF 구조

### 1. 좌표계 정의

#### **Body Frame (기체 좌표계)**
```
    +Z (up)
     |
     |
     +------ +X (nose)
    /
   /
 +Y (right wing)
```

- **속도**: (u, v, w) - Body Frame에서의 속도 성분
- **각속도**: (p, q, r) - 롤, 피치, 요 각속도

#### **Inertial Frame (관성 좌표계)**
```
    +Z (up)
     |
     |
     +------ +X (North)
    /
   /
 +Y (East)
```

- **위치**: (X, Y, Z) - 지구 고정 좌표계
- **오일러각**: (φ, θ, ψ) - 롤, 피치, 요

### 2. 좌표계 변환

#### **Direction Cosine Matrix (DCM)**

```
DCM = R_z(ψ) * R_y(θ) * R_x(φ)

    [cos(θ)cos(ψ)    sin(φ)sin(θ)cos(ψ)-cos(φ)sin(ψ)    cos(φ)sin(θ)cos(ψ)+sin(φ)sin(ψ)]
  = [cos(θ)sin(ψ)    sin(φ)sin(θ)sin(ψ)+cos(φ)cos(ψ)    cos(φ)sin(θ)sin(ψ)-sin(φ)cos(ψ)]
    [-sin(θ)         sin(φ)cos(θ)                       cos(φ)cos(θ)                    ]
```

**용도**:
- Body → Inertial: `V_inertial = DCM * V_body`
- Inertial → Body: `V_body = DCM^T * V_inertial`

---

## 🔬 동역학 방정식

### 1. 병진 운동 (Body Frame)

```
du/dt = rv - qw - g*sin(θ) + (F_x / m)
dv/dt = pw - ru + g*cos(θ)*sin(φ) + (F_y / m)
dw/dt = qu - pv + g*cos(θ)*cos(φ) + (F_z / m)
```

**핵심**: 각속도(p, q, r)가 **직접** 병진 속도 변화에 영향!

**힘 성분 (Body Frame)**:
- `F_x = T - D_x` (추력 - 항력)
- `F_y = -Y` (측력)
- `F_z = -L` (양력, Body Frame에서는 -Z 방향)

### 2. 회전 운동 (Body Frame)

**오일러 각운동 방정식**:
```
p_dot = (L_aero + (I_yy - I_zz) * q * r) / I_xx
q_dot = (M_aero + (I_zz - I_xx) * p * r) / I_yy
r_dot = (N_aero + (I_xx - I_yy) * p * q) / I_zz
```

**오일러각 변화율**:
```
φ_dot = p + q*sin(φ)*tan(θ) + r*cos(φ)*tan(θ)
θ_dot = q*cos(φ) - r*sin(φ)
ψ_dot = (q*sin(φ) + r*cos(φ)) / cos(θ)
```

### 3. 위치 업데이트 (Inertial Frame)

```
[X_dot]       [u]
[Y_dot] = DCM [v]
[Z_dot]       [w]
```

---

## 🌪️ 공력 모델 (자세 의존)

### 1. 받음각 & 측면 받음각

```
V_rel = V_wind - V_body

α (angle of attack) = atan2(w, u)
β (sideslip angle) = asin(v / |V_rel|)
```

**핵심**: Body Frame 속도 성분(u, v, w)으로 계산!

### 2. 공력 계수

```
C_D = C_D0 + K * C_L^2 + C_Dα * α^2
C_L = C_L0 + C_Lα * α
C_Y = C_Yβ * β
```

### 3. 공력 힘 (Body Frame)

```
q_dyn = 0.5 * ρ * V_rel^2

D = q_dyn * S * C_D  (항력, -X 방향)
Y = q_dyn * S * C_Y  (측력, +Y 방향)
L = q_dyn * S * C_L  (양력, -Z 방향)
```

### 4. 공력 모멘트 (Body Frame)

```
L_aero = q_dyn * S * b * (C_l0 + C_lβ * β + C_lp * (p*b)/(2V) + C_lr * (r*b)/(2V))
M_aero = q_dyn * S * c * (C_m0 + C_mα * α + C_mq * (q*c)/(2V))
N_aero = q_dyn * S * b * (C_n0 + C_nβ * β + C_np * (p*b)/(2V) + C_nr * (r*b)/(2V))
```

**핵심**: 각속도(p, q, r)가 공력 모멘트에 영향 → **댐핑 효과**

---

## 🔗 커플링 효과

### 1. 회전 → 병진
- 각속도가 Body Frame 속도에 직접 영향 (`du/dt = rv - qw ...`)
- 자세 변화로 중력 방향 변경
- 공력 방향 변경 (DCM 변환)

### 2. 병진 → 회전
- 속도 변화로 동압 변화 → 공력 모멘트 변화
- 받음각 변화 → 피칭 모멘트 변화
- 상대 풍속 변화 → 댐핑 효과 변화

### 3. 회전 ↔ 회전
- 각운동량 보존 (자이로 효과)
- 관성 커플링 (`(I_yy - I_zz) * q * r`)

---

## 📦 구현 모듈 구조

### 1. `coordinate_transforms.py`
- DCM 계산
- 쿼터니언 변환
- Body ↔ Inertial 변환

### 2. `aerodynamics_6dof.py`
- 받음각/측면 받음각 계산
- 공력 계수 (마하수, 받음각 의존)
- 공력 힘 & 모멘트 (Body Frame)

### 3. `dynamics_6dof.py`
- Body Frame 병진 운동 방정식
- 오일러 각운동 방정식
- 질량 변화

### 4. `main_true_6dof.py`
- 전체 통합
- 시뮬레이션 루프
- 결과 시각화

---

## 🎯 검증 방법

### 1. 단위 테스트
- 좌표계 변환 정확도
- DCM 직교성 검증 (`DCM * DCM^T = I`)
- 오일러각 특이점 처리

### 2. 물리적 타당성
- 에너지 보존 (추력 에너지 = 운동 에너지 + 위치 에너지 + 손실)
- 각운동량 보존 (외부 모멘트 없을 때)
- 자세 안정성 (정적/동적 안정)

### 3. 극한 케이스
- 수직 발사 (θ = 90°)
- 낮은 각도 발사 (θ < 15°)
- 고속 회전 (spin-stabilized)

### 4. 비교 검증
- 3DOF와 비교 (회전 없는 경우 일치해야 함)
- 문헌 데이터와 비교

---

## 📊 예상 결과

### 패션 6DOF vs 진짜 6DOF

| 항목 | 패션 6DOF | 진짜 6DOF |
|------|-----------|----------|
| **궤적 정확도** | 3DOF와 동일 | 자세 영향 반영 |
| **회전 효과** | 독립적 (의미 없음) | 궤적에 직접 영향 |
| **공력 모멘트** | 계산만 함 | 실제 사용 |
| **자세 안정성** | 분석 불가 | 안정성 분석 가능 |
| **계산 비용** | 3DOF + α | 2~3배 증가 |
| **물리적 타당성** | ❌ 낮음 | ✅ 높음 |

---

## 🚀 구현 우선순위

1. **Phase 1**: 좌표계 변환 모듈 (DCM) ✅
2. **Phase 2**: Body Frame 동역학 방정식 ✅
3. **Phase 3**: 공력 모델 재설계 ✅
4. **Phase 4**: 전체 통합 및 테스트 ⏳
5. **Phase 5**: 최적화 및 문서화 ⏳

---

## 📚 참고 문헌

1. Stevens & Lewis - "Aircraft Control and Simulation" (3rd Ed.)
2. Zipfel - "Modeling and Simulation of Aerospace Vehicle Dynamics"
3. Tewari - "Atmospheric and Space Flight Dynamics"
4. Fleeman - "Tactical Missile Design" (2nd Ed.)

---

## ⚠️ 주의사항

1. **오일러각 특이점**: θ = ±90°에서 짐벌락 발생 → 쿼터니언 사용 권장
2. **수치 안정성**: 강성(stiff) ODE → 적절한 적분기 필요
3. **좌표계 일관성**: 모든 벡터의 좌표계를 명확히 표시
4. **단위 통일**: SI 단위 사용 권장

---

**작성일**: 2025-10-27  
**작성자**: Cascade AI  
**버전**: 1.0
