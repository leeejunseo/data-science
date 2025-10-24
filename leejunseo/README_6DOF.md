# 🚀 3DOF → 6DOF 미사일 시뮬레이션 업그레이드 완료!

## 📦 제공 파일

1. **config_6dof.py** - 6DOF 설정 파일 (관성 모멘트, 공력 계수 포함)
2. **main_6dof.py** - 6DOF 시뮬레이션 메인 코드
3. **CONVERSION_GUIDE.md** - 3DOF에서 6DOF로 전환하는 상세 가이드
4. **compare_3dof_6dof.py** - 테스트 및 비교 스크립트

---

## ⚡ 빠른 시작

### 방법 1: 즉시 실행 (기존 코드 유지)

```bash
# 6DOF 단독 실행
python3 main_6dof.py
```

### 방법 2: 기존 코드를 6DOF로 전환

```bash
# 기존 파일 백업
cp config.py config_3dof_backup.py
cp main.py main_3dof_backup.py

# 6DOF로 교체
cp config_6dof.py config.py
cp main_6dof.py main.py

# 실행 (명령어 동일!)
python3 main.py
```

### 방법 3: improved_pattern_generator.py 수정

**첫 줄만 수정:**
```python
# Before
import config as cfg
from main import MissileSimulation

# After  
import config_6dof as cfg
from main_6dof import MissileSimulation6DOF as MissileSimulation
```

---

## 🔍 주요 차이점

### 상태 벡터

| 항목 | 3DOF | 6DOF |
|------|------|------|
| **차원** | 8차원 | **14차원** |
| **병진 운동** | V, γ, ψ, x, y, h | V, γ, ψ, x, y, h |
| **회전 운동** | ❌ | **✅ φ, θ, ψ, p, q, r** |
| **질량** | W, M | M, fuel |

### 새로운 6DOF 기능 ✨

1. **🔄 회전 운동 시뮬레이션**
   - 롤각 (φ), 피치각 (θ), 요각 (ψ)
   - 롤 각속도 (p), 피치 각속도 (q), 요 각속도 (r)

2. **🌪️ 공력 모멘트 계산**
   - 롤 모멘트 (L_aero)
   - 피치 모멘트 (M_aero)
   - 요 모멘트 (N_aero)

3. **⚖️ 관성 모멘트 고려**
   - Ixx, Iyy, Izz (kg·m²)

4. **📊 추가 시각화**
   - 오일러 각도 그래프 (φ, θ, ψ)
   - 각속도 그래프 (p, q, r)

---

## 🎯 실행 예시

### 테스트 스크립트 실행

```bash
python3 compare_3dof_6dof.py
```

**예상 출력:**
```
✅ 모든 테스트 통과!
🚀 이제 6DOF 시뮬레이션을 사용할 수 있습니다!

📈 6DOF 결과 요약:
   - 최종 사거리: 332.29 km
   - 최대 고도: 735.68 km
   - 비행 시간: 300.00 s
   - 최종 롤각: 0.00°
   - 최종 피치각: 16.55°
```

---

## 📚 자세한 내용

- **CONVERSION_GUIDE.md** - 명령어 변경 방법 상세 가이드
- **config_6dof.py** - 관성 모멘트 및 공력 계수 설정
- **main_6dof.py** - 6DOF 운동 방정식 구현

---

## 🔧 미사일별 설정

### SCUD-B 예시
```python
# 관성 모멘트 (kg·m²)
inertia_xx: 58729.12  # 피치/요 관성
inertia_yy: 58729.12
inertia_zz: 567.25    # 롤 관성

# 공력 모멘트 계수
cl_alpha: 0.5        # 양력 기울기
cm_alpha: -0.15      # 피칭 모멘트 (정적 안정)
cn_beta: -0.1        # 요잉 모멘트
cl_p: -0.02          # 롤 댐핑
cm_q: -0.05          # 피치 댐핑
cn_r: -0.05          # 요 댐핑
```

---

## ✅ 검증 완료

- ✅ 모듈 로드 성공
- ✅ 시뮬레이션 실행 성공
- ✅ 결과 시각화 성공
- ✅ 3DOF와 호환성 유지

---

## 💡 팁

1. **기존 3DOF 유지**: 파일명이 다르므로 3DOF와 6DOF 모두 사용 가능
2. **점진적 전환**: 한 번에 하나씩 변경하고 테스트
3. **백업 필수**: 원본 파일은 항상 백업
4. **결과 비교**: 3DOF와 6DOF 결과를 비교하여 차이 확인

---

## 🎓 핵심 요약

### 사용할 명령어

```bash
# ✅ 추천: 6DOF 단독 실행 (기존 코드 그대로 유지)
python3 main_6dof.py

# ✅ 대안: 기존 코드를 6DOF로 전환
# 1. 백업
cp config.py config_3dof.py
cp main.py main_3dof.py

# 2. 교체
cp config_6dof.py config.py  
cp main_6dof.py main.py

# 3. 실행
python3 main.py
```

### 코드 수정이 필요한 경우

```python
# main.py 또는 improved_pattern_generator.py 첫 줄
import config_6dof as cfg  # config → config_6dof
from main_6dof import MissileSimulation6DOF  # 추가
```

---

## 📞 도움이 필요하신가요?

1. **CONVERSION_GUIDE.md** 참고
2. **compare_3dof_6dof.py** 실행하여 테스트
3. 에러 발생 시 파일 위치 확인

---

**🎉 축하합니다! 이제 현실적인 6DOF 미사일 시뮬레이션을 사용할 수 있습니다!** 🚀
