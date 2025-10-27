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

---

# 📖 기능별 사용법

## 1. 기본 시뮬레이션 실행

### 목적
6DOF 미사일 궤적 시뮬레이션을 실행하여 병진 운동과 회전 운동을 모두 포함한 현실적인 비행 궤적을 계산합니다.

### 사용 절차

#### 단계 1: 환경 확인
```bash
# Python 버전 확인 (3.7 이상 필요)
python --version

# 필요한 패키지 확인
pip list | grep -E "numpy|scipy|matplotlib"
```

#### 단계 2: 시뮬레이션 실행
```bash
# 기본 실행
python main_6dof.py
```

#### 단계 3: 결과 확인
- 콘솔에 출력되는 비행 정보 확인
- 생성된 그래프 파일 확인 (PNG 형식)
- 시뮬레이션 결과 데이터 분석

### 예시 출력
```
=== 6DOF 미사일 시뮬레이션 시작 ===
미사일 타입: SCUD-B
발사각: 45.0°
방위각: 90.0°

[Phase 1] 부스트 단계 (0.0 - 60.0s)
  추력: 250000 N
  연료 소모율: 50.0 kg/s

[Phase 2] 자유비행 단계 (60.0 - 300.0s)
  추력: 0 N

최종 결과:
  사거리: 332.29 km
  최대 고도: 735.68 km
  비행 시간: 300.00 s
  최종 속도: 1245.32 m/s
```

### 주의사항
- ⚠️ 시뮬레이션 시간이 길 경우 계산에 수 분이 소요될 수 있습니다
- ⚠️ 그래프 창이 자동으로 닫히지 않으면 수동으로 닫아주세요
- ⚠️ 메모리 부족 시 시뮬레이션 시간을 줄여보세요

---

## 2. 미사일 타입 변경

### 목적
다양한 미사일 모델(SCUD-B, Hwasong-5 등)의 특성을 반영한 시뮬레이션을 수행합니다.

### 사용 절차

#### 단계 1: config_6dof.py 파일 열기
```bash
# 텍스트 에디터로 열기
notepad config_6dof.py  # Windows
vim config_6dof.py      # Linux/Mac
```

#### 단계 2: 미사일 파라미터 수정
```python
# SCUD-B 예시
MISSILE_PARAMS = {
    "SCUD-B": {
        "mass_initial": 6370,      # 초기 질량 (kg)
        "mass_fuel": 3800,         # 연료 질량 (kg)
        "diameter": 0.88,          # 직경 (m)
        "length": 11.25,           # 길이 (m)
        "thrust": 250000,          # 추력 (N)
        "isp": 230,                # 비추력 (s)
        "burn_time": 60,           # 연소 시간 (s)
        
        # 🆕 관성 모멘트 (kg·m²)
        "inertia_xx": 58729.12,    # 피치/요 관성
        "inertia_yy": 58729.12,
        "inertia_zz": 567.25,      # 롤 관성
        
        # 🆕 공력 모멘트 계수
        "cl_alpha": 0.5,           # 양력 기울기
        "cm_alpha": -0.15,         # 피칭 모멘트
        "cn_beta": -0.1,           # 요잉 모멘트
        "cl_p": -0.02,             # 롤 댐핑
        "cm_q": -0.05,             # 피치 댐핑
        "cn_r": -0.05,             # 요 댐핑
    }
}
```

#### 단계 3: 새로운 미사일 추가
```python
# 사용자 정의 미사일 추가
MISSILE_PARAMS["Custom-1"] = {
    "mass_initial": 5000,
    "mass_fuel": 2500,
    # ... 나머지 파라미터
}
```

#### 단계 4: 시뮬레이션 실행
```python
# main_6dof.py에서 미사일 타입 지정
sim = MissileSimulation6DOF(missile_type="Custom-1")
```

### 주의사항
- ⚠️ 관성 모멘트는 미사일의 형상과 질량 분포에 따라 정확히 계산해야 합니다
- ⚠️ 공력 계수는 풍동 실험 또는 CFD 해석 결과를 사용하는 것이 좋습니다
- ⚠️ 비현실적인 값을 입력하면 시뮬레이션이 발산할 수 있습니다

---

## 3. 발사 조건 설정

### 목적
발사각, 방위각 등 초기 조건을 변경하여 다양한 비행 시나리오를 시뮬레이션합니다.

### 사용 절차

#### 단계 1: Python 스크립트 작성
```python
from main_6dof import MissileSimulation6DOF

# 시뮬레이션 객체 생성
sim = MissileSimulation6DOF(missile_type="SCUD-B")

# 초기 조건 설정
launch_angle = 45      # 발사각 (도)
azimuth = 90           # 방위각 (도)
sim_time = 300         # 시뮬레이션 시간 (초)

# 시뮬레이션 초기화
sim.initialize_simulation(
    launch_angle_deg=launch_angle,
    azimuth_deg=azimuth,
    sim_time=sim_time
)

# 실행
sim.run_simulation()
sim.plot_results()
```

#### 단계 2: 다양한 시나리오 테스트
```python
# 최대 사거리 시나리오
for angle in range(30, 60, 5):
    sim = MissileSimulation6DOF()
    sim.initialize_simulation(launch_angle_deg=angle)
    sim.run_simulation()
    print(f"발사각 {angle}°: 사거리 {sim.results['x'][-1]/1000:.2f} km")
```

### 예시 결과
```
발사각 30°: 사거리 285.43 km
발사각 35°: 사거리 310.27 km
발사각 40°: 사거리 328.15 km
발사각 45°: 사거리 332.29 km  ← 최대
발사각 50°: 사거리 325.18 km
발사각 55°: 사거리 308.92 km
```

### 주의사항
- ⚠️ 발사각은 0~90° 범위 내에서 설정하세요
- ⚠️ 방위각은 0~360° 범위입니다 (0° = 북, 90° = 동)
- ⚠️ 시뮬레이션 시간이 너무 짧으면 미사일이 착탄하기 전에 종료됩니다

---

## 4. 3DOF vs 6DOF 비교

### 목적
3DOF와 6DOF 시뮬레이션 결과를 비교하여 회전 운동의 영향을 분석합니다.

### 사용 절차

#### 단계 1: 비교 스크립트 실행
```bash
python compare_3dof_6dof.py
```

#### 단계 2: 결과 분석
스크립트는 다음을 자동으로 수행합니다:
- 3DOF 시뮬레이션 실행
- 6DOF 시뮬레이션 실행
- 결과 비교 및 차이 계산
- 비교 그래프 생성

### 예시 출력
```
=== 3DOF vs 6DOF 비교 ===

3DOF 결과:
  사거리: 335.12 km
  최대 고도: 740.25 km
  비행 시간: 300.00 s

6DOF 결과:
  사거리: 332.29 km
  최대 고도: 735.68 km
  비행 시간: 300.00 s
  최종 롤각: 0.00°
  최종 피치각: 16.55°

차이:
  사거리 차이: -2.83 km (-0.84%)
  고도 차이: -4.57 km (-0.62%)
```

### 주의사항
- ⚠️ 6DOF는 공력 모멘트와 관성 효과를 고려하므로 더 현실적입니다
- ⚠️ 차이가 클 경우 공력 계수 설정을 확인하세요
- ⚠️ 3DOF는 계산이 빠르지만 회전 운동을 무시합니다

---

## 5. 결과 시각화 및 분석

### 목적
시뮬레이션 결과를 다양한 그래프로 시각화하여 비행 특성을 분석합니다.

### 사용 절차

#### 단계 1: 기본 그래프 생성
```python
sim = MissileSimulation6DOF()
sim.initialize_simulation()
sim.run_simulation()
sim.plot_results()  # 모든 그래프 자동 생성
```

#### 단계 2: 생성되는 그래프
1. **궤적 그래프** (trajectory_plot.png)
   - 2D 평면상의 비행 경로
   - X축: 수평 거리 (km)
   - Y축: 고도 (km)

2. **속도 및 각도 그래프** (velocity_angle_plot.png)
   - 속도, 비행경로각, 방위각 시간 변화

3. **오일러 각도 그래프** (euler_angles_plot.png) 🆕
   - 롤각 (φ), 피치각 (θ), 요각 (ψ)
   - 미사일의 자세 변화 추적

4. **각속도 그래프** (angular_rates_plot.png) 🆕
   - 롤 각속도 (p), 피치 각속도 (q), 요 각속도 (r)
   - 회전 운동의 동역학 분석

5. **공력 데이터 그래프** (aerodynamic_plot.png)
   - 받음각, 항력계수, 마하수

#### 단계 3: 사용자 정의 그래프
```python
import matplotlib.pyplot as plt

# 사거리 vs 고도 그래프
plt.figure(figsize=(10, 6))
plt.plot(
    [x/1000 for x in sim.results['x']],
    [h/1000 for h in sim.results['h']],
    'b-', linewidth=2
)
plt.xlabel('사거리 (km)')
plt.ylabel('고도 (km)')
plt.title('미사일 궤적')
plt.grid(True)
plt.savefig('custom_trajectory.png', dpi=300)
plt.show()
```

### 주의사항
- ⚠️ 그래프 파일은 현재 디렉토리에 저장됩니다
- ⚠️ 기존 파일이 있으면 덮어쓰기됩니다
- ⚠️ 고해상도 그래프는 파일 크기가 클 수 있습니다

---

# 🔧 트러블슈팅

## 자주 발생하는 에러

### 1. ModuleNotFoundError: No module named 'numpy'

**에러 메시지:**
```
ModuleNotFoundError: No module named 'numpy'
```

**원인:** 필요한 Python 패키지가 설치되지 않았습니다.

**해결 방법:**
```bash
# 필수 패키지 설치
pip install numpy scipy matplotlib

# 또는 requirements.txt 사용 (있는 경우)
pip install -r requirements.txt
```

---

### 2. ValueError: setting an array element with a sequence

**에러 메시지:**
```
ValueError: setting an array element with a sequence.
```

**원인:** 배열 차원이 맞지 않거나 잘못된 데이터 타입이 전달되었습니다.

**해결 방법:**
```python
# config_6dof.py에서 파라미터 확인
# 모든 수치가 스칼라 값인지 확인

# 잘못된 예:
"thrust": [250000]  # 리스트 형태 ❌

# 올바른 예:
"thrust": 250000    # 스칼라 값 ✅
```

---

### 3. RuntimeWarning: invalid value encountered in double_scalars

**에러 메시지:**
```
RuntimeWarning: invalid value encountered in double_scalars
```

**원인:** 계산 중 0으로 나누기 또는 NaN 값이 발생했습니다.

**해결 방법:**
```python
# 1. 초기 조건 확인
# 속도가 0이 되지 않도록 설정

# 2. 시뮬레이션 시간 단축
sim_time = 200  # 300 → 200으로 줄이기

# 3. 수치 안정성 개선
# config_6dof.py에서
DT_MAX = 0.1  # 시간 간격 줄이기
```

---

### 4. 그래프가 표시되지 않음

**증상:** 시뮬레이션은 완료되지만 그래프 창이 나타나지 않습니다.

**원인:** Matplotlib 백엔드 설정 문제

**해결 방법:**
```python
# main_6dof.py 상단에 추가
import matplotlib
matplotlib.use('TkAgg')  # Windows/Linux
# matplotlib.use('MacOSX')  # Mac의 경우

import matplotlib.pyplot as plt
```

또는

```bash
# 환경 변수 설정 (Linux/Mac)
export MPLBACKEND=TkAgg
python main_6dof.py

# Windows PowerShell
$env:MPLBACKEND="TkAgg"
python main_6dof.py
```

---

### 5. MemoryError

**에러 메시지:**
```
MemoryError: Unable to allocate array
```

**원인:** 시뮬레이션 데이터가 너무 많아 메모리 부족

**해결 방법:**
```python
# 1. 시뮬레이션 시간 단축
sim_time = 150  # 300 → 150

# 2. 데이터 저장 간격 증가
# main_6dof.py에서
if len(self.t) % 10 == 0:  # 매 10번째 데이터만 저장
    self.results['time'].append(t)
    # ...

# 3. 불필요한 데이터 삭제
del sim.states  # 시뮬레이션 후 중간 데이터 삭제
```

---

## 설치 문제

### 1. pip 설치 실패

**증상:**
```
ERROR: Could not install packages due to an EnvironmentError
```

**해결 방법:**
```bash
# 관리자 권한으로 실행 (Windows)
pip install --user numpy scipy matplotlib

# 가상 환경 사용 (권장)
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows
pip install numpy scipy matplotlib
```

---

### 2. Python 버전 호환성

**증상:** 특정 기능이 작동하지 않거나 문법 에러 발생

**해결 방법:**
```bash
# Python 버전 확인
python --version

# Python 3.7 이상 필요
# 버전이 낮으면 업그레이드

# Windows
choco install python

# Linux (Ubuntu)
sudo apt update
sudo apt install python3.9

# Mac
brew install python@3.9
```

---

### 3. 의존성 충돌

**증상:**
```
ERROR: package-name has requirement other-package!=X.Y.Z
```

**해결 방법:**
```bash
# 1. 가상 환경 새로 생성
python -m venv fresh_env
source fresh_env/bin/activate

# 2. 패키지 버전 명시
pip install numpy==1.21.0 scipy==1.7.0 matplotlib==3.4.0

# 3. 의존성 트리 확인
pip install pipdeptree
pipdeptree
```

---

## 실행 문제

### 1. 시뮬레이션이 너무 느림

**증상:** 시뮬레이션 완료까지 수십 분 소요

**해결 방법:**
```python
# 1. 시뮬레이션 시간 단축
sim_time = 100  # 기본값보다 짧게

# 2. 적분 정밀도 조정
# main_6dof.py의 solve_ivp 호출 부분
sol = solve_ivp(
    self.equations_of_motion,
    [0, self.sim_time],
    state0,
    method='RK45',
    rtol=1e-6,  # 1e-9 → 1e-6 (정밀도 낮춤)
    atol=1e-8,  # 1e-12 → 1e-8
    max_step=1.0  # 최대 시간 간격 증가
)

# 3. 불필요한 계산 제거
# 디버그 출력 비활성화
VERBOSE = False
```

---

### 2. 결과가 비현실적임

**증상:** 사거리가 수천 km이거나 음수 고도 발생

**해결 방법:**
```python
# 1. 초기 조건 확인
launch_angle = 45  # 0~90 범위
azimuth = 90       # 0~360 범위

# 2. 미사일 파라미터 확인
# config_6dof.py에서
# - 질량이 양수인지
# - 추력이 합리적인지 (10^4 ~ 10^6 N)
# - 연소 시간이 적절한지 (30~120초)

# 3. 공력 계수 확인
# - CD0는 0.1~0.5 범위
# - CL_alpha는 0.3~0.7 범위

# 4. 관성 모멘트 확인
# - Ixx, Iyy는 비슷한 값
# - Izz는 Ixx의 1/100 정도
```

---

### 3. 파일을 찾을 수 없음

**에러 메시지:**
```
FileNotFoundError: [Errno 2] No such file or directory: 'config_6dof.py'
```

**해결 방법:**
```bash
# 1. 현재 디렉토리 확인
pwd  # Linux/Mac
cd   # Windows

# 2. 파일 목록 확인
ls -la  # Linux/Mac
dir     # Windows

# 3. 올바른 디렉토리로 이동
cd /path/to/JJINleejunseo

# 4. 파일 존재 확인
ls config_6dof.py main_6dof.py

# 5. Python 경로 확인
python -c "import sys; print(sys.path)"
```

---

### 4. 권한 에러

**에러 메시지:**
```
PermissionError: [Errno 13] Permission denied
```

**해결 방법:**
```bash
# 1. 파일 권한 확인 (Linux/Mac)
ls -l main_6dof.py

# 2. 실행 권한 부여
chmod +x main_6dof.py

# 3. 쓰기 권한 확인 (그래프 저장용)
chmod 755 .

# 4. 관리자 권한으로 실행 (Windows)
# PowerShell을 관리자 권한으로 실행
```

---

## 추가 도움말

### 로그 활성화

```python
# main_6dof.py에 로깅 추가
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='simulation.log'
)

logger = logging.getLogger(__name__)
logger.info("시뮬레이션 시작")
```

### 디버그 모드

```python
# 상세 출력 활성화
DEBUG = True

if DEBUG:
    print(f"Time: {t:.2f}s")
    print(f"Velocity: {V:.2f} m/s")
    print(f"Altitude: {h:.2f} m")
    print(f"Roll: {phi:.2f}°")
```

### 성능 프로파일링

```python
import cProfile
import pstats

# 시뮬레이션 프로파일링
profiler = cProfile.Profile()
profiler.enable()

sim.run_simulation()

profiler.disable()
stats = pstats.Stats(profiler)
stats.sort_stats('cumulative')
stats.print_stats(10)  # 상위 10개 함수 출력
```

---

## 문의 및 지원

문제가 해결되지 않으면:

1. **로그 파일 확인**: `simulation.log` 파일의 에러 메시지 확인
2. **GitHub Issues**: 프로젝트 저장소에 이슈 등록
3. **문서 참조**: `CONVERSION_GUIDE.md`, `DEV_GUIDE.md` 참고
4. **커뮤니티**: Stack Overflow에 질문 게시

---

**💡 팁:** 문제 발생 시 에러 메시지 전체와 실행 환경 정보(OS, Python 버전, 패키지 버전)를 함께 제공하면 빠른 해결이 가능합니다!
