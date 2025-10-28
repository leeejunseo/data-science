# 🛠️ 개발 가이드 (Development Guide)

## 목차
- [개발 환경 설정](#개발-환경-설정)
- [테스트 방법](#테스트-방법)
- [배포 절차](#배포-절차)
- [main_fixed.py 주요 변경사항](#main_fixedpy-주요-변경사항)

---

# 📋 개발 환경 설정

## 1. IDE 설정

### Visual Studio Code (권장)

#### 추천 설정 (settings.json)
```json
{
    "python.defaultInterpreterPath": "${workspaceFolder}/venv/bin/python",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "python.formatting.provider": "black",
    "editor.formatOnSave": true,
    "editor.rulers": [100],
    "files.trimTrailingWhitespace": true
}
```

#### 필수 확장
```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-python.black-formatter
code --install-extension eamodio.gitlens
```

## 2. 코드 스타일 가이드

### PEP 8 준수

```python
# 1. 들여쓰기: 4칸 공백
def function_name():
    if condition:
        do_something()

# 2. 최대 줄 길이: 100자
result = some_function(
    parameter1, parameter2,
    parameter3, parameter4
)

# 3. 네이밍 컨벤션
class ClassName:           # PascalCase
    pass

def function_name():       # snake_case
    pass

CONSTANT_NAME = 100        # UPPER_SNAKE_CASE
```

### Docstring 형식 (NumPy Style)

```python
def calculate_force(velocity, altitude):
    """
    공력을 계산합니다.
    
    Parameters
    ----------
    velocity : float
        속도 (m/s)
    altitude : float
        고도 (m)
    
    Returns
    -------
    force : float
        힘 (N)
    """
    pass
```

### 코드 품질 도구

```bash
# Black (포매터)
pip install black
black main_6dof.py

# Flake8 (린터)
pip install flake8
flake8 main_6dof.py

# Pylint (정적 분석)
pip install pylint
pylint main_6dof.py
```

---

# 🧪 테스트 방법

## 1. 단위 테스트 작성 및 실행

### 테스트 구조
```
JJINleejunseo/
├── tests/
│   ├── __init__.py
│   ├── test_config.py
│   ├── test_simulation.py
│   └── test_integration.py
├── main_6dof.py
└── config_6dof.py
```

### pytest 설치
```bash
pip install pytest pytest-cov
```

### 테스트 작성 예시

```python
# tests/test_simulation.py
import pytest
from main_fixed import MissileSimulation6DOF

class TestMissileSimulation:
    @pytest.fixture
    def sim(self):
        return MissileSimulation6DOF(missile_type="SCUD-B")
    
    def test_initialization(self, sim):
        """초기화 테스트"""
        sim.initialize_simulation(launch_angle_deg=45, sim_time=1500)
        assert sim.results is not None
    
    def test_simulation_runs(self, sim):
        """시뮬레이션 실행 테스트"""
        sim.initialize_simulation(sim_time=100)
        sim.run_simulation()
        assert len(sim.results['time']) > 0
    
    def test_velocity_positive(self, sim):
        """속도가 양수인지 확인"""
        sim.initialize_simulation(sim_time=100)
        sim.run_simulation()
        for v in sim.results['velocity']:
            assert v > 0
```

### 테스트 실행

```bash
# 모든 테스트 실행
pytest

# 특정 파일
pytest tests/test_simulation.py

# 커버리지 측정
pytest --cov=. --cov-report=html

# 상세 출력
pytest -v -s
```

---

## 2. 통합 테스트 절차

### 통합 테스트 예시

```python
# tests/test_integration.py
import pytest
import os
from main_fixed import MissileSimulation6DOF

@pytest.mark.integration
class TestIntegration:
    def test_end_to_end(self):
        """전체 시뮬레이션 테스트"""
        sim = MissileSimulation6DOF(missile_type="SCUD-B")
        sim.initialize_simulation(launch_angle_deg=45, sim_time=1500)
        sim.run_simulation()
        
        assert len(sim.results['time']) > 0
        assert sim.results['x'][-1] > 0
        
        sim.plot_results_6dof()
        # 결과 파일이 results_6dof 폴더에 생성되는지 확인
        assert os.path.exists('results_6dof')
```

---

## 3. 테스트 커버리지 목표

### 목표
- **전체 커버리지**: 80% 이상
- **핵심 모듈**: 90% 이상

### 커버리지 확인
```bash
pytest --cov=. --cov-report=term-missing
pytest --cov=. --cov-report=html
# htmlcov/index.html 열기
```

---

# 🚀 배포 절차

## 1. 코드 빌드 방법

### 가상 환경 생성
```bash
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows
pip install -r requirements.txt
```

### requirements.txt 생성
```bash
pip freeze > requirements.txt
```

또는 수동 작성:
```
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0
```

---

## 2. 실행 환경 설정

### 환경 변수 (.env)
```bash
MISSILE_TYPE=SCUD-B
SIM_TIME=300
LAUNCH_ANGLE=45
AZIMUTH=90
OUTPUT_DIR=./results
DEBUG=False
```

### Docker 사용

`Dockerfile`:
```dockerfile
FROM python:3.9-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
CMD ["python", "main_6dof.py"]
```

실행:
```bash
docker build -t missile-sim .
docker run -v $(pwd)/results:/app/results missile-sim
```

---

## 3. 배포 체크리스트

### 릴리스 전 확인사항

- [ ] **코드 품질**
  - [ ] 모든 테스트 통과
  - [ ] 커버리지 80% 이상
  - [ ] Lint 경고 없음

- [ ] **문서화**
  - [ ] README.md 업데이트
  - [ ] USER_GUIDE.md 업데이트
  - [ ] CHANGELOG.md 작성

- [ ] **버전 관리**
  - [ ] 버전 번호 업데이트
  - [ ] Git 태그 생성
  - [ ] 릴리스 노트 작성

- [ ] **호환성**
  - [ ] Python 3.7-3.10 테스트
  - [ ] Windows/Linux/Mac 테스트

### 버전 관리 (SemVer)

```
MAJOR.MINOR.PATCH
예: 1.0.0
```

### Git 태그 생성
```bash
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0
```

### CHANGELOG.md 예시
```markdown
# Changelog

## [1.0.0] - 2025-01-27

### Added
- 6DOF 미사일 시뮬레이션
- 회전 운동 지원
- 공력 모멘트 계산

### Changed
- 3DOF → 6DOF 업그레이드

### Fixed
- 고도 계산 오류 수정
```

---

## 배포 자동화

### GitHub Actions

`.github/workflows/release.yml`:
```yaml
name: Release

on:
  push:
    tags:
      - 'v*'

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.9'
    - name: Install dependencies
      run: pip install -r requirements.txt
    - name: Run tests
      run: pytest
    - name: Create Release
      uses: actions/create-release@v1
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
      with:
        tag_name: ${{ github.ref }}
        release_name: Release ${{ github.ref }}
```



**🎯 개발 시작 전 체크리스트:**
1. ✅ 가상 환경 활성화
2. ✅ 의존성 설치
3. ✅ IDE 설정 완료
4. ✅ Git 설정 확인
5. ✅ 테스트 실행 확인

---

# main_fixed.py 주요 변경사항

## 🆕 버그 수정 및 안정화

### 1. 78km 멈춤 버그 수정
**문제**: 실시간 시뮬레이션이 78km 고도에서 멈추는 현상
**원인**: `sim_time` 기본값이 설정되지 않아 중간단계 시뮬레이션이 조기 종료
**해결**:
```python
# main_fixed.py 라인 99-100
self.sim_time = sim_time if sim_time is not None else 1500
```

### 2. 공력 모멘트 계산 안정화
**문제**: 과도한 공력 모멘트로 인한 수치 발산
**해결**: 라인 198-227에서 제한 적용
```python
# 받음각 제한: ±45도
alpha = np.clip(theta - gamma, -np.pi/4, np.pi/4)

# 각속도 정규화: ±10 rad/s
p_norm = np.clip(p, -10.0, 10.0)

# 모멘트 제한: ±1e6 N·m
L_aero = np.clip(q_dynamic * self.wing_area * self.length * Cl, -max_moment, max_moment)
```

### 3. 오일러 각도 변환 안정화
**문제**: 짐벌락 발생 가능성
**해결**: 라인 181-196에서 강화된 예외 처리
```python
# cos(theta) ≈ 0 예외 처리
cos_theta = np.cos(theta)
tan_theta = np.tan(theta) if abs(cos_theta) > 0.01 else 0.0

# 각도 변화율 제한: ±5 rad/s
dphi_dt = np.clip(dphi_dt, -max_angle_rate, max_angle_rate)
```

## 🎨 새로운 기능

### 1. 실시간 3D 시각화 모드
**함수**: `run_simulation_realtime()` (라인 522-800)
**기능**:
- matplotlib 대화형 모드(plt.ion()) 활용
- 4단계 비행 과정 실시간 애니메이션
- 속도, 고도, 시간 정보 실시간 표시

**사용법**:
```python
sim = MissileSimulation6DOF()
sim.run_simulation_realtime()  # 자동으로 초기화 및 실행
```

### 2. 2가지 실행 모드 지원
**모드 1**: 실시간 3D 궤적 시뮬레이션 (기본)
- 사용자가 비행 과정을 실시간으로 관찰
- 교육 및 시연용으로 적합

**모드 2**: 상세 결과 그래프 (12-패널)
- 모든 물리량의 시간 변화 분석
- 연구 및 데이터 분석용으로 적합

## 📊 코드 구조 개선

### 파일 크기 및 라인 수
- **main_fixed.py**: 978줄 (기존 main_6dof.py 대비 +284줄)
- 주요 추가: 실시간 시각화 시스템 (279줄)

### 주요 함수 위치
| 함수명 | 라인 | 설명 |
|--------|------|------|
| `plot_with_guarantee()` | 22-37 | 그래프 저장/표시 유틸리티 |
| `calculate_euler_rates()` | 181-196 | 오일러 각도 변환 (안정화) |
| `calculate_aerodynamic_moments()` | 198-227 | 공력 모멘트 (스무딩) |
| `dynamics_vertical_6dof()` | 229-287 | 수직 상승 동역학 |
| `dynamics_pitch_6dof()` | 289-344 | 피치 전환 동역학 |
| `dynamics_constant_6dof()` | 346-348 | 등자세 비행 |
| `dynamics_midcourse_6dof()` | 350-391 | 중간단계 비행 |
| `run_simulation()` | 400-520 | 모드 2 실행 |
| `run_simulation_realtime()` | 522-800 | 모드 1 실행 |
| `plot_results_6dof()` | 802-931 | 12-패널 시각화 |

## 🧪 테스트 가이드

### 안정성 테스트
```python
from main_fixed import MissileSimulation6DOF

# 다양한 발사각에서 테스트
for angle in range(10, 91, 10):
    sim = MissileSimulation6DOF()
    sim.initialize_simulation(launch_angle_deg=angle, sim_time=1500)
    try:
        results = sim.run_simulation()
        print(f"✅ {angle}° 성공: 사거리 {results['x'][-1]/1000:.2f}km")
    except Exception as e:
        print(f"❌ {angle}° 실패: {e}")
```

### 실시간 시각화 테스트
```bash
python main_fixed.py
# 모드 1 선택 후 정상 작동 확인
```

## 📝 변경 이력

**v1.1.0 (2025-10-28)** - main_fixed.py
- ✅ 78km 멈춤 버그 수정
- ✅ 공력 모멘트 안정화
- ✅ 오일러 각도 변환 안정화
- ✅ 실시간 3D 시각화 추가
- ✅ 2가지 실행 모드 지원

**v1.0.0 (2025-10-21)** - main_6dof.py
- ✅ 6DOF 시뮬레이션 초기 구현
- ✅ 12-패널 시각화 시스템

---

**Happy Coding! 🚀**
