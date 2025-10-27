# 🛠️ 개발 가이드 (Development Guide)

## 목차
- [개발 환경 설정](#개발-환경-설정)
- [테스트 방법](#테스트-방법)
- [배포 절차](#배포-절차)

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

---

## 2. 필요한 플러그인/확장

### VS Code 확장 목록
- **Python** - Python 개발 지원
- **Pylance** - 빠른 타입 체킹
- **Black Formatter** - 코드 포매팅
- **GitLens** - Git 통합
- **Markdown All in One** - 문서 작성

---

## 3. 코드 스타일 가이드

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
from main_6dof import MissileSimulation6DOF

class TestMissileSimulation:
    @pytest.fixture
    def sim(self):
        return MissileSimulation6DOF(missile_type="SCUD-B")
    
    def test_initialization(self, sim):
        """초기화 테스트"""
        sim.initialize_simulation(launch_angle_deg=45)
        assert sim.results is not None
    
    def test_simulation_runs(self, sim):
        """시뮬레이션 실행 테스트"""
        sim.initialize_simulation(sim_time=10)
        sim.run_simulation()
        assert len(sim.results['time']) > 0
    
    def test_velocity_positive(self, sim):
        """속도가 양수인지 확인"""
        sim.initialize_simulation(sim_time=50)
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
from main_6dof import MissileSimulation6DOF

@pytest.mark.integration
class TestIntegration:
    def test_end_to_end(self):
        """전체 시뮬레이션 테스트"""
        sim = MissileSimulation6DOF(missile_type="SCUD-B")
        sim.initialize_simulation(launch_angle_deg=45, sim_time=300)
        sim.run_simulation()
        
        assert len(sim.results['time']) > 0
        assert sim.results['x'][-1] > 0
        
        sim.plot_results()
        assert os.path.exists('trajectory_plot.png')
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

---

## 추가 리소스

- **PEP 8**: https://pep8.org/
- **pytest 문서**: https://docs.pytest.org/
- **Black**: https://black.readthedocs.io/
- **Docker**: https://docs.docker.com/

---

**🎯 개발 시작 전 체크리스트:**
1. ✅ 가상 환경 활성화
2. ✅ 의존성 설치
3. ✅ IDE 설정 완료
4. ✅ Git 설정 확인
5. ✅ 테스트 실행 확인

**Happy Coding! 🚀**
