# 🔄 3DOF → 6DOF 전환 가이드

## 📋 명령어 변경 방법

### ✅ 방법 1: 파일 교체 (가장 간단)

기존 파일들을 백업하고 6DOF 파일로 교체하세요:

```bash
# 1. 기존 파일 백업
cp config.py config_3dof_backup.py
cp main.py main_3dof_backup.py

# 2. 6DOF 파일을 기본 파일로 복사
cp config_6dof.py config.py
cp main_6dof.py main.py

# 3. 그대로 실행 (명령어 변경 없음!)
python3 main.py
```

---

### ✅ 방법 2: Import 문만 수정 (코드 수정)

기존 코드 파일들을 직접 수정하는 방법:

#### 📝 main.py 수정

**기존 (3DOF):**
```python
import config as cfg
```

**변경 (6DOF):**
```python
import config_6dof as cfg
```

#### 📝 improved_pattern_generator.py 수정

**기존 (3DOF):**
```python
import config as cfg
from main import MissileSimulation
```

**변경 (6DOF):**
```python
import config_6dof as cfg
from main_6dof import MissileSimulation6DOF as MissileSimulation
```

또는:
```python
import config_6dof as cfg
from main_6dof import MissileSimulation6DOF

# 클래스 사용 시
simulator = MissileSimulation6DOF(missile_type=m_type, apply_errors=False)
```

---

### ✅ 방법 3: 새로운 6DOF 전용 실행 파일 생성

기존 코드는 그대로 두고, 새로운 6DOF 전용 스크립트 생성:

#### 📝 run_6dof.py (새로 생성)

```python
#!/usr/bin/env python3
"""6DOF 미사일 시뮬레이션 실행 스크립트"""

# 6DOF 모듈 import
import config_6dof as cfg
from main_6dof import MissileSimulation6DOF

def main():
    print("=" * 60)
    print("6DOF 미사일 궤적 시뮬레이션")
    print("=" * 60)
    
    # 6DOF 시뮬레이션 실행
    simulation = MissileSimulation6DOF(
        missile_type="SCUD-B", 
        apply_errors=False
    )
    
    simulation.initialize_simulation(
        launch_angle_deg=45,
        azimuth_deg=90,
        sim_time=1500
    )
    
    results = simulation.run_simulation()
    
    if results is not None:
        simulation.plot_results_6dof()
        
        # 결과 요약 출력
        import numpy as np
        final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
        max_altitude = np.max(results['h']) / 1000
        
        print(f"\n최종 사거리: {final_range:.2f} km")
        print(f"최대 고도: {max_altitude:.2f} km")
        print(f"비행 시간: {results['time'][-1]:.2f} s")

if __name__ == "__main__":
    main()
```

**실행:**
```bash
python3 run_6dof.py
```

---

## 📊 improved_pattern_generator.py를 6DOF로 변환

### 전체 파일 수정 방법:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6DOF Natural Trajectory Pattern Data Generator
"""

import numpy as np
import os
import config_6dof as cfg  # ⬅️ 여기 변경!
import traceback
import matplotlib.pyplot as plt
from tqdm import tqdm
import itertools

try:
    from main_6dof import MissileSimulation6DOF  # ⬅️ 여기 변경!
except ImportError:
    print("Error: Cannot find MissileSimulation6DOF class in main_6dof.py")
    exit()

class NaturalTrajectoryDataGenerator:
    """6DOF 자연스러운 궤도 패턴 데이터 생성기"""
    
    def __init__(self, output_dir="natural_trajectory_data_6dof"):  # ⬅️ 폴더명 변경
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.launch_angles = list(range(10, 81, 3))
        self.azimuth_angles = list(range(30, 151, 15))
        
        print(f"🌟 6DOF 자연스러운 궤도 데이터 생성기 초기화")
        # ... (나머지 동일)
    
    def generate_comprehensive_natural_dataset(self, missile_types, samples_per_combination=3, sim_time=1500):
        """6DOF 전면적 자연 궤도 데이터셋 생성"""
        
        # ... (중간 코드 생략)
        
        for m_type in missile_types:
            if m_type not in cfg.MISSILE_TYPES:
                print(f"   ⚠️ 미사일 '{m_type}' not found")
                continue
            
            for launch_angle in self.launch_angles:
                for azimuth_angle in self.azimuth_angles:
                    for sample_idx in range(samples_per_combination):
                        try:
                            # 미사일 타입 설정
                            if not cfg.set_missile_type(m_type):
                                continue
                            
                            # 🔄 6DOF 시뮬레이터 사용
                            simulator = MissileSimulation6DOF(  # ⬅️ 여기 변경!
                                missile_type=m_type, 
                                apply_errors=False
                            )
                            
                            simulator.initialize_simulation(
                                launch_angle_deg=actual_angle,
                                azimuth_deg=actual_azimuth,
                                sim_time=sim_time
                            )
                            
                            results = simulator.run_simulation(sim_time=sim_time)
                            
                            # ... (결과 처리는 동일)
                            
                        except Exception as e:
                            continue
        
        # ... (나머지 동일)

# main 함수는 동일
```

---

## 🎯 요약: 어떤 방법을 선택할까?

### 🥇 추천: 방법 1 (파일 교체)
- **장점**: 가장 간단, 명령어 변경 불필요
- **단점**: 원본 파일 백업 필수
- **사용 시나리오**: 완전히 6DOF로 전환할 때

### 🥈 추천: 방법 3 (새 파일 생성)
- **장점**: 3DOF/6DOF 모두 유지 가능
- **단점**: 파일이 늘어남
- **사용 시나리오**: 3DOF와 6DOF를 비교하며 사용

### 🥉 비추천: 방법 2 (코드 수정)
- **장점**: 기존 구조 유지
- **단점**: 여러 파일 수정 필요, 실수 가능성
- **사용 시나리오**: 기존 코드를 완전히 이해하고 있을 때

---

## 📝 실전 예시

### 시나리오 A: "6DOF만 사용하고 싶어요"

```bash
# 백업
mv config.py config_3dof.py.bak
mv main.py main_3dof.py.bak

# 6DOF를 기본으로
cp config_6dof.py config.py
cp main_6dof.py main.py

# 실행 (명령어 동일!)
python3 main.py
```

### 시나리오 B: "3DOF와 6DOF를 둘 다 사용하고 싶어요"

```bash
# 아무것도 변경하지 않고 각각 실행
python3 main.py              # 3DOF
python3 main_6dof.py         # 6DOF

# 데이터 생성도 별도로
python3 improved_pattern_generator.py           # 3DOF 데이터
python3 improved_pattern_generator_6dof.py      # 6DOF 데이터 (새로 생성)
```

### 시나리오 C: "기존 코드를 6DOF로 업그레이드"

**main.py 첫 줄만 수정:**
```python
# Before
import config as cfg

# After
import config_6dof as cfg
```

**improved_pattern_generator.py 첫 줄 수정:**
```python
# Before
import config as cfg
from main import MissileSimulation

# After
import config_6dof as cfg
from main_6dof import MissileSimulation6DOF as MissileSimulation
```

---

## ✅ 변경 확인 방법

수정 후 다음 명령어로 6DOF가 제대로 작동하는지 확인:

```bash
python3 -c "
import config_6dof as cfg
print('✅ Config 6DOF 로드 성공')
print(f'상태 벡터 차원: {cfg.StateVector6DOF.STATE_DIM}')
"
```

**예상 출력:**
```
✅ Config 6DOF 로드 성공
상태 벡터 차원: 14
```

---

## 🔧 문제 해결

### Q1: "ModuleNotFoundError: No module named 'config_6dof'"

**해결:**
```bash
# config_6dof.py 파일이 있는지 확인
ls -la config_6dof.py

# 없다면 현재 디렉토리 확인
pwd
```

### Q2: "MissileSimulation6DOF 클래스를 찾을 수 없습니다"

**해결:**
```bash
# main_6dof.py 파일 확인
ls -la main_6dof.py

# 클래스명 확인
grep "class.*Simulation" main_6dof.py
```

### Q3: "3DOF와 6DOF 결과가 너무 달라요"

**정상입니다!** 6DOF는 회전 운동까지 고려하므로:
- 더 현실적인 궤적
- 롤/피치/요 운동 포함
- 공력 모멘트 효과 반영

---

## 🎓 핵심 정리

| 변경 항목 | 3DOF | 6DOF |
|----------|------|------|
| **Config 파일** | `import config` | `import config_6dof` |
| **Main 파일** | `from main import MissileSimulation` | `from main_6dof import MissileSimulation6DOF` |
| **클래스명** | `MissileSimulation()` | `MissileSimulation6DOF()` |
| **결과 플롯** | `plot_results()` | `plot_results_6dof()` |
| **상태 벡터** | 8차원 | 14차원 |

---

## 💡 팁

1. **백업 필수**: 원본 파일은 항상 백업하세요
2. **점진적 전환**: 한 번에 하나씩 변경하고 테스트
3. **결과 비교**: 3DOF와 6DOF 결과를 비교해보세요
4. **문서화**: 변경 사항을 메모하세요

---

이 가이드로 3DOF 코드를 6DOF로 쉽게 전환할 수 있습니다! 🚀
