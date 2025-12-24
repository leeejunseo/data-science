# 프로젝트 구조 정리 완료

## 📁 최종 프로젝트 구조

```
data-science/
├── final/                          # 🎯 메인 실행 폴더
│   ├── config_6dof.py              # 6DOF 미사일 설정 (SCUD-B, Nodong, KN-23)
│   ├── missile_6dof_true.py        # 6DOF 물리 시뮬레이션 엔진
│   ├── trajectory_io.py            # NPZ 데이터 저장/로드 유틸리티
│   ├── main_visualization.py       # 궤적 시각화 시스템
│   ├── signature_extractor.py      # 시그니처 특성 추출 (32차원)
│   ├── signature_generator.py      # ML 학습용 데이터셋 생성
│   ├── train_large.py              # ML 모델 학습 (RandomForest)
│   ├── eval_by_angle.py            # 발사각 그룹별 성능 평가
│   ├── game_launcher.py            # 🎮 게임 백엔드 API 서버
│   ├── view_npz.py                 # NPZ 파일 뷰어
│   ├── requirements.txt            # Python 의존성
│   ├── README.md                   # 상세 사용 가이드
│   ├── outputs/                    # 시뮬레이션 출력
│   │   └── trajectories/           # NPZ 궤적 파일
│   ├── results_6dof/               # 6DOF 시뮬레이션 결과
│   ├── signature_dataset/          # ML 학습 데이터
│   │   ├── dataset_large.npz       # 학습 데이터셋
│   │   ├── model_rf.joblib         # 학습된 모델
│   │   └── scaler.joblib           # 특성 스케일러
│   ├── signature_datasets/         # 추가 데이터셋
│   └── trained_models/             # 학습된 ML 모델
│
├── jangjunha/                      # 📚 참고 연구 코드
│   ├── config.py                   # 초기 설정 참고
│   ├── improved_pattern_generator.py
│   └── main.py                     # 초기 구현 참고
│
├── lastpang/                       # 🎨 React 프론트엔드 UI
│   ├── src/                        # React 소스 코드
│   ├── public/                     # 정적 파일
│   ├── package.json                # Node.js 의존성
│   └── ...
│
├── run_game.bat                    # 🚀 게임 실행 스크립트 (백엔드+프론트엔드)
├── .gitignore
└── 2팀.hwp                         # 프로젝트 문서

```

## 🎯 핵심 기능

### 1. 미사일 시뮬레이션 (6DOF 물리)
- **파일**: `final/missile_6dof_true.py`
- **기능**: SCUD-B, Nodong, KN-23 세 종류 미사일의 6자유도 비행 시뮬레이션
- **출력**: NPZ 형식의 궤적 데이터

### 2. 시각화
- **파일**: `final/main_visualization.py`
- **기능**: 3D 궤적, 속도, 고도, 각도 등 다양한 그래프 생성

### 3. 시그니처 분석
- **파일**: `final/signature_extractor.py`, `final/signature_generator.py`
- **기능**: 32차원 시그니처 특성 추출 및 데이터셋 생성
- **특성**: 기하학적(8), 속도(6), 가속도(4), 6DOF 고유(10), 파생(4)

### 4. ML 분류 모델
- **학습**: `final/train_large.py` - RandomForest 모델 학습
- **평가**: `final/eval_by_angle.py` - 발사각 그룹별 일반화 성능 평가
- **모델**: `final/signature_dataset/model_rf.joblib`

### 5. 게임 UI
- **백엔드**: `final/game_launcher.py` (Flask API, 포트 5000)
- **프론트엔드**: `lastpang/` (React)
- **실행**: `run_game.bat`
- **기능**: 
  - 랜덤 미사일 발사
  - 시그니처 분석 및 ML 예측
  - 사용자가 미사일 종류 맞추기

## 🚀 실행 방법

### 게임 실행 (통합)
```bash
run_game.bat
```
- Python 백엔드 서버 자동 시작 (포트 5000)
- React 프론트엔드 자동 시작 (포트 3000)

### 개별 실행

#### 1. 미사일 시뮬레이션 + 시각화
```bash
cd final
python main_visualization.py
```

#### 2. ML 모델 학습
```bash
cd final
python train_large.py
```

#### 3. 모델 평가
```bash
cd final
python eval_by_angle.py
```

#### 4. 게임 서버만 실행
```bash
cd final
python game_launcher.py --port 5000
```

## 🗑️ 제거된 파일

### 폴더 (중복 연구 코드)
- ❌ `JJINleejunseo/` (43개 파일) - 중복 연구 코드
- ❌ `yeonu/` (52개 파일) - 중복 연구 코드
- ❌ `results_6dof/` (루트) - 오래된 결과 파일

### final/ 폴더 내 불필요 파일
- ❌ `config.py` - `config_6dof.py`와 중복
- ❌ `kn23_depressed.py` - `missile_6dof_true.py`에 통합됨
- ❌ `missile_manager.py` - `game_launcher.py`로 대체
- ❌ `run_kn23_batch.py` - 테스트 스크립트
- ❌ `test_*.py` - 개발용 테스트 파일들
- ❌ `test_kn23.npz` - 테스트 데이터
- ❌ `test_output/` - 테스트 출력 폴더

## 📊 정리 결과

| 항목 | 이전 | 이후 | 감소 |
|------|------|------|------|
| 폴더 수 | 7개 | 3개 | -4개 |
| final/ 파일 | 27개 | 17개 | -10개 |
| 전체 구조 | 복잡 | 간결 | ✅ |

## ✅ 최적화 완료

프로젝트가 다음과 같이 최적화되었습니다:
1. **중복 제거**: 연구 과정의 중복 코드 삭제
2. **구조 단순화**: 핵심 기능만 유지
3. **실행 준비**: `run_game.bat`로 원클릭 실행 가능
4. **유지보수성**: 명확한 파일 구조와 역할 분리

---
*정리 완료: 2024-12-24*
