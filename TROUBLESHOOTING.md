# 미사일 시뮬레이션 ML 분석 시스템 문제 해결 가이드

## 수정 완료 사항

### 1. ML 모델 로드 개선 ✅
**파일:** `final/game_launcher.py`

**개선 사항:**
- ML 모델 파일 존재 여부 상세 확인
- 모델 로드 실패 시 구체적인 에러 메시지 출력
- Scaler, feature_names, missile_types 파일이 없어도 기본값으로 동작
- 상세한 로그 출력으로 디버깅 용이

**확인 방법:**
```bash
# 서버 시작 시 다음 메시지 확인:
✅ ML 모델 로드 완료:
   - 모델: RandomForest
   - 특성: 15개
   - 미사일 타입: ['SCUD-B', 'Nodong', 'KN-23']
```

### 2. NPZ 데이터 로드 개선 ✅
**파일:** `final/game_launcher.py`

**개선 사항:**
- `q` (pitch rate) 필드 추가로 15개 특징 완전 추출
- 누락된 필드에 대한 안전한 처리

### 3. /api/analyze 엔드포인트 개선 ✅
**파일:** `final/game_launcher.py`

**개선 사항:**
- 각 단계별 상세 로그 출력
- NPZ 로드, 특징 추출, ML 예측 각 단계 확인 가능
- 에러 발생 시 구체적인 원인 출력

**로그 예시:**
```
============================================================
API /api/analyze 호출
============================================================
NPZ 경로: c:\...\results_6dof\SCUD-B_angle30_azim90_seed1234.npz
실제 미사일: SCUD-B
✅ NPZ 데이터 로드 성공 (샘플 수: 1234)
✅ 시그니처 분석 완료
   예측: SCUD-B
   신뢰도: 85.5%
✅ API 응답 준비 완료
============================================================
```

### 4. ML 시그니처 분석 개선 ✅
**파일:** `final/game_launcher.py`

**개선 사항:**
- 15개 특징 추출 과정 로그 출력
- 특징 정규화 상태 확인
- ML 예측 결과 및 확률 분포 출력
- 에러 발생 시 traceback 출력

### 5. 그래프 생성 개선 ✅
**파일:** `final/game_launcher.py`

**개선 사항:**
- matplotlib backend 설정 개선
- 미사일 타입 자동 인식 강화
- base64 이미지 생성 과정 로그 출력
- 에러 처리 강화

## 실행 방법

### 1. 서버 재시작
```bash
cd c:\Users\User\Desktop\25-2교과\데이터과학\project\data-science
.\run_game.bat
```

### 2. 터미널 출력 확인
서버 시작 시 다음을 확인하세요:

```
============================================================
ML 모델 로드 시도
============================================================
모델 경로: c:\...\final\trained_models\rf_model.pkl
모델 존재: True

✅ ML 모델 로드 완료:
   - 모델: RandomForest
   - 특성: 15개
   - 미사일 타입: ['SCUD-B', 'Nodong', 'KN-23']
============================================================
```

### 3. 브라우저에서 테스트
1. http://localhost:3000 접속
2. "시뮬레이션 시작" 클릭
3. "예측하기" 클릭
4. 터미널에서 ML 처리 과정 확인
5. UI에서 결과 확인

## 문제 해결

### ML 모델이 없는 경우

**증상:**
```
❌ ML 모델 파일 없음: c:\...\trained_models\rf_model.pkl
```

**해결 방법:**
```bash
cd c:\Users\User\Desktop\25-2교과\데이터과학\project\data-science\final
python eval_by_angle.py
```

이 명령은 다음을 생성합니다:
- `trained_models/rf_model.pkl` - RandomForest 모델
- `trained_models/scaler.pkl` - StandardScaler
- `trained_models/feature_names.pkl` - 특징 이름
- `trained_models/missile_types.pkl` - 미사일 타입

### NPZ 파일이 없는 경우

**증상:**
```
⚠ No NPZ files found in results_6dof/
```

**해결 방법:**
게임을 시작하면 자동으로 실시간 시뮬레이션이 실행되어 NPZ 파일이 생성됩니다.
또는 수동으로 생성:
```bash
cd final
python main_visualization.py
```

### 그래프가 표시되지 않는 경우

**확인 사항:**
1. 터미널에서 "그래프 생성 완료" 메시지 확인
2. 브라우저 개발자 도구(F12) 콘솔에서 에러 확인
3. 네트워크 탭에서 `/api/analyze` 응답 확인

### UI에 ML 결과가 표시되지 않는 경우

**확인 사항:**
1. 브라우저 콘솔(F12)에서 에러 확인
2. `/api/analyze` 응답에 `identification` 필드 있는지 확인
3. 브라우저 새로고침(F5)

## API 응답 형식

### /api/analyze 정상 응답
```json
{
  "status": "analyzed",
  "identification": {
    "predicted_type": "SCUD-B",
    "confidence": 85.5,
    "reasons": [
      "고도 120.5km",
      "사거리 280.3km",
      "최대마하 5.23"
    ],
    "features": {
      "max_altitude_km": "120.5",
      "range_km": "280.3",
      "flight_time_s": "254.7",
      "has_pullup": false
    },
    "all_probabilities": {
      "SCUD-B": 85.5,
      "Nodong": 10.2,
      "KN-23": 4.3
    },
    "method": "ML (RandomForest, 15 features)"
  },
  "graph_image": "iVBORw0KGgoAAAANSUhEUgAA...",
  "actual_missile": "SCUD-B"
}
```

## 디버깅 팁

### 1. 터미널 로그 확인
모든 주요 단계에서 상세한 로그가 출력됩니다:
- ML 모델 로드
- NPZ 데이터 로드
- 특징 추출
- ML 예측
- 그래프 생성

### 2. 브라우저 개발자 도구
- F12 → Console: JavaScript 에러 확인
- F12 → Network: API 요청/응답 확인

### 3. 단계별 테스트
```bash
# 1. ML 모델 테스트
cd final
python -c "import joblib; model = joblib.load('trained_models/rf_model.pkl'); print('OK')"

# 2. 시뮬레이션 테스트
python main_visualization.py

# 3. 백엔드 테스트
python game_launcher.py
```

## 주요 변경 파일

1. `final/game_launcher.py` - 백엔드 로직 개선
2. `lastpang/src/GameApp.js` - 프론트엔드 (변경 없음, 이미 정상)

## 다음 단계

서버를 재시작하고 터미널 출력을 확인하세요. 
문제가 발생하면 터미널 로그를 공유해주세요.
