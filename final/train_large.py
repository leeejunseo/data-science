#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
대량 궤적 생성 + ML 학습 파이프라인
===================================

목적:
- 1500~5000개 샘플 생성 (미사일 3종 × 여러 발사각 × 각도당 여러 샘플)
- 6DoF/KN-23 모델로 시뮬레이션 후 시그니처(특징 벡터) 추출
- RandomForest 분류 모델 학습
- 혼동행렬/분류리포트/정확도 출력
- feature_importances_ 기반 중요 시그니처 TOP10 출력
- 데이터셋 및 모델 저장

실행:
    python train_large.py

출력:
    - signature_dataset/dataset_large.npz (데이터셋)
    - signature_dataset/model_rf.joblib (학습 모델)
    - signature_dataset/scaler.joblib (정규화 스케일러)
"""

import sys
import os
import io

# Windows 콘솔 인코딩 문제 해결
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

from pathlib import Path

# 현재 디렉토리를 경로에 추가
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import warnings
warnings.filterwarnings('ignore')

# sklearn
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score

# joblib (모델 저장)
import joblib

# 프로젝트 모듈
from signature_generator import MissileSignatureGenerator
from trajectory_io import save_npz_generic


# =============================================================================
# 설정
# =============================================================================

OUTPUT_DIR = Path("signature_dataset")
DATASET_FILE = OUTPUT_DIR / "dataset_large.npz"
MODEL_FILE = OUTPUT_DIR / "model_rf.joblib"
SCALER_FILE = OUTPUT_DIR / "scaler.joblib"

# 데이터 생성 설정
# 빠른 테스트: LAUNCH_ANGLES = [30, 45, 60], SAMPLES_PER_ANGLE = 5
# 전체 학습: LAUNCH_ANGLES = list(range(15, 81, 2)), SAMPLES_PER_ANGLE = 50
LAUNCH_ANGLES = list(range(15, 81, 2))  # 15°~80°, 2° 간격 (33개)
SAMPLES_PER_ANGLE = 50                   # 각도당 50 샘플
NOISE_STD = 1.0                          # 발사각 노이즈 표준편차
MISSILE_TYPES = ["SCUD-B", "Nodong", "KN-23"]

# 예상 샘플 수: 3 * 33 * 50 = 4950


# =============================================================================
# 메인 함수
# =============================================================================

def main():
    print("\n" + "=" * 70)
    print("🚀 대량 궤적 생성 + ML 학습 파이프라인")
    print("=" * 70)
    
    # 출력 디렉토리 생성
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    # ---------------------------------------------------------------------
    # 1. 대량 데이터 생성
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊 STEP 1: 대량 데이터 생성")
    print("=" * 70)
    print(f"  미사일 타입: {MISSILE_TYPES}")
    print(f"  발사각 범위: {LAUNCH_ANGLES[0]}°~{LAUNCH_ANGLES[-1]}° ({len(LAUNCH_ANGLES)}개)")
    print(f"  각도당 샘플: {SAMPLES_PER_ANGLE}")
    print(f"  노이즈 std: {NOISE_STD}°")
    print(f"  예상 샘플 수: {len(MISSILE_TYPES) * len(LAUNCH_ANGLES) * SAMPLES_PER_ANGLE}")
    
    # MissileSignatureGenerator 생성 및 설정 변경
    generator = MissileSignatureGenerator(output_dir=str(OUTPUT_DIR))
    generator.launch_angles = LAUNCH_ANGLES  # 발사각 범위 변경
    
    # 데이터 생성
    features, labels, metadata = generator.generate_dataset(
        missile_types=MISSILE_TYPES,
        samples_per_angle=SAMPLES_PER_ANGLE,
        noise_std=NOISE_STD
    )
    
    if len(features) == 0:
        print("❌ 데이터 생성 실패! 샘플이 0개입니다.")
        return 1
    
    feature_names = metadata['feature_names']
    missile_types = metadata['missile_types']
    all_metadata = metadata['metadata']
    
    print(f"\n✅ 데이터 생성 완료:")
    print(f"   X.shape: {features.shape}")
    print(f"   총 샘플 수: {len(features)}")
    print(f"   특성 수: {len(feature_names)}")
    
    # ---------------------------------------------------------------------
    # 2. 데이터셋 저장
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("💾 STEP 2: 데이터셋 저장")
    print("=" * 70)
    
    # metadata를 numpy 배열로 변환 (저장 가능한 형태)
    nominal_angles = np.array([m['nominal_angle'] for m in all_metadata], dtype=np.float32)
    actual_angles = np.array([m['actual_angle'] for m in all_metadata], dtype=np.float32)
    sample_indices = np.array([m['sample_idx'] for m in all_metadata], dtype=np.int32)
    missile_labels = np.array([m['missile_type'] for m in all_metadata])
    
    save_npz_generic(DATASET_FILE, {
        'features': features,
        'labels': labels,
        'feature_names': np.array(feature_names),
        'missile_types': np.array(missile_types),
        'nominal_angles': nominal_angles,
        'actual_angles': actual_angles,
        'sample_indices': sample_indices,
        'missile_labels': missile_labels,
        # 설정 정보
        'angle_start': LAUNCH_ANGLES[0],
        'angle_end': LAUNCH_ANGLES[-1],
        'angle_step': LAUNCH_ANGLES[1] - LAUNCH_ANGLES[0] if len(LAUNCH_ANGLES) > 1 else 0,
        'samples_per_angle': SAMPLES_PER_ANGLE,
        'noise_std': NOISE_STD
    })
    
    print(f"✅ 데이터셋 저장: {DATASET_FILE}")
    
    # ---------------------------------------------------------------------
    # 3. 랜덤 Split 학습 및 평가
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("🤖 STEP 3: RandomForest 학습 (랜덤 Split)")
    print("=" * 70)
    
    # 데이터 분할
    X_train, X_test, y_train, y_test = train_test_split(
        features, labels, test_size=0.2, stratify=labels, random_state=42
    )
    
    print(f"  Train: {len(X_train)} 샘플")
    print(f"  Test: {len(X_test)} 샘플")
    
    # 정규화
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)
    
    # RandomForest 학습
    model = RandomForestClassifier(
        n_estimators=100,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    
    print("\n🎯 학습 중...")
    model.fit(X_train_scaled, y_train)
    
    # 예측
    y_pred = model.predict(X_test_scaled)
    
    # ---------------------------------------------------------------------
    # 4. 결과 출력
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊 STEP 4: 분류 결과")
    print("=" * 70)
    
    # 정확도
    accuracy = accuracy_score(y_test, y_pred)
    print(f"\n✅ 정확도 (Accuracy): {accuracy * 100:.2f}%")
    
    # Classification Report
    print("\n📋 Classification Report:")
    print("-" * 50)
    print(classification_report(y_test, y_pred, target_names=missile_types))
    
    # Confusion Matrix
    cm = confusion_matrix(y_test, y_pred)
    print("📊 Confusion Matrix:")
    print("-" * 50)
    print(f"{'':>12}", end="")
    for m in missile_types:
        print(f"{m:>10}", end="")
    print()
    for i, m in enumerate(missile_types):
        print(f"{m:>12}", end="")
        for j in range(len(missile_types)):
            print(f"{cm[i, j]:>10}", end="")
        print()
    
    # ---------------------------------------------------------------------
    # 5. Feature Importance TOP10
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("🔍 STEP 5: 중요 시그니처 TOP10")
    print("=" * 70)
    
    importances = model.feature_importances_
    feature_importance_dict = dict(zip(feature_names, importances))
    
    sorted_features = sorted(
        feature_importance_dict.items(),
        key=lambda x: x[1],
        reverse=True
    )[:10]
    
    print("\n순위  Feature Name                    Importance")
    print("-" * 55)
    for rank, (name, score) in enumerate(sorted_features, 1):
        print(f"{rank:>3}.  {name:<30}  {score:.6f}")
    
    # ---------------------------------------------------------------------
    # 6. 모델 저장
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("💾 STEP 6: 모델 저장")
    print("=" * 70)
    
    joblib.dump(model, MODEL_FILE)
    print(f"✅ 모델 저장: {MODEL_FILE}")
    
    joblib.dump(scaler, SCALER_FILE)
    print(f"✅ 스케일러 저장: {SCALER_FILE}")
    
    # ---------------------------------------------------------------------
    # 최종 요약
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("✅ 완료 요약")
    print("=" * 70)
    print(f"  총 샘플 수: {len(features)}")
    print(f"  특성 수: {len(feature_names)}")
    print(f"  랜덤 Split 정확도: {accuracy * 100:.2f}%")
    print(f"\n  저장된 파일:")
    print(f"    - {DATASET_FILE}")
    print(f"    - {MODEL_FILE}")
    print(f"    - {SCALER_FILE}")
    print(f"\n  다음 단계:")
    print(f"    python eval_by_angle.py  # 발사각 그룹 평가")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
