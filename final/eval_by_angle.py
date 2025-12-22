#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
발사각 그룹 단위 평가 (GroupShuffleSplit)
==========================================

목적:
- train_large.py가 저장한 dataset_large.npz 로드
- nominal_angle 그룹 단위로 train/test 분리 (각도 누수 방지)
- RandomForest 학습 및 평가
- 혼동행렬/분류리포트/정확도 출력
- feature_importances_ 기반 중요 시그니처 TOP10 출력
- 테스트에 사용된 nominal_angle 목록 출력

실행:
    python eval_by_angle.py

입력:
    - signature_dataset/dataset_large.npz (train_large.py가 생성)

출력:
    - 콘솔에 평가 결과 출력
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
from sklearn.model_selection import GroupShuffleSplit
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
import joblib


# =============================================================================
# 설정
# =============================================================================

DATASET_FILE = Path("signature_dataset") / "dataset_large.npz"


# =============================================================================
# 메인 함수
# =============================================================================

def main():
    print("\n" + "=" * 70)
    print("🎯 발사각 그룹 단위 평가 (GroupShuffleSplit)")
    print("=" * 70)
    
    # ---------------------------------------------------------------------
    # 1. 데이터셋 로드
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📂 STEP 1: 데이터셋 로드")
    print("=" * 70)
    
    if not DATASET_FILE.exists():
        print(f"❌ 데이터셋 파일이 없습니다: {DATASET_FILE}")
        print("   먼저 train_large.py를 실행하세요.")
        return 1
    
    data = np.load(DATASET_FILE, allow_pickle=True)
    
    features = data['features']
    labels = data['labels']
    feature_names = list(data['feature_names'])
    missile_types = list(data['missile_types'])
    nominal_angles = data['nominal_angles']
    
    print(f"✅ 데이터셋 로드 완료:")
    print(f"   X.shape: {features.shape}")
    print(f"   총 샘플 수: {len(features)}")
    print(f"   특성 수: {len(feature_names)}")
    print(f"   미사일 타입: {missile_types}")
    print(f"   발사각 범위: {nominal_angles.min():.0f}°~{nominal_angles.max():.0f}°")
    print(f"   고유 발사각 수: {len(np.unique(nominal_angles))}")
    
    # ---------------------------------------------------------------------
    # 2. GroupShuffleSplit으로 데이터 분할
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊 STEP 2: GroupShuffleSplit (발사각 그룹 단위 분할)")
    print("=" * 70)
    
    # groups = nominal_angle
    groups = nominal_angles.astype(int)
    
    gss = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=42)
    
    for train_idx, test_idx in gss.split(features, labels, groups):
        X_train, X_test = features[train_idx], features[test_idx]
        y_train, y_test = labels[train_idx], labels[test_idx]
        groups_train = groups[train_idx]
        groups_test = groups[test_idx]
    
    # 테스트에 사용된 발사각
    test_angles = np.unique(groups_test)
    train_angles = np.unique(groups_train)
    
    print(f"  Train: {len(X_train)} 샘플, {len(train_angles)} 발사각 그룹")
    print(f"  Test: {len(X_test)} 샘플, {len(test_angles)} 발사각 그룹")
    
    print(f"\n📋 테스트에 사용된 nominal_angle 목록:")
    print(f"   {sorted(test_angles.tolist())}°")
    
    print(f"\n📋 훈련에 사용된 nominal_angle 목록:")
    print(f"   {sorted(train_angles.tolist())}°")
    
    # 발사각 겹침 확인
    overlap = set(train_angles) & set(test_angles)
    if overlap:
        print(f"\n⚠️ 경고: train/test 발사각 겹침 발생: {overlap}")
    else:
        print(f"\n✅ train/test 발사각 완전 분리 확인됨 (누수 없음)")
    
    # ---------------------------------------------------------------------
    # 3. 정규화 및 학습
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("🤖 STEP 3: RandomForest 학습")
    print("=" * 70)
    
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
    
    print("🎯 학습 중...")
    model.fit(X_train_scaled, y_train)
    
    # 예측
    y_pred = model.predict(X_test_scaled)
    
    # ---------------------------------------------------------------------
    # 4. 결과 출력
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊 STEP 4: 분류 결과 (발사각 그룹 평가)")
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
    print("🔍 STEP 5: 중요 시그니처 TOP10 (발사각 그룹 평가)")
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
    # 6. 미사일 타입별 성능 분석
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📈 STEP 6: 미사일 타입별 성능 분석")
    print("=" * 70)
    
    for i, m_type in enumerate(missile_types):
        mask = y_test == i
        if mask.sum() > 0:
            type_acc = (y_pred[mask] == y_test[mask]).mean() * 100
            print(f"  {m_type}: {type_acc:.1f}% ({mask.sum()} 샘플)")
    
    # ---------------------------------------------------------------------
    # 최종 요약
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("✅ 발사각 그룹 평가 완료")
    print("=" * 70)
    print(f"""
  📊 평가 설정:
     - GroupShuffleSplit (test_size=0.2)
     - 그룹 기준: nominal_angle
     - train/test 발사각 겹침: {'있음 ⚠️' if overlap else '없음 ✅'}
  
  📋 테스트 발사각:
     {sorted(test_angles.tolist())}
  
  🎯 결과:
     - 발사각 그룹 평가 정확도: {accuracy * 100:.2f}%
  
  💡 해석:
     - 이 평가는 "학습하지 않은 발사각"에서의 일반화 성능을 측정합니다.
     - 랜덤 Split 평가보다 낮은 정확도가 나올 수 있으며, 
       이는 모델이 특정 발사각에 과적합되지 않았음을 의미합니다.
""")
    
    # ---------------------------------------------------------------------
    # 7. 모델 저장 (게임 연동용)
    # ---------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("💾 STEP 7: 모델 저장")
    print("=" * 70)
    
    model_dir = Path("trained_models")
    model_dir.mkdir(exist_ok=True)
    
    # 전체 데이터로 재학습 (게임용 최종 모델)
    print("🎯 전체 데이터로 최종 모델 학습 중...")
    scaler_final = StandardScaler()
    X_all_scaled = scaler_final.fit_transform(features)
    
    model_final = RandomForestClassifier(
        n_estimators=100,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    model_final.fit(X_all_scaled, labels)
    
    # 저장
    joblib.dump(model_final, model_dir / "rf_model.pkl")
    joblib.dump(scaler_final, model_dir / "scaler.pkl")
    joblib.dump(feature_names, model_dir / "feature_names.pkl")
    joblib.dump(missile_types, model_dir / "missile_types.pkl")
    
    print(f"✅ 모델 저장 완료:")
    print(f"   - {model_dir / 'rf_model.pkl'}")
    print(f"   - {model_dir / 'scaler.pkl'}")
    print(f"   - {model_dir / 'feature_names.pkl'}")
    print(f"   - {model_dir / 'missile_types.pkl'}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
