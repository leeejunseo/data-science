#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Test ML model loading"""

import sys
from pathlib import Path
import joblib

# Add final directory to path
final_dir = Path(__file__).parent / "final"
sys.path.insert(0, str(final_dir))

MODEL_DIR = final_dir / "trained_models"

print("=" * 60)
print("ML 모델 로딩 테스트")
print("=" * 60)

# Check if directory exists
print(f"\n1. 디렉토리 확인:")
print(f"   경로: {MODEL_DIR}")
print(f"   존재: {MODEL_DIR.exists()}")

if MODEL_DIR.exists():
    files = list(MODEL_DIR.glob("*.pkl"))
    print(f"   파일 개수: {len(files)}")
    for f in files:
        print(f"     - {f.name} ({f.stat().st_size} bytes)")

# Try to load each file
print(f"\n2. 모델 파일 로딩 테스트:")

model_path = MODEL_DIR / "rf_model.pkl"
scaler_path = MODEL_DIR / "scaler.pkl"
features_path = MODEL_DIR / "feature_names.pkl"
types_path = MODEL_DIR / "missile_types.pkl"

try:
    print(f"\n   rf_model.pkl 로딩...")
    model = joblib.load(model_path)
    print(f"   ✅ 성공!")
    print(f"      타입: {type(model).__name__}")
    if hasattr(model, 'n_estimators'):
        print(f"      트리 개수: {model.n_estimators}")
    if hasattr(model, 'feature_importances_'):
        print(f"      특징 개수: {len(model.feature_importances_)}")
except Exception as e:
    print(f"   ❌ 실패: {e}")
    import traceback
    traceback.print_exc()

try:
    print(f"\n   scaler.pkl 로딩...")
    scaler = joblib.load(scaler_path)
    print(f"   ✅ 성공!")
    print(f"      타입: {type(scaler).__name__}")
except Exception as e:
    print(f"   ❌ 실패: {e}")

try:
    print(f"\n   feature_names.pkl 로딩...")
    features = joblib.load(features_path)
    print(f"   ✅ 성공!")
    print(f"      특징 개수: {len(features)}")
    print(f"      특징 목록: {features[:5]}...")
except Exception as e:
    print(f"   ❌ 실패: {e}")

try:
    print(f"\n   missile_types.pkl 로딩...")
    types = joblib.load(types_path)
    print(f"   ✅ 성공!")
    print(f"      미사일 타입: {types}")
except Exception as e:
    print(f"   ❌ 실패: {e}")

print("\n" + "=" * 60)
print("테스트 완료")
print("=" * 60)
