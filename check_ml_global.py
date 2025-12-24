#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check if ML model is actually loaded in game_launcher"""

import sys
from pathlib import Path

# Add final directory to path
final_dir = Path(__file__).parent / "final"
sys.path.insert(0, str(final_dir))

# Import game_launcher module
import game_launcher

print("=" * 60)
print("ML 모델 전역 변수 확인")
print("=" * 60)

print(f"\n1. 초기 상태:")
print(f"   ML_MODEL: {game_launcher.ML_MODEL}")
print(f"   ML_SCALER: {game_launcher.ML_SCALER}")
print(f"   ML_FEATURE_NAMES: {game_launcher.ML_FEATURE_NAMES}")
print(f"   ML_MISSILE_TYPES: {game_launcher.ML_MISSILE_TYPES}")

print(f"\n2. load_ml_model() 호출...")
result = game_launcher.load_ml_model()
print(f"   반환값: {result}")

print(f"\n3. 로딩 후 상태:")
print(f"   ML_MODEL: {game_launcher.ML_MODEL is not None}")
print(f"   ML_SCALER: {game_launcher.ML_SCALER is not None}")
print(f"   ML_FEATURE_NAMES: {len(game_launcher.ML_FEATURE_NAMES) if game_launcher.ML_FEATURE_NAMES else 0}")
print(f"   ML_MISSILE_TYPES: {game_launcher.ML_MISSILE_TYPES}")

if game_launcher.ML_MODEL:
    print(f"\n4. ML 모델 정보:")
    print(f"   타입: {type(game_launcher.ML_MODEL).__name__}")
    if hasattr(game_launcher.ML_MODEL, 'n_estimators'):
        print(f"   트리 개수: {game_launcher.ML_MODEL.n_estimators}")
    if hasattr(game_launcher.ML_MODEL, 'feature_importances_'):
        print(f"   특징 개수: {len(game_launcher.ML_MODEL.feature_importances_)}")

print("\n" + "=" * 60)
