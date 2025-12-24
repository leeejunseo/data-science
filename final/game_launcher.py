#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Game Launcher - Missile Identification System
==============================================

Event-driven architecture:
1. Random missile selection at startup
2. Load real .npz data when triggered
3. Run signature analysis
4. Display visualization and prediction

Author: Data Science Project
"""

import numpy as np
import json
import random
import subprocess
import sys
import io
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import os
import joblib

# Import main_visualization for real-time simulation
try:
    from main_visualization import run_simulation_programmatic
    _simulation_available = True
except ImportError:
    print("⚠ main_visualization 모듈 없음. 실시간 시뮬레이션 비활성화")
    run_simulation_programmatic = None
    _simulation_available = False

# Windows 콘솔 UTF-8 인코딩 설정
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# Paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results_6dof"
SIGNATURE_DIR = SCRIPT_DIR / "signature_datasets"
MODEL_DIR = SCRIPT_DIR / "trained_models"

# =============================================================================
# ML 모델 로드
# =============================================================================
ML_MODEL = None
ML_SCALER = None
ML_FEATURE_NAMES = None
ML_MISSILE_TYPES = None

def load_ml_model():
    """Load trained ML model from eval_by_angle.py"""
    global ML_MODEL, ML_SCALER, ML_FEATURE_NAMES, ML_MISSILE_TYPES
    
    model_path = MODEL_DIR / "rf_model.pkl"
    scaler_path = MODEL_DIR / "scaler.pkl"
    features_path = MODEL_DIR / "feature_names.pkl"
    types_path = MODEL_DIR / "missile_types.pkl"
    
    print(f"\n{'='*60}")
    print(f"ML 모델 로드 시도")
    print(f"{'='*60}")
    print(f"모델 경로: {model_path}")
    print(f"모델 존재: {model_path.exists()}")
    
    if not model_path.exists():
        print(f"\n❌ ML 모델 파일 없음: {model_path}")
        print(f"   trained_models/ 디렉토리 확인:")
        if MODEL_DIR.exists():
            print(f"   디렉토리 존재: {MODEL_DIR}")
            files = list(MODEL_DIR.glob("*.pkl"))
            if files:
                print(f"   발견된 파일:")
                for f in files:
                    print(f"     - {f.name}")
            else:
                print(f"   ⚠ .pkl 파일 없음")
        else:
            print(f"   ⚠ trained_models/ 디렉토리 없음")
        print(f"\n   해결 방법:")
        print(f"   1. eval_by_angle.py 실행하여 모델 생성")
        print(f"   2. 또는 기존 모델을 trained_models/에 복사")
        return False
    
    try:
        ML_MODEL = joblib.load(model_path)
        
        # Scaler, feature names, missile types는 선택적
        if scaler_path.exists():
            ML_SCALER = joblib.load(scaler_path)
        else:
            print(f"⚠ Scaler 없음, StandardScaler 기본값 사용")
            from sklearn.preprocessing import StandardScaler
            ML_SCALER = StandardScaler()
        
        if features_path.exists():
            ML_FEATURE_NAMES = joblib.load(features_path)
        else:
            ML_FEATURE_NAMES = [f"feature_{i}" for i in range(15)]
        
        if types_path.exists():
            ML_MISSILE_TYPES = joblib.load(types_path)
        else:
            ML_MISSILE_TYPES = ["SCUD-B", "Nodong", "KN-23"]
        
        print(f"\n✅ ML 모델 로드 완료:")
        print(f"   - 모델: RandomForest")
        print(f"   - 특성: {len(ML_FEATURE_NAMES)}개")
        print(f"   - 미사일 타입: {ML_MISSILE_TYPES}")
        print(f"{'='*60}\n")
        return True
    except Exception as e:
        print(f"\n❌ ML 모델 로드 실패: {e}")
        import traceback
        traceback.print_exc()
        print(f"{'='*60}\n")
        return False

# Find Python executable (prefer venv)
def get_python_executable():
    """Get the correct Python executable (prefer virtual environment)"""
    # Check for .venv in parent directory
    venv_python = SCRIPT_DIR.parent / ".venv" / "Scripts" / "python.exe"
    if venv_python.exists():
        return str(venv_python)
    
    # Check for .venv in current directory
    venv_python2 = SCRIPT_DIR / ".venv" / "Scripts" / "python.exe"
    if venv_python2.exists():
        return str(venv_python2)
    
    # Fallback to current Python
    return sys.executable

PYTHON_EXE = get_python_executable()

# Available NPZ files for each missile type
NPZ_FILES = {
    "SCUD-B": list(RESULTS_DIR.glob("SCUD-B_*.npz")),
    "Nodong": list(RESULTS_DIR.glob("Nodong_*.npz")),
    "KN-23": list(RESULTS_DIR.glob("KN-23_*.npz")),
}

# Signature profiles for identification
SIGNATURE_PROFILES = {
    "SCUD-B": {
        "altitude_range": (80, 150),  # km
        "range_range": (200, 400),    # km
        "has_pullup": False,
        "description": "Standard ballistic trajectory, short range"
    },
    "Nodong": {
        "altitude_range": (200, 450),  # km
        "range_range": (800, 1600),    # km
        "has_pullup": False,
        "description": "High parabolic arc, medium-long range"
    },
    "KN-23": {
        "altitude_range": (30, 70),    # km (depressed trajectory!)
        "range_range": (400, 900),     # km
        "has_pullup": True,
        "description": "Quasi-ballistic, low altitude, terminal pull-up maneuver"
    }
}


class GameState:
    """Global game state"""
    hidden_missile: str = None
    npz_path: str = None
    trajectory_data: dict = None
    identification_result: dict = None
    revealed: bool = False
    launch_angle: float = None
    azimuth: float = None
    seed: int = None


def select_random_missile():
    """Randomly select a missile type and run real-time simulation"""
    # 미사일 종류 랜덤 선택
    missile_types = ["SCUD-B", "Nodong", "KN-23"]
    missile_type = random.choice(missile_types)
    
    # 고각 및 방위각 설정
    launch_angle = random.uniform(15, 80)  # 15~80도
    azimuth = 90.0                          # 90도 고정
    seed = random.randint(0, 9999)
    
    print(f"\n{'='*60}")
    print(f"🎯 [HIDDEN] Selected missile: {missile_type}")
    print(f"   Launch angle: {launch_angle:.1f}°")
    print(f"   Azimuth: {azimuth:.1f}°")
    print(f"   Seed: {seed}")
    print(f"{'='*60}")
    
    # 실시간 시뮬레이션 실행
    if _simulation_available and run_simulation_programmatic is not None:
        try:
            npz_path = run_simulation_programmatic(
                missile_type=missile_type,
                launch_angle_deg=launch_angle,
                azimuth_deg=azimuth,
                seed=seed
            )
            
            if npz_path:
                GameState.hidden_missile = missile_type
                GameState.npz_path = str(npz_path)
                GameState.revealed = False
                GameState.launch_angle = launch_angle
                GameState.azimuth = azimuth
                GameState.seed = seed
                print(f"✅ 실시간 시뮬레이션 완료: {npz_path}")
                return missile_type, str(npz_path)
            else:
                print("⚠ 시뮬레이션 실패, 기존 NPZ 파일 사용")
        except Exception as e:
            print(f"⚠ 실시간 시뮬레이션 오류: {e}")
            import traceback
            traceback.print_exc()
    
    # Fallback: 기존 NPZ 파일 사용
    print("⚠ Fallback: 기존 NPZ 파일 사용")
    available = {k: v for k, v in NPZ_FILES.items() if v}
    
    if not available:
        print("⚠ No NPZ files found in results_6dof/")
        return None, None
    
    missile_type = random.choice(list(available.keys()))
    npz_file = random.choice(available[missile_type])
    
    GameState.hidden_missile = missile_type
    GameState.npz_path = str(npz_file)
    GameState.revealed = False
    
    print(f"   NPZ file: {npz_file.name}")
    
    return missile_type, str(npz_file)


def load_npz_data(npz_path: str) -> dict:
    """Load trajectory data from NPZ file"""
    try:
        data = np.load(npz_path, allow_pickle=True)
        
        # Extract key arrays
        result = {
            'time': data['time'].tolist() if 'time' in data else [],
            'x': data['position_x'].tolist() if 'position_x' in data else [],
            'y': data['position_y'].tolist() if 'position_y' in data else [],
            'z': data['position_z'].tolist() if 'position_z' in data else [],
            'V': data['V'].tolist() if 'V' in data else [],
            'theta': data['theta'].tolist() if 'theta' in data else [],
            'gamma': data['gamma'].tolist() if 'gamma' in data else [],
            'alpha': data['alpha'].tolist() if 'alpha' in data else [],
            'mach': data['mach'].tolist() if 'mach' in data else [],
        }
        
        # Calculate key features
        if result['z']:
            result['max_altitude_km'] = max(result['z']) / 1000
        if result['x'] and result['y']:
            final_x = result['x'][-1]
            final_y = result['y'][-1]
            result['range_km'] = np.sqrt(final_x**2 + final_y**2) / 1000
        if result['time']:
            result['flight_time_s'] = result['time'][-1]
        
        GameState.trajectory_data = result
        return result
        
    except Exception as e:
        print(f"⚠ Error loading NPZ: {e}")
        return None


def extract_15_features(data: dict) -> np.ndarray:
    """
    Extract 15 features from trajectory data (12 radar + 3 6DOF)
    Same features as signature_generator.py
    """
    t = np.array(data.get('time', []))
    x = np.array(data.get('x', []))
    y = np.array(data.get('y', []))
    h = np.array(data.get('z', []))
    V = np.array(data.get('V', []))
    gamma = np.array(data.get('gamma', []))
    mach = np.array(data.get('mach', []))
    alpha = np.array(data.get('alpha', []))
    q = np.array(data.get('q', []))  # pitch rate (if available)
    
    if len(t) < 10:
        return None
    
    features = np.zeros(15, dtype=np.float32)
    
    # 1. max_altitude_km
    max_h = np.max(h)
    features[0] = max_h / 1000
    
    # 2. final_range_km
    final_range = np.sqrt(x[-1]**2 + y[-1]**2)
    features[1] = final_range / 1000
    
    # 3. impact_angle_deg
    features[2] = np.abs(np.rad2deg(gamma[-1])) if len(gamma) > 0 else 0
    
    # 4. total_flight_time
    features[3] = t[-1]
    
    # 5. max_velocity
    features[4] = np.max(V)
    
    # 6. terminal_velocity
    features[5] = V[-1]
    
    # 7. max_mach
    features[6] = np.max(mach) if len(mach) > 0 else np.max(V) / 340
    
    # 8. velocity_loss_ratio
    features[7] = (np.max(V) - V[-1]) / (np.max(V) + 1e-6)
    
    # 9. max_deceleration
    dV_dt = np.gradient(V, t)
    features[8] = np.min(dV_dt)
    
    # 10. ground_track_curvature
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = np.abs(dx*ddy - dy*ddx) / (dx**2 + dy**2 + 1e-6)**1.5
    features[9] = np.nanmean(curvature)
    
    # 11. path_efficiency
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(h)**2))
    features[10] = final_range / (path_length + 1e-6)
    
    # 12. energy_ratio (use last mass approximation)
    est_mass = 1000  # approximate
    KE = 0.5 * est_mass * V[-1]**2
    PE = est_mass * 9.81 * max_h
    features[11] = KE / (PE + 1e-6)
    
    # === 6DOF 기동성 보강 (3개) ===
    # 13. alpha_std_deg (받음각 표준편차)
    if len(alpha) > 10:
        alpha_deg = np.rad2deg(alpha)
        features[12] = np.std(alpha_deg)
    else:
        features[12] = 0.0
    
    # 14. q_max_deg_s (최대 피치율)
    if len(q) > 10:
        q_deg_s = np.rad2deg(q)
        features[13] = np.max(np.abs(q_deg_s))
    else:
        # Estimate from gamma if q not available
        if len(gamma) > 10:
            dgamma_dt = np.gradient(gamma, t)
            features[13] = np.max(np.abs(np.rad2deg(dgamma_dt)))
        else:
            features[13] = 0.0
    
    # 15. alpha_q_correlation (받음각-피치율 상관계수)
    if len(alpha) > 10 and len(q) > 10:
        alpha_deg = np.rad2deg(alpha)
        q_deg_s = np.rad2deg(q)
        if np.std(alpha_deg) > 1e-6 and np.std(q_deg_s) > 1e-6:
            features[14] = np.corrcoef(alpha_deg, q_deg_s)[0, 1]
        else:
            features[14] = 0.0
    else:
        features[14] = 0.0
    
    # NaN 처리
    features = np.nan_to_num(features, nan=0.0, posinf=0.0, neginf=0.0)
    
    return features


def analyze_signature_ml(data: dict) -> dict:
    """
    ML 기반 미사일 분류 (15개 시그니처 사용: 레이더 12 + 6DOF 3)
    """
    print(f"\n{'='*60}")
    print(f"ML 시그니처 분석 시작")
    print(f"{'='*60}")
    
    if ML_MODEL is None:
        print("❌ ML 모델 없음")
        return None
    
    if ML_SCALER is None:
        print("⚠ Scaler 없음, 정규화 없이 진행")
    
    print("15개 특징 추출 중...")
    features = extract_15_features(data)
    if features is None:
        print("❌ 특징 추출 실패")
        return None
    
    print(f"✅ 특징 추출 완료: {features[:5]}... (총 {len(features)}개)")
    
    # Scale and predict
    try:
        if ML_SCALER is not None:
            features_scaled = ML_SCALER.transform([features])
            print("✅ 특징 정규화 완료")
        else:
            features_scaled = [features]
            print("⚠ 정규화 생략")
        
        prediction = ML_MODEL.predict(features_scaled)[0]
        probabilities = ML_MODEL.predict_proba(features_scaled)[0]
        
        predicted_type = ML_MISSILE_TYPES[prediction]
        confidence = float(probabilities[prediction] * 100)
        
        # Feature importance 기반 이유 생성
        feature_dict = dict(zip(ML_FEATURE_NAMES, features))
        
        # Get feature importances from the model
        feature_importances = {}
        if hasattr(ML_MODEL, 'feature_importances_'):
            for i, importance in enumerate(ML_MODEL.feature_importances_):
                if i < len(ML_FEATURE_NAMES):
                    feature_importances[ML_FEATURE_NAMES[i]] = round(float(importance) * 100, 2)
        
        print(f"✅ ML 예측 완료:")
        print(f"   예측: {predicted_type}")
        print(f"   신뢰도: {confidence:.1f}%")
        print(f"   확률 분포: {dict(zip(ML_MISSILE_TYPES, probabilities))}")
        
        # Debug: Print top features for this prediction
        print(f"\n🔍 주요 특징 분석:")
        top_5_features = sorted(feature_importances.items(), key=lambda x: x[1], reverse=True)[:5]
        for feat_name, importance in top_5_features:
            feat_value = feature_dict.get(feat_name, 0)
            print(f"   {feat_name}: {feat_value:.3f} (중요도: {importance:.1f}%)")
        
        # Debug: Check pullup detection
        has_pullup = detect_pullup_maneuver(data)
        print(f"\n🎯 Pull-up 기동 감지: {'예' if has_pullup else '아니오'}")
        
        # Get individual tree predictions for visualization
        # Debug: Show tree voting breakdown first
        print(f"\n🗳️ 트리 투표 분석 (샘플 10개):")
        tree_predictions = []
        if hasattr(ML_MODEL, 'estimators_'):
            n_trees = min(len(ML_MODEL.estimators_), 10)  # Show first 10 trees
            for i, tree in enumerate(ML_MODEL.estimators_[:n_trees]):
                tree_pred = tree.predict(features_scaled)[0]
                tree_proba = tree.predict_proba(features_scaled)[0]
                tree_predictions.append({
                    "tree_id": i + 1,
                    "prediction": ML_MISSILE_TYPES[tree_pred],
                    "probabilities": {
                        ML_MISSILE_TYPES[j]: round(float(p) * 100, 1)
                        for j, p in enumerate(tree_proba)
                    }
                })
            
            # Print tree voting summary
            for missile in ML_MISSILE_TYPES:
                votes = sum(1 for t in tree_predictions if t['prediction'] == missile)
                print(f"   {missile}: {votes}/10 표")
        
        # Detailed feature analysis
        detailed_features = []
        for i, (name, value) in enumerate(zip(ML_FEATURE_NAMES, features)):
            scaled_value = features_scaled[0][i] if ML_SCALER is not None else value
            detailed_features.append({
                "name": name,
                "raw_value": round(float(value), 3),
                "scaled_value": round(float(scaled_value), 3),
                "importance": feature_importances.get(name, 0)
            })
        
        # Sort by importance
        detailed_features.sort(key=lambda x: x['importance'], reverse=True)
        
        # Generate detailed reasons based on top features
        top_features = sorted(feature_importances.items(), key=lambda x: x[1], reverse=True)[:5]
        reasons = []
        for feat_name, importance in top_features:
            feat_value = feature_dict.get(feat_name, 0)
            if 'altitude' in feat_name:
                reasons.append(f"{feat_name}: {feat_value:.1f}km (중요도 {importance:.1f}%)")
            elif 'range' in feat_name:
                reasons.append(f"{feat_name}: {feat_value:.1f}km (중요도 {importance:.1f}%)")
            elif 'mach' in feat_name or 'velocity' in feat_name:
                reasons.append(f"{feat_name}: {feat_value:.2f} (중요도 {importance:.1f}%)")
            elif 'angle' in feat_name:
                reasons.append(f"{feat_name}: {feat_value:.1f}° (중요도 {importance:.1f}%)")
            else:
                reasons.append(f"{feat_name}: {feat_value:.3f} (중요도 {importance:.1f}%)")
        
        # Add pullup detection info
        has_pullup = detect_pullup_maneuver(data)
        if has_pullup:
            reasons.append("⚠️ Pull-up 기동 감지 (KN-23 특징)")
        
        # Add classification hints based on probabilities
        prob_dict = dict(zip(ML_MISSILE_TYPES, probabilities))
        sorted_probs = sorted(prob_dict.items(), key=lambda x: x[1], reverse=True)
        if len(sorted_probs) >= 2:
            diff = sorted_probs[0][1] - sorted_probs[1][1]
            if diff < 0.2:  # Less than 20% difference
                reasons.append(f"⚠️ {sorted_probs[1][0]}와 유사 (차이 {diff*100:.1f}%)")
        
        result = {
            "predicted_type": predicted_type,
            "confidence": round(confidence, 1),
            "reasons": reasons,
            "features": {
                "max_altitude_km": f"{feature_dict.get('max_altitude_km', 0):.1f}",
                "range_km": f"{feature_dict.get('final_range_km', 0):.1f}",
                "flight_time_s": f"{feature_dict.get('total_flight_time', 0):.1f}",
                "has_pullup": detect_pullup_maneuver(data)
            },
            "all_probabilities": {
                ML_MISSILE_TYPES[i]: round(float(p) * 100, 1) 
                for i, p in enumerate(probabilities)
            },
            "method": "ML (RandomForest, 15 features)",
            # Detailed ML analysis data for visualization
            "ml_details": {
                "n_estimators": len(ML_MODEL.estimators_) if hasattr(ML_MODEL, 'estimators_') else 0,
                "detailed_features": detailed_features,
                "tree_predictions": tree_predictions,
                "feature_importances": feature_importances,
                "voting_summary": {
                    missile: sum(1 for t in tree_predictions if t['prediction'] == missile)
                    for missile in ML_MISSILE_TYPES
                }
            }
        }
        
        print(f"{'='*60}\n")
        return result
    except Exception as e:
        print(f"❌ ML 예측 오류: {e}")
        import traceback
        traceback.print_exc()
        print(f"{'='*60}\n")
        return None


def analyze_signature(data: dict) -> dict:
    """
    Analyze trajectory signature and predict missile type
    Uses ML model if available, otherwise falls back to rule-based
    """
    if not data:
        return {"predicted_type": "UNKNOWN", "confidence": 0, "reasons": []}
    
    # Debug: Check ML_MODEL status
    print(f"\n[DEBUG] analyze_signature 호출")
    print(f"[DEBUG] ML_MODEL is None: {ML_MODEL is None}")
    print(f"[DEBUG] ML_MODEL type: {type(ML_MODEL)}")
    
    # Try ML-based analysis first
    if ML_MODEL is not None:
        print(f"[DEBUG] ML 분석 시도...")
        ml_result = analyze_signature_ml(data)
        if ml_result is not None:
            print(f"[DEBUG] ML 분석 성공!")
            GameState.identification_result = ml_result
            return ml_result
        else:
            print(f"[DEBUG] ML 분석 실패, Rule-based로 폴백")
    else:
        print(f"[DEBUG] ML_MODEL이 None, Rule-based로 폴백")
    
    # Fallback to rule-based analysis
    print("⚠ ML 모델 없음, 규칙 기반 분류 사용")
    
    max_alt_km = data.get('max_altitude_km', 0)
    range_km = data.get('range_km', 0)
    
    # Detect pull-up maneuver (KN-23 signature)
    has_pullup = detect_pullup_maneuver(data)
    
    # Score each missile type
    scores = {}
    reasons = {}
    
    for missile_type, profile in SIGNATURE_PROFILES.items():
        score = 0
        type_reasons = []
        
        alt_min, alt_max = profile["altitude_range"]
        range_min, range_max = profile["range_range"]
        
        # Altitude check
        if alt_min <= max_alt_km <= alt_max:
            score += 35
            type_reasons.append(f"Altitude {max_alt_km:.1f}km matches {missile_type} profile ({alt_min}-{alt_max}km)")
        elif max_alt_km < alt_min and missile_type == "KN-23":
            score += 30
            type_reasons.append(f"Very low altitude ({max_alt_km:.1f}km) - depressed trajectory")
        
        # Range check
        if range_min <= range_km <= range_max:
            score += 25
            type_reasons.append(f"Range {range_km:.1f}km matches {missile_type} profile")
        
        # Pull-up detection (critical for KN-23)
        if profile["has_pullup"] and has_pullup:
            score += 35
            type_reasons.append("Terminal pull-up maneuver detected!")
        elif not profile["has_pullup"] and not has_pullup:
            score += 15
            type_reasons.append("No pull-up maneuver (standard ballistic)")
        
        # KN-23 specific: low altitude + decent range
        if missile_type == "KN-23" and max_alt_km < 70 and range_km > 400:
            score += 20
            type_reasons.append("Quasi-ballistic signature: low apogee with extended range")
        
        # Nodong specific: very high altitude
        if missile_type == "Nodong" and max_alt_km > 200:
            score += 20
            type_reasons.append("High parabolic arc detected")
        
        scores[missile_type] = score
        reasons[missile_type] = type_reasons
    
    # Find best match
    best_type = max(scores, key=scores.get)
    total_score = sum(scores.values())
    confidence = scores[best_type] / max(total_score, 1) * 100
    
    # Boost confidence for clear KN-23 signature
    if best_type == "KN-23" and max_alt_km < 60 and has_pullup:
        confidence = min(confidence + 15, 99)
    
    result = {
        "predicted_type": best_type,
        "confidence": round(confidence, 1),
        "reasons": reasons[best_type],
        "features": {
            "max_altitude_km": round(max_alt_km, 1),
            "range_km": round(range_km, 1),
            "flight_time_s": round(data.get('flight_time_s', 0), 1),
            "has_pullup": has_pullup
        },
        "all_scores": scores
    }
    
    GameState.identification_result = result
    return result


def detect_pullup_maneuver(data: dict) -> bool:
    """
    Detect terminal pull-up maneuver (KN-23 signature)
    
    Look for sudden pitch-up at 10-30km altitude during descent
    """
    z = data.get('z', [])
    gamma = data.get('gamma', [])
    alpha = data.get('alpha', [])
    
    if len(z) < 50 or len(gamma) < 50:
        return False
    
    # Find apogee
    apogee_idx = z.index(max(z))
    
    # Check terminal phase (after apogee, in 10-30km altitude range)
    for i in range(apogee_idx + 10, len(z) - 5):
        alt_km = z[i] / 1000
        
        if 10 < alt_km < 30:
            # Check gamma (flight path angle) increase
            if i > 0 and len(gamma) > i:
                gamma_change = gamma[i] - gamma[i-1]
                if gamma_change > 0.5:  # Significant positive change
                    return True
            
            # Check alpha (angle of attack) spike
            if len(alpha) > i and abs(alpha[i]) > 0.15:  # > ~8.5 degrees
                return True
    
    return False


def generate_visualization_base64(npz_path: str) -> str:
    """Generate visualization and return as base64 encoded image"""
    try:
        print(f"\n{'='*60}")
        print(f"그래프 생성 시작: {Path(npz_path).name}")
        print(f"{'='*60}")
        
        import base64
        from io import BytesIO
        
        # Set matplotlib to non-interactive backend before importing pyplot
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        # Import visualization class
        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))
        from main_visualization import MissileVisualization6DOF
        
        # Extract missile type from filename
        fname = Path(npz_path).name
        if 'SCUD' in fname or 'scud' in fname.lower():
            missile_type = "SCUD-B"
        elif 'Nodong' in fname or 'nodong' in fname.lower():
            missile_type = "Nodong"
        elif 'KN-23' in fname or 'kn23' in fname.lower() or 'kn-23' in fname.lower():
            missile_type = "KN-23"
        else:
            missile_type = "SCUD-B"
        
        print(f"미사일 타입: {missile_type}")
        
        # Create visualization
        viz = MissileVisualization6DOF(missile_type=missile_type)
        if viz.load_from_npz(npz_path):
            print("NPZ 로드 성공, 그래프 생성 중...")
            
            # Generate plot but don't show
            viz.plot_comprehensive(save_dir=str(SCRIPT_DIR / "results_6dof"))
            
            # Get current figure and save to base64
            fig = plt.gcf()
            buf = BytesIO()
            fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
            buf.seek(0)
            img_base64 = base64.b64encode(buf.read()).decode('utf-8')
            plt.close(fig)
            
            print(f"✅ 그래프 생성 완료 (크기: {len(img_base64)} bytes)")
            print(f"{'='*60}\n")
            return img_base64
        else:
            print("⚠ NPZ 로드 실패")
            print(f"{'='*60}\n")
            return None
    except Exception as e:
        print(f"\n❌ 그래프 생성 오류: {e}")
        import traceback
        traceback.print_exc()
        print(f"{'='*60}\n")
        return None


def run_visualization(npz_path: str):
    """Run main_visualization.py to show graphs"""
    try:
        # Ensure absolute paths
        npz_abs_path = str(Path(npz_path).resolve())
        main_viz_script = SCRIPT_DIR / "main_visualization.py"
        
        print(f"\n{'='*50}")
        print(f"🚀 Launching Visualization")
        print(f"   Script: {main_viz_script}")
        print(f"   NPZ: {npz_abs_path}")
        print(f"   Python: {PYTHON_EXE}")
        print(f"{'='*50}")
        
        if main_viz_script.exists():
            # Run: python main_visualization.py --file <npz_path>
            # Use CREATE_NEW_CONSOLE on Windows for visible window
            import platform
            
            if platform.system() == 'Windows':
                # Windows: create new console window
                CREATE_NEW_CONSOLE = 0x00000010
                subprocess.Popen(
                    [PYTHON_EXE, str(main_viz_script), "--file", npz_abs_path],
                    cwd=str(SCRIPT_DIR),
                    creationflags=CREATE_NEW_CONSOLE
                )
            else:
                # Unix/Mac
                subprocess.Popen(
                    [PYTHON_EXE, str(main_viz_script), "--file", npz_abs_path],
                    cwd=str(SCRIPT_DIR)
                )
            
            print(f"✓ Launched main_visualization.py for: {Path(npz_path).name}")
        else:
            # Fallback to view_npz.py
            view_script = SCRIPT_DIR / "view_npz.py"
            if view_script.exists():
                import platform
                if platform.system() == 'Windows':
                    CREATE_NEW_CONSOLE = 0x00000010
                    subprocess.Popen(
                        [PYTHON_EXE, str(view_script), npz_abs_path],
                        creationflags=CREATE_NEW_CONSOLE
                    )
                else:
                    subprocess.Popen([PYTHON_EXE, str(view_script), npz_abs_path])
                print(f"✓ Launched view_npz.py for: {Path(npz_path).name}")
            else:
                print(f"⚠ No visualization script found")
    except Exception as e:
        import traceback
        print(f"⚠ Error launching visualization: {e}")
        traceback.print_exc()


# =============================================================================
# API Server
# =============================================================================

class GameAPIHandler(BaseHTTPRequestHandler):
    """HTTP API handler for the game"""
    
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        
        response = {"error": "Unknown endpoint"}
        status = 404
        
        if path == "/api/status":
            response = {
                "status": "ready",
                "has_hidden_missile": GameState.hidden_missile is not None,
                "revealed": GameState.revealed
            }
            status = 200
            
        elif path == "/api/start":
            # Start new game - select random missile
            missile_type, npz_path = select_random_missile()
            if missile_type:
                response = {
                    "status": "started",
                    "message": "Missile selected (hidden)"
                }
                status = 200
            else:
                response = {"error": "No NPZ files available"}
                status = 500
                
        elif path == "/api/analyze":
            # Load data and run signature analysis
            print(f"\n{'='*60}")
            print(f"API /api/analyze 호출")
            print(f"{'='*60}")
            
            if not GameState.npz_path:
                print("❌ NPZ 경로 없음")
                response = {"error": "No missile selected. Call /api/start first"}
                status = 400
            else:
                print(f"NPZ 경로: {GameState.npz_path}")
                print(f"실제 미사일: {GameState.hidden_missile}")
                
                # Load NPZ data
                data = load_npz_data(GameState.npz_path)
                if not data:
                    print("❌ NPZ 데이터 로드 실패")
                    response = {"error": "Failed to load NPZ data"}
                    status = 500
                else:
                    print(f"✅ NPZ 데이터 로드 성공 (샘플 수: {len(data.get('time', []))})")
                    
                    # Run signature analysis
                    result = analyze_signature(data)
                    if not result:
                        print("❌ 시그니처 분석 실패")
                        response = {"error": "Signature analysis failed"}
                        status = 500
                    else:
                        print(f"✅ 시그니처 분석 완료")
                        print(f"   예측: {result.get('predicted_type')}")
                        print(f"   신뢰도: {result.get('confidence')}%")
                        
                        # Generate visualization as base64
                        graph_base64 = generate_visualization_base64(GameState.npz_path)
                        
                        response = {
                            "status": "analyzed",
                            "identification": result,
                            "graph_image": graph_base64,
                            "actual_missile": GameState.hidden_missile,
                            "trajectory_params": {
                                "launch_angle": round(GameState.launch_angle, 1) if GameState.launch_angle else None,
                                "azimuth": round(GameState.azimuth, 1) if GameState.azimuth else None,
                                "seed": GameState.seed
                            }
                        }
                        status = 200
                        print(f"✅ API 응답 준비 완료")
                        print(f"{'='*60}\n")
                
        elif path == "/api/visualize":
            # Launch visualization popup
            if GameState.npz_path:
                run_visualization(GameState.npz_path)
                response = {"status": "visualization_launched"}
                status = 200
            else:
                response = {"error": "No missile selected"}
                status = 400
                
        elif path == "/api/reveal":
            # Reveal the actual missile type
            GameState.revealed = True
            response = {
                "actual_type": GameState.hidden_missile,
                "npz_file": Path(GameState.npz_path).name if GameState.npz_path else None,
                "profile": SIGNATURE_PROFILES.get(GameState.hidden_missile, {})
            }
            status = 200
            
        elif path == "/api/identification":
            # Get current identification result
            if GameState.identification_result:
                response = GameState.identification_result
                status = 200
            else:
                response = {"error": "No analysis performed yet"}
                status = 404
        
        self._send_response(response, status)
    
    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        
        response = {"error": "Unknown endpoint"}
        status = 404
        
        if path == "/api/reset":
            # Reset game state
            GameState.hidden_missile = None
            GameState.npz_path = None
            GameState.trajectory_data = None
            GameState.identification_result = None
            GameState.revealed = False
            response = {"status": "reset"}
            status = 200
            
        elif path == "/api/guess":
            # Handle user guess
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                data = json.loads(body) if body else {}
                user_guess = data.get("guess", "")
                
                is_correct = user_guess == GameState.hidden_missile
                GameState.revealed = True
                
                response = {
                    "user_guess": user_guess,
                    "actual_type": GameState.hidden_missile,
                    "is_correct": is_correct,
                    "identification": GameState.identification_result
                }
                status = 200
            except json.JSONDecodeError:
                response = {"error": "Invalid JSON"}
                status = 400
        
        self._send_response(response, status)
    
    def _send_response(self, data: dict, status: int = 200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        # Enable logging for debugging
        print(f"[HTTP] {format % args}")


def run_server(port: int = 5000):
    """Run the API server"""
    print(f"\n{'='*60}")
    print(f"🚀 게임 서버 시작")
    print(f"{'='*60}\n")
    
    # Load ML model at startup
    print("1. ML 모델 로딩 중...")
    ml_loaded = load_ml_model()
    
    if ml_loaded:
        print(f"\n✅ ML 모델 사용 가능 - ML 기반 분석 활성화")
        print(f"   Global ML_MODEL: {ML_MODEL is not None}")
    else:
        print(f"\n⚠️  ML 모델 로드 실패 - Rule-based 분석으로 폴백")
    
    # Auto-select missile at startup
    print(f"\n2. 초기 미사일 선택 중...")
    select_random_missile()
    
    server = HTTPServer(('localhost', port), GameAPIHandler)
    print(f"\n{'='*60}")
    print(f"✅ 서버 준비 완료")
    print(f"{'='*60}")
    print(f"\n🌐 Server URL: http://localhost:{port}")
    print(f"\n📡 API Endpoints:")
    print(f"  GET  /api/status      - Server status")
    print(f"  GET  /api/start       - Start new game (random missile)")
    print(f"  GET  /api/analyze     - Run signature analysis")
    print(f"  GET  /api/visualize   - Launch graph popup")
    print(f"  GET  /api/reveal      - Reveal actual missile")
    print(f"  GET  /api/identification - Get analysis result")
    print(f"  POST /api/guess       - Submit user guess")
    print(f"  POST /api/reset       - Reset game")
    print(f"\n⌨️  Press Ctrl+C to stop")
    print(f"{'='*60}\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nServer stopped.")
        server.shutdown()


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Missile Identification Game Server')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--demo', action='store_true', help='Run demo analysis')
    
    args = parser.parse_args()
    
    if args.demo:
        # Demo mode
        print("\n" + "="*60)
        print("🎯 Missile Identification Demo")
        print("="*60)
        
        # Load ML model
        load_ml_model()
        
        missile_type, npz_path = select_random_missile()
        if npz_path:
            data = load_npz_data(npz_path)
            result = analyze_signature(data)
            
            print(f"\n📊 SIGNATURE ANALYSIS:")
            print(f"  Predicted: {result['predicted_type']}")
            print(f"  Confidence: {result['confidence']}%")
            print(f"\n  Features:")
            for k, v in result['features'].items():
                print(f"    • {k}: {v}")
            print(f"\n  Reasons:")
            for reason in result['reasons']:
                print(f"    • {reason}")
            
            print(f"\n🎯 ACTUAL: {GameState.hidden_missile}")
            correct = result['predicted_type'] == GameState.hidden_missile
            print(f"  Result: {'✓ CORRECT' if correct else '✗ INCORRECT'}")
    else:
        run_server(args.port)
