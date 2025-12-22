#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6DOF 미사일 시그니처 데이터 생성기
- final/missile_6dof_true.py 기반 True 6DOF 시뮬레이션 (Quaternion)
- 탄종별 고유 시그니처 특성 추출
- 분류 모델 학습용 데이터셋 생성

핵심 시그니처 특성 (6DOF 고유):
1. 받음각(alpha) 프로파일 - 탄종별 고유 패턴
2. 피치 각속도(q) 동특성 - 공력 댐핑 특성
3. α-q 위상 평면 - 동적 안정성 시그니처
4. Roll-Yaw Coupling (p-r) - 관성 특성
5. 속도/고도 프로파일 - 추진 특성
"""

import numpy as np
import os
import sys
import io
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

# Windows 콘솔 UTF-8 인코딩 설정
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# 현재 디렉토리를 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from missile_6dof_true import True6DOFSimulator
    import config_6dof as cfg
    from trajectory_io import save_npz_generic
    print("[OK] missile_6dof_true, config_6dof, trajectory_io load success")
except ImportError as e:
    print(f"[FAIL] Module load failed: {e}")
    sys.exit(1)


class MissileSignatureGenerator:
    """
    True 6DOF 기반 미사일 시그니처 데이터 생성기 (Quaternion)
    
    탄종 분류를 위한 핵심 시그니처:
    1. 기하학적 특성: 권적 형상, 최대고도/사거리 비율
    2. 동역학적 특성: 받음각, 각속도, 위상 평면
    3. 추진 특성: 가속도 프로파일, 연소 종료 시점
    4. 공력 특성: 양항비, 항력 프로파일
    """
    
    # 지원 미사일 타입
    SUPPORTED_MISSILES = ["SCUD-B", "Nodong", "KN-23"]
    
    # 시그니처 특성 정의 (15차원)
    # ================================================================
    # 레이더 관측 기반 시그니처 (12차원) + 6DOF 기동성 보강 (3차원)
    # - 설계상수/내부정보 제외 (burnout_velocity, burn_time_ratio,
    #   thrust_to_weight_initial, angular_momentum_ratio,
    #   dynamic_stability_index, ballistic_coefficient, glide_ratio,
    #   reentry_heating_index 등)
    # - 레이더로 관측 가능한 궤적/속도/기동 특성 사용
    # - 6DOF 기동성 지표 최소 추가 (alpha_std, q_max, alpha_q_corr)
    # ================================================================
    SIGNATURE_FEATURES = [
        # 궤적 형태 (4개) - 레이더 추적으로 관측 가능
        'max_altitude_km',           # 최대 고도
        'final_range_km',            # 최종 사거리
        'impact_angle_deg',          # 낙하각
        'total_flight_time',         # 총 비행시간
        
        # 속도/마하 (4개) - 레이더 도플러로 관측 가능
        'max_velocity',              # 최대 속도
        'terminal_velocity',         # 종말 속도
        'max_mach',                  # 최대 마하수
        'velocity_loss_ratio',       # 속도 손실률
        
        # 감속/기동/효율 (4개) - 궤적 변화로 추정 가능
        'max_deceleration',          # 최대 감속도
        'ground_track_curvature',    # 지상 궤적 곡률
        'path_efficiency',           # 경로 효율성
        'energy_ratio',              # 에너지 비율 (관측 가능)
        
        # 6DOF 기동성 보강 (3개) - 탄도 불안정성/기동 특성
        'alpha_std_deg',             # 받음각 표준편차 (기동 강도)
        'q_max_deg_s',               # 최대 피치율 (급기동 지표)
        'alpha_q_correlation',       # 받음각-피치율 상관계수 (동적 안정성)
    ]
    
    def __init__(self, output_dir: str = "signature_dataset"):
        """
        초기화
        
        Args:
            output_dir: 데이터셋 저장 디렉토리
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 시뮬레이션 파라미터
        self.launch_angles = list(range(25, 71, 5))  # 25°~70°, 5° 간격
        self.azimuth_angles = [90]  # 방위각 고정 (단순화)
        
        # 통계
        self.generation_stats = {
            'total_attempts': 0,
            'successful': 0,
            'failed': 0,
            'by_missile': {}
        }
        
        print(f"\n{'='*60}")
        print("6DOF 미사일 시그니처 생성기 초기화")
        print(f"{'='*60}")
        print(f"출력 디렉토리: {self.output_dir}")
        print(f"발사각 범위: {self.launch_angles[0]}°~{self.launch_angles[-1]}° ({len(self.launch_angles)}개)")
        print(f"시그니처 특성: {len(self.SIGNATURE_FEATURES)}차원")
        print(f"지원 미사일: {self.SUPPORTED_MISSILES}")
    
    def generate_dataset(
        self, 
        missile_types: List[str] = None,
        samples_per_angle: int = 3,
        noise_std: float = 0.5
    ) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """
        시그니처 데이터셋 생성
        
        Args:
            missile_types: 미사일 타입 리스트 (None이면 전체)
            samples_per_angle: 각 발사각당 샘플 수
            noise_std: 발사각 노이즈 표준편차 (도)
        
        Returns:
            features: 시그니처 특성 배열 [N, 15]
            labels: 미사일 타입 레이블 [N]
            metadata: 메타데이터 딕셔너리
        """
        if missile_types is None:
            missile_types = self.SUPPORTED_MISSILES
        
        print(f"\n🚀 시그니처 데이터셋 생성 시작")
        print(f"   미사일: {missile_types}")
        print(f"   발사각당 샘플: {samples_per_angle}")
        
        all_features = []
        all_labels = []
        all_trajectories = []
        all_metadata = []
        
        missile_to_idx = {m: i for i, m in enumerate(missile_types)}
        
        for missile_type in missile_types:
            print(f"\n📍 {missile_type} 시뮬레이션...")
            self.generation_stats['by_missile'][missile_type] = {
                'attempts': 0, 'success': 0
            }
            
            for launch_angle in self.launch_angles:
                for sample_idx in range(samples_per_angle):
                    self.generation_stats['total_attempts'] += 1
                    self.generation_stats['by_missile'][missile_type]['attempts'] += 1
                    
                    # 발사각에 노이즈 추가
                    actual_angle = launch_angle + np.random.normal(0, noise_std)
                    actual_angle = np.clip(actual_angle, 20, 75)
                    
                    try:
                        # True 6DOF 시뮬레이션 실행
                        sim = True6DOFSimulator(missile_type=missile_type)
                        results = sim.simulate(
                            elevation_deg=actual_angle,
                            azimuth_deg=90
                        )
                        
                        # 시그니처 특성 추출
                        features = self._extract_signature_features(
                            results, missile_type, actual_angle
                        )
                        
                        if features is not None:
                            all_features.append(features)
                            all_labels.append(missile_to_idx[missile_type])
                            all_trajectories.append(self._compress_trajectory(results))
                            all_metadata.append({
                                'missile_type': missile_type,
                                'nominal_angle': launch_angle,
                                'actual_angle': actual_angle,
                                'sample_idx': sample_idx
                            })
                            
                            self.generation_stats['successful'] += 1
                            self.generation_stats['by_missile'][missile_type]['success'] += 1
                        else:
                            self.generation_stats['failed'] += 1
                            
                    except Exception as e:
                        self.generation_stats['failed'] += 1
                        continue
                
                # 진행 상황 출력
                success = self.generation_stats['by_missile'][missile_type]['success']
                attempts = self.generation_stats['by_missile'][missile_type]['attempts']
                print(f"   {launch_angle}°: {success}/{attempts} 성공", end="\r")
            
            print(f"\n   ✓ {missile_type} 완료: {self.generation_stats['by_missile'][missile_type]['success']} 샘플")
        
        # 배열로 변환
        features_array = np.array(all_features, dtype=np.float32)
        labels_array = np.array(all_labels, dtype=np.int32)
        
        # 데이터셋 저장
        self._save_dataset(
            features_array, labels_array, 
            all_trajectories, all_metadata,
            missile_types
        )
        
        # 통계 출력
        self._print_statistics(missile_types)
        
        return features_array, labels_array, {
            'missile_types': missile_types,
            'feature_names': self.SIGNATURE_FEATURES,
            'trajectories': all_trajectories,
            'metadata': all_metadata
        }
    
    def _extract_signature_features(
        self, 
        results: Dict, 
        missile_type: str,
        launch_angle: float
    ) -> Optional[np.ndarray]:
        """
        6DOF 시뮬레이션 결과에서 시그니처 특성 추출
        
        Args:
            results: 시뮬레이션 결과 딕셔너리
            missile_type: 미사일 타입
            launch_angle: 발사각
        
        Returns:
            features: 15차원 시그니처 벡터 (레이더 12 + 6DOF 기동성 3)
        """
        try:
            t = results['time']
            
            # True6DOF 결과 키 이름 변환
            # True6DOF는 Body Frame (u,v,w)를 반환
            u = results['u']
            v = results['v']
            w = results['w']
            V = results['V']
            
            x = results['x']
            y = results['y']
            h = results['z']  # altitude는 'z'로 반환
            
            phi = results['phi']
            theta = results['theta']
            psi = results['psi']
            
            p = results['p']
            q = results['q']
            r = results['r']
            
            # alpha, gamma, mach 계산 (True6DOF는 제공 안 함)
            alpha = np.arctan2(w, np.maximum(np.abs(u), 0.1))
            gamma = np.arcsin(np.clip(w / np.maximum(V, 0.1), -1, 1))
            
            # mach 계산
            T = 288.15 - 0.0065 * h
            T[h > 11000] = 216.65
            a = np.sqrt(1.4 * 287.05 * T)
            mach = V / a
            
            # mass 추정 (없을 경우)
            if 'mass' not in results:
                missile_info = cfg.MISSILE_TYPES.get(missile_type, {})
                initial_mass = missile_info.get('launch_weight', 5860)
                propellant_mass = missile_info.get('propellant_mass', 4875)
                burn_time = missile_info.get('burn_time', 65)
                dry_mass = initial_mass - propellant_mass
                
                mass = np.zeros_like(t)
                for i, ti in enumerate(t):
                    if ti < burn_time:
                        mass[i] = initial_mass - propellant_mass * ti / burn_time
                    else:
                        mass[i] = dry_mass
            else:
                mass = results['mass']
            
            # 데이터 검증
            if len(t) < 50 or np.any(np.isnan(V)) or np.any(np.isinf(h)):
                return None
            
            # 미사일 정보
            missile_info = cfg.MISSILE_TYPES.get(missile_type, {})
            burn_time = missile_info.get('burn_time', 65)
            initial_mass = missile_info.get('launch_weight', 5860)
            
            features = np.zeros(len(self.SIGNATURE_FEATURES), dtype=np.float32)
            
            # === 궤적 형태 (4개) - 레이더 추적으로 관측 가능 ===
            max_h = np.max(h)
            final_range = np.sqrt(x[-1]**2 + y[-1]**2)
            apogee_idx = np.argmax(h)
            
            features[0] = max_h / 1000  # max_altitude_km
            features[1] = final_range / 1000  # final_range_km
            features[2] = np.abs(np.rad2deg(gamma[-1]))  # impact_angle_deg (낙하각)
            features[3] = t[-1]  # total_flight_time
            
            # === 속도/마하 (4개) - 레이더 도플러로 관측 가능 ===
            features[4] = np.max(V)  # max_velocity
            features[5] = V[-1]  # terminal_velocity
            features[6] = np.max(mach)  # max_mach
            features[7] = (np.max(V) - V[-1]) / (np.max(V) + 1e-6)  # velocity_loss_ratio
            
            # === 감속/기동/효율 (4개) - 궤적 변화로 추정 가능 ===
            dV_dt = np.gradient(V, t)
            features[8] = np.min(dV_dt)  # max_deceleration
            
            # 지상 궤적 곡률
            dx = np.gradient(x)
            dy = np.gradient(y)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            curvature = np.abs(dx*ddy - dy*ddx) / (dx**2 + dy**2 + 1e-6)**1.5
            features[9] = np.mean(curvature[~np.isnan(curvature)])  # ground_track_curvature
            
            # 경로 효율성
            path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(h)**2))
            features[10] = final_range / (path_length + 1e-6)  # path_efficiency
            
            # 에너지 비율 (운동에너지 / 위치에너지) - 레이더 관측으로 추정 가능
            KE = 0.5 * mass[-1] * V[-1]**2
            PE = mass[-1] * 9.81 * max_h
            features[11] = KE / (PE + 1e-6)  # energy_ratio
            
            # === 6DOF 기동성 보강 (3개) ===
            # 받음각 표준편차 (기동 강도 지표)
            alpha_deg = np.rad2deg(alpha)
            features[12] = np.std(alpha_deg)  # alpha_std_deg
            
            # 최대 피치율 (급기동 지표)
            q_deg_s = np.rad2deg(q)  # q는 이미 results에서 가져옴
            features[13] = np.max(np.abs(q_deg_s))  # q_max_deg_s
            
            # 받음각-피치율 상관계수 (동적 안정성)
            if len(alpha_deg) > 10 and np.std(alpha_deg) > 1e-6 and np.std(q_deg_s) > 1e-6:
                features[14] = np.corrcoef(alpha_deg, q_deg_s)[0, 1]  # alpha_q_correlation
            else:
                features[14] = 0.0
            
            # NaN 처리
            features = np.nan_to_num(features, nan=0.0, posinf=0.0, neginf=0.0)
            
            return features
            
        except Exception as e:
            return None
    
    def _compress_trajectory(self, results: Dict, max_points: int = 200) -> Dict:
        """궤적 데이터 압축 저장"""
        t = results['time']
        n = len(t)
        
        if n > max_points:
            indices = np.linspace(0, n-1, max_points, dtype=int)
        else:
            indices = np.arange(n)
        
        # True6DOF 결과 키 변환
        u = results['u']
        v = results['v']
        w = results['w']
        V = results['V']
        
        # alpha, gamma 계산
        alpha = np.arctan2(w, np.maximum(np.abs(u), 0.1))
        gamma = np.arcsin(np.clip(w / np.maximum(V, 0.1), -1, 1))
        
        # mach 계산
        h_full = results['z']
        T = 288.15 - 0.0065 * h_full
        T[h_full > 11000] = 216.65
        a = np.sqrt(1.4 * 287.05 * T)
        mach = V / a
        
        return {
            'time': t[indices].astype(np.float32),
            'V': V[indices].astype(np.float32),
            'h': results['z'][indices].astype(np.float32),
            'x': results['x'][indices].astype(np.float32),
            'y': results['y'][indices].astype(np.float32),
            'alpha': alpha[indices].astype(np.float32),
            'q': results['q'][indices].astype(np.float32),
            'gamma': gamma[indices].astype(np.float32),
            'mach': mach[indices].astype(np.float32)
        }
    
    def _save_dataset(
        self,
        features: np.ndarray,
        labels: np.ndarray,
        trajectories: List[Dict],
        metadata: List[Dict],
        missile_types: List[str]
    ):
        """데이터셋 저장"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 1. 시그니처 특성 저장
        features_file = self.output_dir / f"signature_features_{timestamp}.npz"
        save_npz_generic(features_file, {
            'features': features,
            'labels': labels,
            'feature_names': self.SIGNATURE_FEATURES,
            'missile_types': missile_types,
            'n_samples': len(features),
            'n_features': len(self.SIGNATURE_FEATURES)
        })
        
        # 2. 궤적 데이터 저장
        trajectory_file = self.output_dir / f"trajectories_{timestamp}.npz"
        traj_dict = {f'traj_{i}': traj for i, traj in enumerate(trajectories)}
        save_npz_generic(trajectory_file, traj_dict)
        
        # 3. 메타데이터 저장
        meta_file = self.output_dir / f"metadata_{timestamp}.npz"
        save_npz_generic(meta_file, {
            'metadata': metadata,
            'generation_stats': self.generation_stats,
            'launch_angles': self.launch_angles,
            'azimuth_angles': self.azimuth_angles
        })
        
        print(f"\n💾 데이터셋 저장 완료:")
        print(f"   시그니처: {features_file}")
        print(f"   궤적: {trajectory_file}")
        print(f"   메타데이터: {meta_file}")
    
    def _print_statistics(self, missile_types: List[str]):
        """통계 출력"""
        print(f"\n{'='*60}")
        print("📊 생성 통계")
        print(f"{'='*60}")
        
        total = self.generation_stats['total_attempts']
        success = self.generation_stats['successful']
        print(f"총 시도: {total}")
        print(f"성공: {success} ({100*success/total:.1f}%)")
        print(f"실패: {self.generation_stats['failed']}")
        
        print(f"\n미사일별 통계:")
        for m_type in missile_types:
            stats = self.generation_stats['by_missile'].get(m_type, {})
            s = stats.get('success', 0)
            a = stats.get('attempts', 0)
            rate = 100*s/a if a > 0 else 0
            print(f"  {m_type}: {s}/{a} ({rate:.1f}%)")


class MissileClassifier:
    """
    시그니처 기반 미사일 분류기
    
    지원 모델:
    1. Random Forest (기본)
    2. Gradient Boosting
    3. SVM (RBF kernel)
    4. Neural Network (MLP)
    """
    
    def __init__(self, model_type: str = 'random_forest'):
        """
        Args:
            model_type: 'random_forest', 'gradient_boosting', 'svm', 'mlp'
        """
        self.model_type = model_type
        self.model = None
        self.scaler = None
        self.feature_names = None
        self.missile_types = None
        
    def train(
        self, 
        features: np.ndarray, 
        labels: np.ndarray,
        feature_names: List[str] = None,
        missile_types: List[str] = None,
        test_size: float = 0.2
    ) -> Dict:
        """
        분류기 학습
        
        Returns:
            metrics: 학습 결과 메트릭
        """
        try:
            from sklearn.model_selection import train_test_split
            from sklearn.preprocessing import StandardScaler
            from sklearn.metrics import classification_report, confusion_matrix
        except ImportError:
            print("⚠ scikit-learn이 필요합니다: pip install scikit-learn")
            return {}
        
        self.feature_names = feature_names
        self.missile_types = missile_types
        
        # 데이터 분할
        X_train, X_test, y_train, y_test = train_test_split(
            features, labels, test_size=test_size, stratify=labels, random_state=42
        )
        
        # 정규화
        self.scaler = StandardScaler()
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        # 모델 선택 및 학습
        if self.model_type == 'random_forest':
            from sklearn.ensemble import RandomForestClassifier
            self.model = RandomForestClassifier(
                n_estimators=100, max_depth=10, random_state=42
            )
        elif self.model_type == 'gradient_boosting':
            from sklearn.ensemble import GradientBoostingClassifier
            self.model = GradientBoostingClassifier(
                n_estimators=100, max_depth=5, random_state=42
            )
        elif self.model_type == 'svm':
            from sklearn.svm import SVC
            self.model = SVC(kernel='rbf', C=1.0, probability=True, random_state=42)
        elif self.model_type == 'mlp':
            from sklearn.neural_network import MLPClassifier
            self.model = MLPClassifier(
                hidden_layer_sizes=(64, 32), max_iter=500, random_state=42
            )
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
        
        print(f"\n🎯 {self.model_type} 분류기 학습 중...")
        self.model.fit(X_train_scaled, y_train)
        
        # 평가
        y_pred = self.model.predict(X_test_scaled)
        
        print(f"\n📊 분류 결과:")
        print(classification_report(
            y_test, y_pred, 
            target_names=missile_types if missile_types else None
        ))
        
        # 혼동 행렬
        cm = confusion_matrix(y_test, y_pred)
        
        # 특성 중요도 (Random Forest의 경우)
        feature_importance = None
        if hasattr(self.model, 'feature_importances_'):
            feature_importance = dict(zip(
                feature_names if feature_names else range(features.shape[1]),
                self.model.feature_importances_
            ))
            
            print(f"\n🔍 주요 시그니처 특성 (상위 10개):")
            sorted_features = sorted(
                feature_importance.items(), 
                key=lambda x: x[1], 
                reverse=True
            )[:10]
            for name, importance in sorted_features:
                print(f"   {name}: {importance:.4f}")
        
        return {
            'accuracy': self.model.score(X_test_scaled, y_test),
            'confusion_matrix': cm,
            'feature_importance': feature_importance
        }
    
    def predict(self, features: np.ndarray) -> np.ndarray:
        """예측"""
        if self.model is None:
            raise ValueError("모델이 학습되지 않았습니다")
        
        features_scaled = self.scaler.transform(features)
        return self.model.predict(features_scaled)
    
    def predict_proba(self, features: np.ndarray) -> np.ndarray:
        """확률 예측"""
        if self.model is None:
            raise ValueError("모델이 학습되지 않았습니다")
        
        features_scaled = self.scaler.transform(features)
        return self.model.predict_proba(features_scaled)


def visualize_signatures(
    features: np.ndarray,
    labels: np.ndarray,
    feature_names: List[str],
    missile_types: List[str],
    output_dir: str = "signature_dataset"
):
    """시그니처 분포 시각화"""
    try:
        import matplotlib.pyplot as plt
        from sklearn.decomposition import PCA
        from sklearn.manifold import TSNE
    except ImportError:
        print("⚠ matplotlib, scikit-learn이 필요합니다")
        return
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('6DOF Missile Signature Analysis', fontsize=16, fontweight='bold')
    
    colors = plt.cm.Set1(np.linspace(0, 1, len(missile_types)))
    
    # 1. PCA 2D
    ax = axes[0, 0]
    pca = PCA(n_components=2)
    features_pca = pca.fit_transform(features)
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features_pca[mask, 0], features_pca[mask, 1], 
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel(f'PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)')
    ax.set_ylabel(f'PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)')
    ax.set_title('PCA Projection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. 궤적 형상: 고도 vs 사거리
    ax = axes[0, 1]
    alt_idx = feature_names.index('max_altitude_km') if 'max_altitude_km' in feature_names else 0
    range_idx = feature_names.index('final_range_km') if 'final_range_km' in feature_names else 1
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, range_idx], features[mask, alt_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Range (km)')
    ax.set_ylabel('Max Altitude (km)')
    ax.set_title('Trajectory Shape')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. 마하수 vs 비행시간
    ax = axes[0, 2]
    mach_idx = feature_names.index('max_mach') if 'max_mach' in feature_names else 6
    time_idx = feature_names.index('total_flight_time') if 'total_flight_time' in feature_names else 3
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, time_idx], features[mask, mach_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Flight Time (s)')
    ax.set_ylabel('Max Mach')
    ax.set_title('Mach vs Flight Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 4. 속도 특성
    ax = axes[1, 0]
    max_v_idx = feature_names.index('max_velocity') if 'max_velocity' in feature_names else 4
    term_v_idx = feature_names.index('terminal_velocity') if 'terminal_velocity' in feature_names else 5
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, max_v_idx], features[mask, term_v_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Max Velocity (m/s)')
    ax.set_ylabel('Terminal Velocity (m/s)')
    ax.set_title('Velocity Signature')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 5. 감속도 vs 경로 효율
    ax = axes[1, 1]
    decel_idx = feature_names.index('max_deceleration') if 'max_deceleration' in feature_names else 8
    eff_idx = feature_names.index('path_efficiency') if 'path_efficiency' in feature_names else 10
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, decel_idx], features[mask, eff_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Max Deceleration (m/s²)')
    ax.set_ylabel('Path Efficiency')
    ax.set_title('Deceleration & Efficiency')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 6. t-SNE (샘플 수가 충분하면)
    ax = axes[1, 2]
    if len(features) > 30:
        tsne = TSNE(n_components=2, random_state=42, perplexity=min(30, len(features)-1))
        features_tsne = tsne.fit_transform(features)
        for i, m_type in enumerate(missile_types):
            mask = labels == i
            ax.scatter(features_tsne[mask, 0], features_tsne[mask, 1],
                      c=[colors[i]], label=m_type, alpha=0.6, s=30)
        ax.set_title('t-SNE Projection')
    else:
        ax.text(0.5, 0.5, 'Not enough samples\nfor t-SNE', 
               ha='center', va='center', transform=ax.transAxes)
        ax.set_title('t-SNE (N/A)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_path = Path(output_dir) / 'signature_analysis.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n📊 시각화 저장: {output_path}")
    plt.show()


def main():
    """메인 실행"""
    print("\n" + "="*70)
    print("🎯 6DOF 미사일 시그니처 분류 시스템")
    print("="*70)
    
    # 1. 시그니처 데이터 생성
    generator = MissileSignatureGenerator(output_dir="signature_dataset")
    
    features, labels, metadata = generator.generate_dataset(
        missile_types=["SCUD-B", "Nodong", "KN-23"],
        samples_per_angle=3,
        noise_std=0.5
    )
    
    if len(features) == 0:
        print("❌ 데이터 생성 실패")
        return
    
    # 2. 시각화
    visualize_signatures(
        features, labels,
        metadata['feature_names'],
        metadata['missile_types'],
        output_dir="signature_dataset"
    )
    
    # 3. 분류기 학습
    print("\n" + "="*70)
    print("🤖 분류기 학습")
    print("="*70)
    
    classifier = MissileClassifier(model_type='random_forest')
    metrics = classifier.train(
        features, labels,
        feature_names=metadata['feature_names'],
        missile_types=metadata['missile_types']
    )
    
    print(f"\n✅ 최종 정확도: {metrics.get('accuracy', 0)*100:.1f}%")
    
    # 4. 분류 전략 요약
    print("\n" + "="*70)
    print("📋 탄종별 시그니처 분류 전략")
    print("="*70)
    print("""
    【핵심 시그니처 특성】
    
    1. 기하학적 시그니처
       - 고도/사거리 비율: 탄종별 고유 탄도 형상
       - 정점 도달 시간: 추진 특성 반영
       - 낙하각: 재진입 특성
    
    2. 6DOF 동역학 시그니처 ★ (본 프로젝트 핵심)
       - α-q 상관계수: 받음각-피치각속도 동적 관계
       - α-q 위상면적: 동적 안정성 특성
       - p-r 커플링: Roll-Yaw 관성 커플링
       - 동적 안정성 지수: 받음각 변동성
    
    3. 추진 시그니처
       - 최대 속도, 연소종료 속도
       - 추력/중량비
       - 연소시간 비율
    
    4. 공력 시그니처
       - 탄도계수: 형상/질량 특성
       - 활공비: 재진입 특성
       - 재진입 가열 지수: 열하중
    
    【분류 파이프라인】
    
    [6DOF 시뮬레이션] 
         ↓
    [32차원 시그니처 추출]
         ↓
    [특성 정규화]
         ↓
    [Random Forest / Gradient Boosting]
         ↓
    [탄종 분류 결과]
    
    【향후 발전 방향】
    
    1. 딥러닝 분류기 (CNN, LSTM)
       - 시계열 궤적 직접 입력
       - α-q 위상 이미지 기반 CNN
    
    2. 노이즈/불확실성 처리
       - 센서 노이즈 모델링
       - Dropout 기반 불확실성 추정
    
    3. 실시간 분류
       - 부분 궤적 기반 조기 분류
       - 확률적 업데이트
    """)


if __name__ == "__main__":
    main()
