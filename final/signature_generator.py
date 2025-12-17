#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6DOF 미사일 시그니처 데이터 생성기
- final/missile_6dof.py 기반 6DOF 시뮬레이션
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
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

# 현재 디렉토리를 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from missile_6dof import Missile6DOF_Professor
    import config as cfg
    print("✓ missile_6dof, config 모듈 로드 성공")
except ImportError as e:
    print(f"✗ 모듈 로드 실패: {e}")
    sys.exit(1)


class MissileSignatureGenerator:
    """
    6DOF 기반 미사일 시그니처 데이터 생성기
    
    탄종 분류를 위한 핵심 시그니처:
    1. 기하학적 특성: 궤적 형상, 최대고도/사거리 비율
    2. 동역학적 특성: 받음각, 각속도, 위상 평면
    3. 추진 특성: 가속도 프로파일, 연소 종료 시점
    4. 공력 특성: 양항비, 항력 프로파일
    """
    
    # 지원 미사일 타입
    SUPPORTED_MISSILES = ["SCUD-B", "NODONG", "KN-23"]
    
    # 시그니처 특성 정의 (32차원)
    SIGNATURE_FEATURES = [
        # 기하학적 특성 (8개)
        'max_altitude_km',           # 최대 고도
        'final_range_km',            # 최종 사거리
        'altitude_range_ratio',      # 고도/사거리 비율
        'apogee_time_ratio',         # 정점 도달 시간 비율
        'path_efficiency',           # 경로 효율성
        'ground_track_curvature',    # 지상 궤적 곡률
        'impact_angle_deg',          # 낙하각
        'total_flight_time',         # 총 비행시간
        
        # 속도 특성 (6개)
        'max_velocity',              # 최대 속도
        'burnout_velocity',          # 연소종료 속도
        'terminal_velocity',         # 종말 속도
        'velocity_loss_ratio',       # 속도 손실률
        'max_mach',                  # 최대 마하수
        'mach_at_apogee',            # 정점 마하수
        
        # 가속도 특성 (4개)
        'max_acceleration',          # 최대 가속도
        'max_deceleration',          # 최대 감속도
        'burn_time_ratio',           # 연소시간 비율
        'thrust_to_weight_initial',  # 초기 추력/중량비
        
        # 6DOF 고유 특성 (10개) ★ 핵심 시그니처
        'alpha_max_deg',             # 최대 받음각
        'alpha_mean_deg',            # 평균 받음각
        'alpha_std_deg',             # 받음각 표준편차
        'q_max_deg_s',               # 최대 피치 각속도
        'q_mean_deg_s',              # 평균 피치 각속도
        'alpha_q_correlation',       # α-q 상관계수 ★
        'alpha_q_phase_area',        # α-q 위상면적 ★
        'p_r_coupling_strength',     # Roll-Yaw 커플링 ★
        'angular_momentum_ratio',    # 각운동량 비율
        'dynamic_stability_index',   # 동적 안정성 지수
        
        # 추가 파생 특성 (4개)
        'ballistic_coefficient',     # 탄도계수
        'energy_ratio',              # 에너지 비율
        'glide_ratio',               # 활공비
        'reentry_heating_index',     # 재진입 가열 지수
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
            features: 시그니처 특성 배열 [N, 32]
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
                        # 6DOF 시뮬레이션 실행
                        sim = Missile6DOF_Professor(missile_type=missile_type)
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
            features: 32차원 시그니처 벡터
        """
        try:
            t = results['time']
            V = results['V']
            gamma = results['gamma']
            x = results['position_x']
            y = results['position_y']
            h = results['altitude']
            alpha = results['alpha']
            theta = results['theta']
            phi = results['phi']
            p = results['p']
            q = results['q']
            r = results['r']
            mass = results['mass']
            mach = results['mach']
            
            # 데이터 검증
            if len(t) < 50 or np.any(np.isnan(V)) or np.any(np.isinf(h)):
                return None
            
            # 미사일 정보
            missile_info = cfg.ENHANCED_MISSILE_TYPES.get(missile_type, {})
            burn_time = missile_info.get('burn_time', 65)
            initial_mass = missile_info.get('launch_weight', 5860)
            
            features = np.zeros(len(self.SIGNATURE_FEATURES), dtype=np.float32)
            
            # === 기하학적 특성 (8개) ===
            max_h = np.max(h)
            final_range = np.sqrt(x[-1]**2 + y[-1]**2)
            apogee_idx = np.argmax(h)
            
            features[0] = max_h / 1000  # max_altitude_km
            features[1] = final_range / 1000  # final_range_km
            features[2] = max_h / (final_range + 1e-6)  # altitude_range_ratio
            features[3] = t[apogee_idx] / t[-1]  # apogee_time_ratio
            
            # 경로 효율성
            path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(h)**2))
            features[4] = final_range / (path_length + 1e-6)  # path_efficiency
            
            # 지상 궤적 곡률
            dx = np.gradient(x)
            dy = np.gradient(y)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            curvature = np.abs(dx*ddy - dy*ddx) / (dx**2 + dy**2 + 1e-6)**1.5
            features[5] = np.mean(curvature[~np.isnan(curvature)])  # ground_track_curvature
            
            # 낙하각 (마지막 gamma)
            features[6] = np.abs(np.rad2deg(gamma[-1]))  # impact_angle_deg
            features[7] = t[-1]  # total_flight_time
            
            # === 속도 특성 (6개) ===
            burn_idx = np.argmin(np.abs(t - burn_time))
            
            features[8] = np.max(V)  # max_velocity
            features[9] = V[min(burn_idx, len(V)-1)]  # burnout_velocity
            features[10] = V[-1]  # terminal_velocity
            features[11] = (np.max(V) - V[-1]) / (np.max(V) + 1e-6)  # velocity_loss_ratio
            features[12] = np.max(mach)  # max_mach
            features[13] = mach[apogee_idx]  # mach_at_apogee
            
            # === 가속도 특성 (4개) ===
            dV_dt = np.gradient(V, t)
            features[14] = np.max(dV_dt)  # max_acceleration
            features[15] = np.min(dV_dt)  # max_deceleration
            features[16] = burn_time / t[-1]  # burn_time_ratio
            
            # 초기 추력/중량비 (추정)
            isp = missile_info.get('isp_sea', 230)
            propellant_mass = missile_info.get('propellant_mass', 4875)
            thrust = isp * (propellant_mass / burn_time) * 9.81
            features[17] = thrust / (initial_mass * 9.81)  # thrust_to_weight_initial
            
            # === 6DOF 고유 특성 (10개) ★ 핵심 ===
            alpha_deg = np.rad2deg(alpha)
            q_deg_s = np.rad2deg(q)
            p_deg_s = np.rad2deg(p)
            r_deg_s = np.rad2deg(r)
            
            features[18] = np.max(np.abs(alpha_deg))  # alpha_max_deg
            features[19] = np.mean(np.abs(alpha_deg))  # alpha_mean_deg
            features[20] = np.std(alpha_deg)  # alpha_std_deg
            features[21] = np.max(np.abs(q_deg_s))  # q_max_deg_s
            features[22] = np.mean(np.abs(q_deg_s))  # q_mean_deg_s
            
            # α-q 상관계수 ★
            valid_mask = ~(np.isnan(alpha_deg) | np.isnan(q_deg_s))
            if np.sum(valid_mask) > 10:
                corr = np.corrcoef(alpha_deg[valid_mask], q_deg_s[valid_mask])[0, 1]
                features[23] = corr if not np.isnan(corr) else 0  # alpha_q_correlation
            
            # α-q 위상면적 ★ (Shoelace formula)
            try:
                area = 0.5 * np.abs(np.sum(alpha_deg[:-1] * q_deg_s[1:] - 
                                           alpha_deg[1:] * q_deg_s[:-1]))
                features[24] = np.log1p(area)  # alpha_q_phase_area (log scale)
            except:
                features[24] = 0
            
            # Roll-Yaw 커플링 ★
            valid_mask = ~(np.isnan(p_deg_s) | np.isnan(r_deg_s))
            if np.sum(valid_mask) > 10:
                corr = np.corrcoef(p_deg_s[valid_mask], r_deg_s[valid_mask])[0, 1]
                features[25] = corr if not np.isnan(corr) else 0  # p_r_coupling_strength
            
            # 각운동량 비율
            I_ratio = 1.0  # I_yy / I_zz (실제론 미사일별 다름)
            features[26] = np.mean(np.abs(q)) / (np.mean(np.abs(r)) + 1e-6)  # angular_momentum_ratio
            
            # 동적 안정성 지수 (alpha 변동성 기반)
            alpha_variation = np.std(np.diff(alpha_deg))
            features[27] = 1.0 / (1.0 + alpha_variation)  # dynamic_stability_index
            
            # === 추가 파생 특성 (4개) ===
            # 탄도계수 (BC = m / (Cd * A))
            ref_area = missile_info.get('reference_area', 0.6)
            cd_base = missile_info.get('cd_base', 0.3)
            features[28] = initial_mass / (cd_base * ref_area + 1e-6)  # ballistic_coefficient
            
            # 에너지 비율 (운동에너지 / 위치에너지)
            KE = 0.5 * mass[-1] * V[-1]**2
            PE = mass[-1] * 9.81 * max_h
            features[29] = KE / (PE + 1e-6)  # energy_ratio
            
            # 활공비 (탄도비행 구간)
            if apogee_idx < len(h) - 10:
                descent_range = np.sqrt((x[-1]-x[apogee_idx])**2 + (y[-1]-y[apogee_idx])**2)
                descent_alt = h[apogee_idx]
                features[30] = descent_range / (descent_alt + 1e-6)  # glide_ratio
            
            # 재진입 가열 지수 (속도 * 밀도^0.5)
            # 간단히 최대 동압 사용
            rho_approx = 1.225 * np.exp(-h / 8500)  # 지수 대기 모델
            q_dynamic = 0.5 * rho_approx * V**2
            features[31] = np.max(q_dynamic) / 1e6  # reentry_heating_index (MPa)
            
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
        
        return {
            'time': t[indices].astype(np.float32),
            'V': results['V'][indices].astype(np.float32),
            'h': results['altitude'][indices].astype(np.float32),
            'x': results['position_x'][indices].astype(np.float32),
            'y': results['position_y'][indices].astype(np.float32),
            'alpha': results['alpha'][indices].astype(np.float32),
            'q': results['q'][indices].astype(np.float32),
            'gamma': results['gamma'][indices].astype(np.float32),
            'mach': results['mach'][indices].astype(np.float32)
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
        np.savez_compressed(
            features_file,
            features=features,
            labels=labels,
            feature_names=self.SIGNATURE_FEATURES,
            missile_types=missile_types,
            n_samples=len(features),
            n_features=len(self.SIGNATURE_FEATURES)
        )
        
        # 2. 궤적 데이터 저장
        trajectory_file = self.output_dir / f"trajectories_{timestamp}.npz"
        traj_dict = {f'traj_{i}': traj for i, traj in enumerate(trajectories)}
        np.savez_compressed(trajectory_file, **traj_dict)
        
        # 3. 메타데이터 저장
        meta_file = self.output_dir / f"metadata_{timestamp}.npz"
        np.savez_compressed(
            meta_file,
            metadata=metadata,
            generation_stats=self.generation_stats,
            launch_angles=self.launch_angles,
            azimuth_angles=self.azimuth_angles
        )
        
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
    
    # 2. 주요 시그니처: α-q correlation
    ax = axes[0, 1]
    alpha_q_idx = feature_names.index('alpha_q_correlation') if 'alpha_q_correlation' in feature_names else 23
    alpha_max_idx = feature_names.index('alpha_max_deg') if 'alpha_max_deg' in feature_names else 18
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, alpha_max_idx], features[mask, alpha_q_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Max Alpha (deg)')
    ax.set_ylabel('Alpha-Q Correlation')
    ax.set_title('6DOF Signature: Alpha Dynamics')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. 궤적 형상: 고도/사거리 비율
    ax = axes[0, 2]
    alt_range_idx = feature_names.index('altitude_range_ratio') if 'altitude_range_ratio' in feature_names else 2
    range_idx = feature_names.index('final_range_km') if 'final_range_km' in feature_names else 1
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, range_idx], features[mask, alt_range_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Range (km)')
    ax.set_ylabel('Altitude/Range Ratio')
    ax.set_title('Trajectory Shape')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 4. 속도 특성
    ax = axes[1, 0]
    max_v_idx = feature_names.index('max_velocity') if 'max_velocity' in feature_names else 8
    term_v_idx = feature_names.index('terminal_velocity') if 'terminal_velocity' in feature_names else 10
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, max_v_idx], features[mask, term_v_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Max Velocity (m/s)')
    ax.set_ylabel('Terminal Velocity (m/s)')
    ax.set_title('Velocity Signature')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 5. 동적 특성
    ax = axes[1, 1]
    q_max_idx = feature_names.index('q_max_deg_s') if 'q_max_deg_s' in feature_names else 21
    stab_idx = feature_names.index('dynamic_stability_index') if 'dynamic_stability_index' in feature_names else 27
    for i, m_type in enumerate(missile_types):
        mask = labels == i
        ax.scatter(features[mask, q_max_idx], features[mask, stab_idx],
                  c=[colors[i]], label=m_type, alpha=0.6, s=30)
    ax.set_xlabel('Max Pitch Rate (deg/s)')
    ax.set_ylabel('Dynamic Stability Index')
    ax.set_title('6DOF Dynamic Signature')
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
        missile_types=["SCUD-B", "NODONG", "KN-23"],
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
