"""
미사일 시그니처 분석을 위한 데이터 추출 및 배치 시뮬레이션 모듈
=====================================================================

주요 기능:
1. 시그니처 물리량 보강 (가속도, 비에너지)
2. 이벤트 플래그 (Burn-out, Apogee)
3. DataFrame 변환 및 CSV/NPZ 저장
4. 배치 시뮬레이션 (15°~80°, 5° 간격)

Author: Data Science Project
Date: 2024
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import os

from missile_6dof_true import (
    True6DOFSimulator, MISSILES, G0, get_atmosphere, quat_to_dcm, quat_to_euler
)

# 표준 NPZ 저장 함수 사용
from trajectory_io import save_npz_generic


# ============================================================================
# 시그니처 데이터 추출 함수
# ============================================================================

def extract_results(sim_result: Dict, missile_type: str, elevation_deg: float,
                    include_reference: bool = False) -> pd.DataFrame:
    """
    시뮬레이션 결과에서 시그니처 분석용 데이터를 추출
    
    Args:
        sim_result: True6DOFSimulator.simulate() 반환값
        missile_type: 미사일 기종명
        elevation_deg: 발사 고각 [deg]
        include_reference: CD, 연료소모량 등 참조용 데이터 포함 여부
        
    Returns:
        시그니처 분석용 DataFrame
    """
    t = sim_result['time']
    n_points = len(t)
    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1] if len(dt) > 1 else 0.01
    
    # 기본 데이터 추출
    x = sim_result['x']
    y = sim_result['y']
    altitude = sim_result['z']  # 이미 altitude로 변환됨
    
    u = sim_result['u']
    v = sim_result['v']
    w = sim_result['w']
    V = sim_result['V']
    
    phi = sim_result['phi']
    theta = sim_result['theta']
    psi = sim_result['psi']
    
    p = sim_result['p']
    q = sim_result['q']
    r = sim_result['r']
    
    # ================================================================
    # 1. 가속도 계산 (Body Frame)
    # ================================================================
    # 속도의 시간 미분으로 가속도 계산
    a_u = np.gradient(u, t)  # Body x-axis acceleration
    a_v = np.gradient(v, t)  # Body y-axis acceleration
    a_w = np.gradient(w, t)  # Body z-axis acceleration
    
    # ================================================================
    # 2. 관성 좌표계 가속도 계산
    # ================================================================
    # 관성 좌표계 속도 계산
    V_x = np.gradient(x, t)
    V_y = np.gradient(y, t)
    V_z = np.gradient(altitude, t)
    
    # 관성 좌표계 가속도
    a_x = np.gradient(V_x, t)
    a_y = np.gradient(V_y, t)
    a_z = np.gradient(V_z, t)
    
    # 총 가속도 크기
    a_total = np.sqrt(a_x**2 + a_y**2 + a_z**2)
    
    # ================================================================
    # 3. 비에너지 (Specific Energy) 계산
    # ================================================================
    # E_s = altitude + V^2 / (2*g)
    E_s = altitude + (V**2) / (2 * G0)
    
    # ================================================================
    # 4. 마하수 및 동압 계산
    # ================================================================
    mach = np.zeros(n_points)
    q_dyn = np.zeros(n_points)
    
    for i in range(n_points):
        rho, P, T, a = get_atmosphere(altitude[i])
        mach[i] = V[i] / a
        q_dyn[i] = 0.5 * rho * V[i]**2
    
    # ================================================================
    # 5. 받음각 및 옆미끄럼각 계산
    # ================================================================
    alpha = np.arctan2(w, np.maximum(u, 0.1))
    beta = np.arcsin(np.clip(v / np.maximum(V, 0.1), -1, 1))
    
    # ================================================================
    # 6. 이벤트 플래그 생성
    # ================================================================
    cfg = MISSILES[missile_type]
    
    # Burn-out 플래그: 연소 종료 시점
    flag_burnout = np.zeros(n_points, dtype=int)
    burnout_idx = np.searchsorted(t, cfg.burn_time)
    if burnout_idx < n_points:
        flag_burnout[burnout_idx] = 1
    
    # Apogee 플래그: 최고 고도 도달 시점
    flag_apogee = np.zeros(n_points, dtype=int)
    apogee_idx = np.argmax(altitude)
    flag_apogee[apogee_idx] = 1
    
    # 비행 단계 (Phase) 인코딩
    phase = np.zeros(n_points, dtype=int)
    for i, ti in enumerate(t):
        if ti < cfg.vertical_time:
            phase[i] = 0  # Vertical
        elif ti < cfg.vertical_time + cfg.pitch_time:
            phase[i] = 1  # Pitch
        elif ti < cfg.burn_time:
            phase[i] = 2  # Gravity Turn
        else:
            phase[i] = 3  # Ballistic
    
    # ================================================================
    # 7. DataFrame 구성 (시그니처 분석용)
    # ================================================================
    data = {
        # 시간
        'time': t,
        
        # 위치 (관성 좌표계)
        'x': x,
        'y': y,
        'altitude': altitude,
        
        # 속도 (Body Frame)
        'u': u,
        'v': v,
        'w': w,
        'V': V,
        
        # 가속도 (Body Frame) - 핵심 시그니처
        'a_u': a_u,
        'a_v': a_v,
        'a_w': a_w,
        
        # 가속도 (관성 좌표계)
        'a_x': a_x,
        'a_y': a_y,
        'a_z': a_z,
        'a_total': a_total,
        
        # 자세각 (Euler angles)
        'phi': phi,
        'theta': theta,
        'psi': psi,
        
        # 각속도 (Body Frame) - 핵심 시그니처
        'p': p,
        'q': q,
        'r': r,
        
        # 공력 파라미터
        'alpha': alpha,
        'beta': beta,
        'mach': mach,
        'q_dyn': q_dyn,
        
        # 에너지 시그니처
        'E_s': E_s,
        
        # 이벤트 플래그
        'flag_burnout': flag_burnout,
        'flag_apogee': flag_apogee,
        'phase': phase,
        
        # 메타데이터 (모든 행에 동일)
        'missile_type': missile_type,
        'elevation_deg': elevation_deg,
    }
    
    # 참조용 데이터 (선택적 포함)
    if include_reference:
        # 질량 변화 (참조용)
        mass = np.zeros(n_points)
        for i, ti in enumerate(t):
            if ti < cfg.burn_time:
                fuel_remaining = 1.0 - ti / cfg.burn_time
                mass[i] = cfg.mass_dry + cfg.mass_propellant * fuel_remaining
            else:
                mass[i] = cfg.mass_dry
        
        # 추력 (참조용)
        thrust = np.zeros(n_points)
        for i, ti in enumerate(t):
            if ti < cfg.burn_time:
                rho, P, T, a = get_atmosphere(altitude[i])
                p_ratio = P / 101325
                isp = cfg.isp_sea + (cfg.isp_vac - cfg.isp_sea) * (1 - p_ratio)
                mdot = cfg.mass_propellant / cfg.burn_time
                thrust[i] = isp * mdot * G0
        
        data['_ref_mass'] = mass
        data['_ref_thrust'] = thrust
        data['_ref_burn_time'] = cfg.burn_time
    
    df = pd.DataFrame(data)
    
    return df


def compute_additional_features(df: pd.DataFrame) -> pd.DataFrame:
    """
    추가 시그니처 특징 계산
    
    Args:
        df: extract_results()로 생성된 DataFrame
        
    Returns:
        추가 특징이 포함된 DataFrame
    """
    df = df.copy()
    
    # 각속도 크기
    df['omega_total'] = np.sqrt(df['p']**2 + df['q']**2 + df['r']**2)
    
    # 가속도 변화율 (Jerk) - 시그니처 특징
    df['jerk_u'] = np.gradient(df['a_u'], df['time'])
    df['jerk_v'] = np.gradient(df['a_v'], df['time'])
    df['jerk_w'] = np.gradient(df['a_w'], df['time'])
    
    # 비에너지 변화율
    df['dE_s_dt'] = np.gradient(df['E_s'], df['time'])
    
    # 수평 거리
    df['range'] = np.sqrt(df['x']**2 + df['y']**2)
    
    # 비행 경로각 (Flight Path Angle)
    V_horizontal = np.sqrt(np.gradient(df['x'], df['time'])**2 + 
                           np.gradient(df['y'], df['time'])**2)
    V_vertical = np.gradient(df['altitude'], df['time'])
    df['gamma'] = np.arctan2(V_vertical, np.maximum(V_horizontal, 0.1))
    
    return df


# ============================================================================
# 데이터 저장 함수
# ============================================================================

def save_signature_data(df: pd.DataFrame, output_dir: str, 
                        file_format: str = 'both') -> Tuple[str, str]:
    """
    시그니처 데이터를 파일로 저장
    
    Args:
        df: 시그니처 DataFrame
        output_dir: 출력 디렉토리 경로
        file_format: 'csv', 'npz', 또는 'both'
        
    Returns:
        (csv_path, npz_path) 튜플
    """
    # 출력 디렉토리 생성
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # 파일명 생성
    missile_type = df['missile_type'].iloc[0]
    elevation = int(df['elevation_deg'].iloc[0])
    base_name = f"{missile_type}_{elevation}deg_dataset"
    
    csv_path = None
    npz_path = None
    
    if file_format in ['csv', 'both']:
        csv_path = output_path / f"{base_name}.csv"
        df.to_csv(csv_path, index=False)
        print(f"  ✓ CSV 저장: {csv_path}")
    
    if file_format in ['npz', 'both']:
        npz_path = output_path / f"{base_name}.npz"
        
        # DataFrame을 numpy 배열로 변환
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        arrays = {col: df[col].values for col in numeric_cols}
        
        # 문자열 컬럼은 별도 처리
        arrays['missile_type'] = np.array([missile_type])
        arrays['elevation_deg'] = np.array([elevation])
        arrays['column_names'] = np.array(list(numeric_cols))
        
        save_npz_generic(npz_path, arrays)
        print(f"  ✓ NPZ 저장: {npz_path}")
    
    return str(csv_path) if csv_path else None, str(npz_path) if npz_path else None


# ============================================================================
# 배치 시뮬레이션 함수
# ============================================================================

def run_batch_simulation(missile_type: str,
                         elevation_start: float = 15,
                         elevation_end: float = 80,
                         elevation_step: float = 5,
                         output_dir: str = None,
                         file_format: str = 'both',
                         include_reference: bool = False,
                         add_features: bool = True) -> List[pd.DataFrame]:
    """
    특정 기종에 대해 다양한 발사각으로 배치 시뮬레이션 수행
    
    Args:
        missile_type: 미사일 기종명 ('SCUD-B', 'Nodong', 'KN-23')
        elevation_start: 시작 발사각 [deg] (기본값: 15)
        elevation_end: 종료 발사각 [deg] (기본값: 80)
        elevation_step: 발사각 간격 [deg] (기본값: 5)
        output_dir: 출력 디렉토리 (None이면 현재 디렉토리/signature_datasets)
        file_format: 저장 형식 ('csv', 'npz', 'both', 'none')
        include_reference: 참조용 데이터(CD, 연료소모량) 포함 여부
        add_features: 추가 특징 계산 여부
        
    Returns:
        각 발사각별 DataFrame 리스트
    """
    if missile_type not in MISSILES:
        raise ValueError(f"Unknown missile type: {missile_type}. "
                        f"Available: {list(MISSILES.keys())}")
    
    # 출력 디렉토리 설정
    if output_dir is None:
        output_dir = Path(__file__).parent / "signature_datasets" / missile_type
    else:
        output_dir = Path(output_dir) / missile_type
    
    print(f"\n{'='*70}")
    print(f"배치 시뮬레이션 시작: {missile_type}")
    print(f"발사각 범위: {elevation_start}° ~ {elevation_end}° (간격: {elevation_step}°)")
    print(f"출력 디렉토리: {output_dir}")
    print(f"{'='*70}")
    
    # 발사각 리스트 생성
    elevations = np.arange(elevation_start, elevation_end + elevation_step, elevation_step)
    
    results = []
    
    for i, elev in enumerate(elevations):
        print(f"\n[{i+1}/{len(elevations)}] 발사각: {elev}°")
        print("-" * 40)
        
        try:
            # 시뮬레이션 실행
            sim = True6DOFSimulator(missile_type)
            sim_result = sim.simulate(elevation_deg=elev)
            
            # 시그니처 데이터 추출
            df = extract_results(sim_result, missile_type, elev, 
                               include_reference=include_reference)
            
            # 추가 특징 계산
            if add_features:
                df = compute_additional_features(df)
            
            # 파일 저장
            if file_format != 'none':
                save_signature_data(df, str(output_dir), file_format)
            
            results.append(df)
            
            print(f"  → 데이터 포인트: {len(df)}")
            print(f"  → 사거리: {sim_result['range_km']:.1f} km")
            print(f"  → 최고 고도: {sim_result['max_alt_km']:.1f} km")
            
        except Exception as e:
            print(f"  ✗ 오류 발생: {e}")
            continue
    
    print(f"\n{'='*70}")
    print(f"배치 시뮬레이션 완료: {len(results)}/{len(elevations)} 성공")
    print(f"{'='*70}")
    
    return results


def run_all_missiles_batch(output_dir: str = None,
                           elevation_start: float = 15,
                           elevation_end: float = 80,
                           elevation_step: float = 5,
                           file_format: str = 'both') -> Dict[str, List[pd.DataFrame]]:
    """
    모든 미사일 기종에 대해 배치 시뮬레이션 수행
    
    Args:
        output_dir: 출력 디렉토리
        elevation_start: 시작 발사각 [deg]
        elevation_end: 종료 발사각 [deg]
        elevation_step: 발사각 간격 [deg]
        file_format: 저장 형식
        
    Returns:
        {missile_type: [DataFrame, ...]} 딕셔너리
    """
    all_results = {}
    
    for missile_type in MISSILES.keys():
        results = run_batch_simulation(
            missile_type=missile_type,
            elevation_start=elevation_start,
            elevation_end=elevation_end,
            elevation_step=elevation_step,
            output_dir=output_dir,
            file_format=file_format
        )
        all_results[missile_type] = results
    
    return all_results


def merge_datasets(results: List[pd.DataFrame]) -> pd.DataFrame:
    """
    여러 시뮬레이션 결과를 하나의 DataFrame으로 병합
    
    Args:
        results: DataFrame 리스트
        
    Returns:
        병합된 DataFrame
    """
    if not results:
        return pd.DataFrame()
    
    # 각 DataFrame에 고유 ID 추가
    merged_dfs = []
    for i, df in enumerate(results):
        df_copy = df.copy()
        df_copy['simulation_id'] = i
        merged_dfs.append(df_copy)
    
    return pd.concat(merged_dfs, ignore_index=True)


def create_ml_dataset(all_results: Dict[str, List[pd.DataFrame]],
                      output_path: str = None) -> pd.DataFrame:
    """
    머신러닝 학습용 통합 데이터셋 생성
    
    Args:
        all_results: run_all_missiles_batch() 반환값
        output_path: 저장 경로 (None이면 저장 안함)
        
    Returns:
        통합 DataFrame
    """
    all_dfs = []
    
    for missile_type, results in all_results.items():
        for df in results:
            all_dfs.append(df)
    
    merged = pd.concat(all_dfs, ignore_index=True)
    
    # 레이블 인코딩
    label_map = {name: i for i, name in enumerate(MISSILES.keys())}
    merged['label'] = merged['missile_type'].map(label_map)
    
    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        merged.to_csv(output_path, index=False)
        print(f"\n✓ ML 데이터셋 저장: {output_path}")
        print(f"  총 샘플 수: {len(merged)}")
        print(f"  기종별 분포:")
        for mt in MISSILES.keys():
            count = len(merged[merged['missile_type'] == mt])
            print(f"    - {mt}: {count}")
    
    return merged


# ============================================================================
# 데이터 분석 유틸리티
# ============================================================================

def get_signature_summary(df: pd.DataFrame) -> Dict:
    """
    시그니처 데이터 요약 통계
    
    Args:
        df: 시그니처 DataFrame
        
    Returns:
        요약 통계 딕셔너리
    """
    summary = {
        'missile_type': df['missile_type'].iloc[0],
        'elevation_deg': df['elevation_deg'].iloc[0],
        'n_samples': len(df),
        'flight_time': df['time'].max(),
        'max_altitude': df['altitude'].max(),
        'max_velocity': df['V'].max(),
        'max_mach': df['mach'].max(),
        'max_acceleration': df['a_total'].max(),
        'max_specific_energy': df['E_s'].max(),
        'burnout_time': df[df['flag_burnout'] == 1]['time'].values[0] if df['flag_burnout'].sum() > 0 else None,
        'apogee_time': df[df['flag_apogee'] == 1]['time'].values[0] if df['flag_apogee'].sum() > 0 else None,
        'range_km': np.sqrt(df['x'].iloc[-1]**2 + df['y'].iloc[-1]**2) / 1000,
    }
    
    return summary


def print_signature_summary(df: pd.DataFrame):
    """시그니처 요약 출력"""
    s = get_signature_summary(df)
    
    print(f"\n{'='*50}")
    print(f"시그니처 데이터 요약: {s['missile_type']} @ {s['elevation_deg']}°")
    print(f"{'='*50}")
    print(f"  샘플 수: {s['n_samples']}")
    print(f"  비행 시간: {s['flight_time']:.1f} s")
    print(f"  최대 고도: {s['max_altitude']/1000:.1f} km")
    print(f"  최대 속도: {s['max_velocity']:.1f} m/s (Mach {s['max_mach']:.2f})")
    print(f"  최대 가속도: {s['max_acceleration']:.1f} m/s²")
    print(f"  최대 비에너지: {s['max_specific_energy']/1000:.1f} km")
    print(f"  사거리: {s['range_km']:.1f} km")
    if s['burnout_time']:
        print(f"  연소 종료: {s['burnout_time']:.1f} s")
    if s['apogee_time']:
        print(f"  최고점 도달: {s['apogee_time']:.1f} s")


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='미사일 시그니처 데이터 추출 및 배치 시뮬레이션')
    parser.add_argument('--missile', type=str, default='all',
                       help='미사일 기종 (SCUD-B, Nodong, KN-23, all)')
    parser.add_argument('--start', type=float, default=15,
                       help='시작 발사각 (기본값: 15)')
    parser.add_argument('--end', type=float, default=80,
                       help='종료 발사각 (기본값: 80)')
    parser.add_argument('--step', type=float, default=5,
                       help='발사각 간격 (기본값: 5)')
    parser.add_argument('--output', type=str, default=None,
                       help='출력 디렉토리')
    parser.add_argument('--format', type=str, default='both',
                       choices=['csv', 'npz', 'both', 'none'],
                       help='저장 형식')
    
    args = parser.parse_args()
    
    if args.missile == 'all':
        # 모든 기종 배치 시뮬레이션
        all_results = run_all_missiles_batch(
            output_dir=args.output,
            elevation_start=args.start,
            elevation_end=args.end,
            elevation_step=args.step,
            file_format=args.format
        )
        
        # ML 데이터셋 생성
        if args.output:
            ml_path = Path(args.output) / "ml_dataset_all.csv"
        else:
            ml_path = Path(__file__).parent / "signature_datasets" / "ml_dataset_all.csv"
        
        create_ml_dataset(all_results, str(ml_path))
        
    else:
        # 단일 기종 배치 시뮬레이션
        results = run_batch_simulation(
            missile_type=args.missile,
            elevation_start=args.start,
            elevation_end=args.end,
            elevation_step=args.step,
            output_dir=args.output,
            file_format=args.format
        )
        
        # 첫 번째 결과 요약 출력
        if results:
            print_signature_summary(results[0])
