"""
6DOF 미사일 궤적 데이터 표준 I/O 모듈 (리팩터링 완료)
==========================================================

표준 NPZ 포맷 단일화 - 모든 trajectory 저장은 이 모듈을 통해서만 수행

표준 키 세트:
- time
- position_x, position_y, position_z
- u, v, w
- phi, theta, psi
- p, q, r
- mass
+ 메타데이터 (missile_type, launch_angle, azimuth, seed 등)

저장 폴더/파일명 규칙:
- 기본 폴더: ./outputs/trajectories/
- 파일명: {missile_type}__elev{ELEV:.1f}__azi{AZI:.1f}__seed{SEED}__{YYYYMMDD_HHMMSS}.npz
"""
import numpy as np
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Any, Union, List

# =============================================================================
# 상수 정의
# =============================================================================

# 표준 저장 경로
DEFAULT_OUTPUT_DIR = Path(__file__).parent / "outputs" / "trajectories"

# 필수 키 목록
REQUIRED_KEYS = [
    'time', 'position_x', 'position_y', 'position_z',
    'u', 'v', 'w', 'phi', 'theta', 'psi',
    'p', 'q', 'r', 'mass'
]

# 선택적 키 목록 (자동 계산 가능)
OPTIONAL_KEYS = ['V', 'gamma', 'chi', 'a_x', 'a_y', 'a_z', 'E_s', 'q_dyn', 'alpha', 'beta', 'mach']

# 물리 상수
G0 = 9.80665


# =============================================================================
# 표준화 헬퍼 함수 (핵심!)
# =============================================================================

def standardize_results_to_trajectory(
    results: Dict[str, Any],
    missile_type: str,
    metadata: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    시뮬레이션 결과 dict를 표준 trajectory 포맷으로 변환
    
    다양한 시뮬레이터 출력 형식을 표준 NPZ 포맷으로 통일:
    - True6DOFSimulator 출력
    - KN23Depressed 출력
    - main_visualization results dict
    
    Parameters:
    -----------
    results : dict
        시뮬레이션 결과 딕셔너리 (다양한 키 이름 허용)
    missile_type : str
        미사일 종류 ("SCUD-B", "Nodong", "KN-23")
    metadata : dict, optional
        추가 메타데이터 (elevation, azimuth, seed 등)
    
    Returns:
    --------
    traj : dict
        표준 trajectory 포맷 딕셔너리 (save_trajectory에 바로 전달 가능)
    """
    traj = {}
    
    # 1. time 추출
    traj['time'] = _extract_array(results, ['time', 't'])
    n_points = len(traj['time'])
    
    # 2. position 추출 (다양한 키 이름 매핑)
    traj['position_x'] = _extract_array(results, ['position_x', 'x', 'pos_x', 'X'])
    traj['position_y'] = _extract_array(results, ['position_y', 'y', 'pos_y', 'Y'], default=np.zeros(n_points))
    traj['position_z'] = _extract_array(results, ['position_z', 'z', 'h', 'altitude', 'alt', 'pos_z', 'Z'])
    
    # 3. Body Frame 속도 (u, v, w) 추출 또는 계산
    if 'u' in results and 'v' in results and 'w' in results:
        traj['u'] = np.asarray(results['u'])
        traj['v'] = np.asarray(results['v'])
        traj['w'] = np.asarray(results['w'])
    else:
        # V, gamma, psi로부터 근사 계산
        V = _extract_array(results, ['V', 'velocity', 'speed', 'vel'], default=None)
        gamma = _extract_array(results, ['gamma', 'flight_path_angle', 'fpa'], default=None)
        psi = _extract_array(results, ['psi', 'heading', 'azimuth_angle', 'chi'], default=np.zeros(n_points))
        
        if V is not None and gamma is not None:
            # Body frame 속도 근사 (정확하지 않지만 시각화용으로 충분)
            traj['u'] = V * np.cos(gamma) * np.cos(psi)
            traj['v'] = V * np.cos(gamma) * np.sin(psi)
            traj['w'] = V * np.sin(gamma)
        else:
            # 위치 미분으로 계산
            traj['u'] = np.gradient(traj['position_x'], traj['time'])
            traj['v'] = np.gradient(traj['position_y'], traj['time'])
            traj['w'] = np.gradient(traj['position_z'], traj['time'])
    
    # 4. Euler 각도 추출
    traj['phi'] = _extract_array(results, ['phi', 'roll', 'bank'], default=np.zeros(n_points))
    traj['theta'] = _extract_array(results, ['theta', 'pitch', 'pitch_angle'], default=np.zeros(n_points))
    traj['psi'] = _extract_array(results, ['psi', 'psi_euler', 'yaw', 'heading'], default=np.zeros(n_points))
    
    # 5. 각속도 추출
    traj['p'] = _extract_array(results, ['p', 'roll_rate', 'omega_x'], default=np.zeros(n_points))
    traj['q'] = _extract_array(results, ['q', 'pitch_rate', 'omega_y'], default=np.zeros(n_points))
    traj['r'] = _extract_array(results, ['r', 'yaw_rate', 'omega_z'], default=np.zeros(n_points))
    
    # 6. 질량 추출
    traj['mass'] = _extract_array(results, ['mass', 'm', 'weight'], default=np.ones(n_points) * 1000)
    
    # 7. 선택적 데이터 추출/계산
    # V (속도 크기)
    if 'V' in results:
        traj['V'] = np.asarray(results['V'])
    elif 'velocity' in results:
        traj['V'] = np.asarray(results['velocity'])
    else:
        traj['V'] = np.sqrt(traj['u']**2 + traj['v']**2 + traj['w']**2)
    
    # gamma (비행경로각)
    traj['gamma'] = _extract_array(results, ['gamma', 'flight_path_angle'], 
                                   default=np.arcsin(np.clip(traj['w'] / np.maximum(traj['V'], 0.1), -1, 1)))
    
    # chi (방위각)
    traj['chi'] = _extract_array(results, ['chi', 'heading', 'course'], default=traj['psi'].copy())
    
    # alpha, beta, mach
    traj['alpha'] = _extract_array(results, ['alpha', 'angle_of_attack', 'aoa'], default=np.zeros(n_points))
    traj['beta'] = _extract_array(results, ['beta', 'sideslip'], default=np.zeros(n_points))
    
    # mach - 자동 계산
    mach_arr = None
    for key in ['mach', 'mach_number']:
        if key in results:
            mach_arr = np.asarray(results[key])
            break
    
    if mach_arr is None:
        # 마하수 계산
        h = traj['position_z']
        T = np.where(h < 11000, 288.15 - 0.0065 * h, 216.65)
        a = np.sqrt(1.4 * 287.05 * np.maximum(T, 1))
        traj['mach'] = traj['V'] / np.maximum(a, 1)
    else:
        traj['mach'] = mach_arr
    
    # 8. 메타데이터 추가
    traj['missile_type'] = missile_type.replace(' ', '-')
    
    if metadata:
        for key, value in metadata.items():
            traj[key] = value
    
    return traj


def _extract_array(
    data: Dict, 
    key_candidates: List[str], 
    default: Any = None
) -> np.ndarray:
    """여러 후보 키 중 존재하는 키의 값을 배열로 반환"""
    for key in key_candidates:
        if key in data:
            return np.asarray(data[key])
    
    if default is not None:
        return np.asarray(default) if not isinstance(default, np.ndarray) else default
    
    raise KeyError(f"필수 키 누락: {key_candidates} 중 하나가 필요합니다")


# =============================================================================
# 파일명 생성 함수
# =============================================================================

def generate_trajectory_filename(
    missile_type: str,
    elevation: float,
    azimuth: float = 90.0,
    seed: int = 0
) -> str:
    """
    표준 파일명 생성
    
    형식: {missile_type}__elev{ELEV:.1f}__azi{AZI:.1f}__seed{SEED}__{YYYYMMDD_HHMMSS}.npz
    """
    # 공백을 '-'로 치환
    missile_type_safe = missile_type.replace(' ', '-')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    filename = f"{missile_type_safe}__elev{elevation:.1f}__azi{azimuth:.1f}__seed{seed}__{timestamp}.npz"
    return filename


# =============================================================================
# 통합 저장 함수 (권장 진입점)
# =============================================================================

def save_trajectory_unified(
    results: Dict[str, Any],
    missile_type: str,
    elevation: float,
    azimuth: float = 90.0,
    seed: int = 0,
    output_dir: Optional[Union[str, Path]] = None,
    extra_metadata: Optional[Dict[str, Any]] = None
) -> str:
    """
    시뮬레이션 결과를 표준 NPZ 포맷으로 저장 (통합 진입점)
    
    이 함수 하나로 모든 trajectory 저장을 처리합니다.
    
    Parameters:
    -----------
    results : dict
        시뮬레이션 결과 딕셔너리 (다양한 형식 허용)
    missile_type : str
        미사일 종류
    elevation : float
        발사각 (deg)
    azimuth : float
        방위각 (deg), 기본값 90
    seed : int
        시뮬레이션 시드, 기본값 0
    output_dir : str or Path, optional
        저장 디렉토리 (기본값: ./outputs/trajectories/)
    extra_metadata : dict, optional
        추가 메타데이터
    
    Returns:
    --------
    filepath : str
        저장된 파일 경로
    """
    # 출력 디렉토리 설정
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 파일명 생성
    filename = generate_trajectory_filename(missile_type, elevation, azimuth, seed)
    filepath = output_dir / filename
    
    # 메타데이터 구성
    metadata = {
        'launch_angle': elevation,
        'elevation': elevation,
        'azimuth': azimuth,
        'seed': seed,
    }
    if extra_metadata:
        metadata.update(extra_metadata)
    
    # 결과를 표준 포맷으로 변환
    traj = standardize_results_to_trajectory(results, missile_type, metadata)
    
    # 저장
    save_trajectory(
        filepath=str(filepath),
        time=traj['time'],
        position_x=traj['position_x'],
        position_y=traj['position_y'],
        position_z=traj['position_z'],
        u=traj['u'],
        v=traj['v'],
        w=traj['w'],
        phi=traj['phi'],
        theta=traj['theta'],
        psi=traj['psi'],
        p=traj['p'],
        q=traj['q'],
        r=traj['r'],
        mass=traj['mass'],
        V=traj['V'],
        gamma=traj['gamma'],
        chi=traj['chi'],
        missile_type=traj['missile_type'],
        launch_angle=elevation,
        alpha=traj.get('alpha'),
        beta=traj.get('beta'),
        mach=traj.get('mach'),
        elevation=elevation,
        azimuth=azimuth,
        seed=seed
    )
    
    return str(filepath)


# =============================================================================
# 저수준 저장 함수 (내부용, 직접 호출도 가능)
# =============================================================================

def save_trajectory(
    filepath: str,
    time: np.ndarray,
    position_x: np.ndarray,
    position_y: np.ndarray,
    position_z: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    w: np.ndarray,
    phi: np.ndarray,
    theta: np.ndarray,
    psi: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    mass: np.ndarray,
    V: np.ndarray = None,
    gamma: np.ndarray = None,
    chi: np.ndarray = None,
    a_x: np.ndarray = None,
    a_y: np.ndarray = None,
    a_z: np.ndarray = None,
    E_s: np.ndarray = None,
    q_dyn: np.ndarray = None,
    missile_type: str = "Unknown",
    launch_angle: float = 0.0,
    **kwargs
) -> str:
    """
    표준 NPZ 포맷으로 궤적 데이터 저장
    
    NOTE: 가능하면 save_trajectory_unified()를 사용하세요.
    """
    # numpy 배열로 변환
    time = np.asarray(time)
    position_x = np.asarray(position_x)
    position_y = np.asarray(position_y)
    position_z = np.asarray(position_z)
    u = np.asarray(u)
    v = np.asarray(v)
    w = np.asarray(w)
    phi = np.asarray(phi)
    theta = np.asarray(theta)
    psi = np.asarray(psi)
    p = np.asarray(p)
    q = np.asarray(q)
    r = np.asarray(r)
    mass = np.asarray(mass)
    
    # 데이터 무결성 검증
    arrays = {
        'time': time, 'position_x': position_x, 'position_y': position_y,
        'position_z': position_z, 'u': u, 'v': v, 'w': w,
        'phi': phi, 'theta': theta, 'psi': psi,
        'p': p, 'q': q, 'r': r, 'mass': mass
    }
    
    n_points = len(time)
    
    # 검증
    errors = []
    for name, arr in arrays.items():
        if len(arr) != n_points:
            errors.append(f"배열 길이 불일치: {name} ({len(arr)}) != time ({n_points})")
        if np.any(np.isnan(arr)):
            nan_count = np.sum(np.isnan(arr))
            errors.append(f"NaN 값 발견: {name} ({nan_count}개)")
        if np.any(np.isinf(arr)):
            inf_count = np.sum(np.isinf(arr))
            errors.append(f"Inf 값 발견: {name} ({inf_count}개)")
    
    if errors:
        error_msg = "데이터 검증 실패:\n" + "\n".join(f"  - {e}" for e in errors)
        raise ValueError(error_msg)
    
    # Trajectory Frame 자동 계산
    if V is None:
        V = np.sqrt(u**2 + v**2 + w**2)
    else:
        V = np.asarray(V)
    
    if gamma is None:
        gamma = np.arcsin(np.clip(w / np.maximum(V, 0.1), -1, 1))
    else:
        gamma = np.asarray(gamma)
    
    if chi is None:
        chi = psi.copy()
    else:
        chi = np.asarray(chi)
    
    # 시그니처 분석용 데이터 자동 계산
    if a_x is None:
        V_x = np.gradient(position_x, time)
        a_x = np.gradient(V_x, time)
    if a_y is None:
        a_y = np.zeros_like(time)
    if a_z is None:
        V_z = np.gradient(position_z, time)
        a_z = np.gradient(V_z, time)
    
    if E_s is None:
        E_s = position_z + (V**2) / (2 * G0)
    
    if q_dyn is None:
        q_dyn = np.zeros_like(time)
        for i in range(len(time)):
            h = position_z[i]
            if h < 11000:
                T = 288.15 - 0.0065 * h
                P = 101325 * (T / 288.15) ** 5.2561
            else:
                T = 216.65
                P = 22632 * np.exp(-0.00015769 * (h - 11000))
            rho = P / (287.05 * max(T, 1))
            q_dyn[i] = 0.5 * rho * V[i]**2
    
    # 저장 데이터 구성
    save_dict = {
        'time': time,
        'position_x': position_x,
        'position_y': position_y,
        'position_z': position_z,
        'u': u, 'v': v, 'w': w,
        'phi': phi, 'theta': theta, 'psi': psi,
        'p': p, 'q': q, 'r': r,
        'mass': mass,
        'V': V, 'gamma': gamma, 'chi': chi,
        'a_x': a_x, 'a_y': a_y, 'a_z': a_z,
        'E_s': E_s,
        'q_dyn': q_dyn,
        'missile_type': missile_type,
        'launch_angle': launch_angle,
        'generation_time': datetime.now().isoformat(),
        'n_points': n_points,
        'duration': float(time[-1] - time[0]),
    }
    
    # 추가 메타데이터 병합
    for key, value in kwargs.items():
        if value is not None:
            if isinstance(value, np.ndarray):
                save_dict[key] = value
            else:
                save_dict[key] = value
    
    # 디렉토리 생성
    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    
    # NPZ 저장 (압축)
    _save_npz_compressed(filepath, save_dict)
    
    print(f"✓ 궤적 데이터 저장 완료: {filepath}")
    print(f"  - 미사일: {missile_type}, 발사각: {launch_angle}°")
    print(f"  - 데이터 포인트: {n_points}, 시간: {time[-1]:.1f}s")
    
    return filepath


def _save_npz_compressed(filepath: str, data: Dict[str, Any]):
    """내부용 NPZ 저장 (np.savez_compressed 래핑)"""
    np.savez_compressed(filepath, **data)


# =============================================================================
# 로드 함수
# =============================================================================

def load_trajectory(filepath: str, validate: bool = True) -> Dict[str, Any]:
    """
    NPZ 파일에서 궤적 데이터 로드
    
    Parameters:
    -----------
    filepath : str
        NPZ 파일 경로
    validate : bool
        로드 후 검증 수행 여부
    
    Returns:
    --------
    data : dict
        모든 배열 및 메타데이터를 포함하는 딕셔너리
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"파일을 찾을 수 없음: {filepath}")
    
    npz_file = np.load(str(filepath), allow_pickle=True)
    data = {key: npz_file[key] for key in npz_file.files}
    npz_file.close()
    
    # 필수 필드 검증
    missing = [f for f in REQUIRED_KEYS if f not in data]
    if missing:
        raise ValueError(f"필수 필드 누락: {missing}")
    
    # 선택적 검증
    if validate:
        validate_trajectory(data, raise_on_error=False)
    
    # 메타데이터 추출
    missile_type = data.get('missile_type', 'Unknown')
    if isinstance(missile_type, np.ndarray):
        missile_type = str(missile_type)
    n_points = data.get('n_points', len(data['time']))
    if isinstance(n_points, np.ndarray):
        n_points = int(n_points)
    
    print(f"✓ 궤적 데이터 로드 완료: {filepath}")
    print(f"  - 미사일: {missile_type}")
    print(f"  - 데이터 포인트: {n_points}")
    
    return data


# =============================================================================
# 검증 함수 (강화됨)
# =============================================================================

def validate_trajectory(
    data: Dict[str, Any], 
    raise_on_error: bool = False
) -> bool:
    """
    로드된 데이터의 물리적 정합성 검증
    
    검증 항목:
    1. 필수 키 존재 여부
    2. 배열 길이 일치
    3. NaN/Inf 값 체크
    4. 속도 크기 일치 (Body vs Trajectory)
    5. 각도 범위 검증
    6. 고도 음수 체크
    7. 시간 간격 균일성
    
    Parameters:
    -----------
    data : dict
        검증할 데이터 딕셔너리
    raise_on_error : bool
        True면 오류 시 예외 발생, False면 경고만 출력
    
    Returns:
    --------
    valid : bool
        검증 통과 여부
    """
    issues = []
    
    # 1. 필수 키 검증
    missing_keys = [k for k in REQUIRED_KEYS if k not in data]
    if missing_keys:
        issues.append(f"필수 키 누락: {missing_keys}")
    
    if missing_keys:
        # 필수 키 없으면 더 이상 검증 불가
        _report_validation_issues(issues, raise_on_error)
        return False
    
    # 2. 배열 길이 일치 검증
    n_points = len(data['time'])
    for key in REQUIRED_KEYS:
        if key in data:
            arr = np.asarray(data[key])
            if len(arr) != n_points:
                issues.append(f"배열 길이 불일치: {key} ({len(arr)}) != time ({n_points})")
    
    # 3. NaN/Inf 검증
    for key in REQUIRED_KEYS:
        if key in data:
            arr = np.asarray(data[key])
            nan_count = np.sum(np.isnan(arr))
            inf_count = np.sum(np.isinf(arr))
            if nan_count > 0:
                issues.append(f"NaN 값 발견: {key} ({nan_count}개, 인덱스: {np.where(np.isnan(arr))[0][:5]}...)")
            if inf_count > 0:
                issues.append(f"Inf 값 발견: {key} ({inf_count}개, 인덱스: {np.where(np.isinf(arr))[0][:5]}...)")
    
    # 4. 속도 크기 일치 확인
    if 'V' in data:
        u, v, w = data['u'], data['v'], data['w']
        V_body = np.sqrt(np.asarray(u)**2 + np.asarray(v)**2 + np.asarray(w)**2)
        V_traj = np.asarray(data['V'])
        velocity_error = np.abs(V_body - V_traj).max()
        if velocity_error > 10.0:  # 10 m/s 이상 차이
            issues.append(f"속도 크기 불일치: 최대 오차 {velocity_error:.2f} m/s")
    
    # 5. 각도 범위 확인
    if 'gamma' in data:
        gamma = np.asarray(data['gamma'])
        if np.any(np.abs(gamma) > np.pi + 0.1):
            issues.append(f"비행경로각 범위 초과: min={np.rad2deg(gamma.min()):.1f}°, max={np.rad2deg(gamma.max()):.1f}°")
    
    # 6. 고도 음수 확인
    pos_z = np.asarray(data['position_z'])
    if np.any(pos_z < -1000):  # 1km 이하로 음수면 경고
        min_alt = pos_z.min()
        issues.append(f"고도가 비정상 음수: 최소 {min_alt:.1f}m")
    
    # 7. 시간 간격 균일성
    time = np.asarray(data['time'])
    dt = np.diff(time)
    if len(dt) > 0:
        dt_mean = dt.mean()
        dt_std = dt.std()
        if dt_mean > 0 and dt_std > dt_mean * 0.5:  # 50% 이상 변동
            issues.append(f"시간 간격 불균일: mean={dt_mean:.4f}s, std={dt_std:.4f}s")
    
    # 결과 보고
    return _report_validation_issues(issues, raise_on_error)


def _report_validation_issues(issues: List[str], raise_on_error: bool) -> bool:
    """검증 결과 보고"""
    if issues:
        msg = "데이터 검증 실패:\n" + "\n".join(f"  - {issue}" for issue in issues)
        if raise_on_error:
            raise ValueError(msg)
        else:
            print(f"⚠ {msg}")
        return False
    else:
        print("✓ 데이터 검증 통과")
        return True


# =============================================================================
# 유틸리티 함수
# =============================================================================

def list_trajectory_files(directory: Optional[Union[str, Path]] = None) -> List[Path]:
    """지정 디렉토리의 모든 trajectory NPZ 파일 목록 반환"""
    if directory is None:
        directory = DEFAULT_OUTPUT_DIR
    directory = Path(directory)
    
    if not directory.exists():
        return []
    
    return sorted(directory.glob("*.npz"), key=lambda x: x.stat().st_mtime, reverse=True)


def get_trajectory_info(filepath: Union[str, Path]) -> Dict[str, Any]:
    """NPZ 파일의 메타정보만 빠르게 추출"""
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"파일을 찾을 수 없음: {filepath}")
    
    npz = np.load(str(filepath), allow_pickle=True)
    info = {
        'filepath': str(filepath),
        'filename': filepath.name,
        'filesize_kb': filepath.stat().st_size / 1024,
        'keys': list(npz.files),
        'n_points': int(npz['n_points']) if 'n_points' in npz.files else len(npz['time']),
        'missile_type': str(npz['missile_type']) if 'missile_type' in npz.files else 'Unknown',
        'launch_angle': float(npz['launch_angle']) if 'launch_angle' in npz.files else 0.0,
    }
    npz.close()
    return info


# =============================================================================
# 범용 NPZ 저장 함수 (시그니처 데이터 등 비-trajectory용)
# =============================================================================

def save_npz_generic(filepath: Union[str, Path], data: Dict[str, Any], compressed: bool = True):
    """
    범용 NPZ 저장 함수 (trajectory 외 데이터용)
    
    np.savez/np.savez_compressed의 표준 래퍼.
    레포 전체에서 np.savez 직접 호출을 제거하기 위해 사용.
    
    Parameters:
    -----------
    filepath : str or Path
        저장 경로
    data : dict
        저장할 데이터 딕셔너리
    compressed : bool
        압축 여부 (기본 True)
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    if compressed:
        np.savez_compressed(str(filepath), **data)
    else:
        np.savez(str(filepath), **data)


def load_npz_generic(filepath: Union[str, Path]) -> Dict[str, Any]:
    """
    범용 NPZ 로드 함수
    
    Parameters:
    -----------
    filepath : str or Path
        로드할 파일 경로
    
    Returns:
    --------
    data : dict
        로드된 데이터 딕셔너리
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"파일을 찾을 수 없음: {filepath}")
    
    npz = np.load(str(filepath), allow_pickle=True)
    data = {key: npz[key] for key in npz.files}
    npz.close()
    return data
