"""
6DOF 미사일 궤적 데이터 표준 I/O 모듈
chanjinkim 물리 엔진 출력 → NPZ 저장 → JJINleejunseo 시각화 입력
"""
import numpy as np
from datetime import datetime
from pathlib import Path


def save_trajectory(
    filepath: str,
    time: np.ndarray,
    # Inertial Frame 위치 (NED)
    position_x: np.ndarray,  # North (m)
    position_y: np.ndarray,  # East (m)
    position_z: np.ndarray,  # Altitude (m) or Down (-m) depending on convention
    # Body Frame 속도
    u: np.ndarray,
    v: np.ndarray,
    w: np.ndarray,
    # Euler 각도
    phi: np.ndarray,    # Roll (rad)
    theta: np.ndarray,  # Pitch (rad)
    psi: np.ndarray,    # Yaw (rad)
    # 각속도
    p: np.ndarray,  # Roll rate (rad/s)
    q: np.ndarray,  # Pitch rate (rad/s)
    r: np.ndarray,  # Yaw rate (rad/s)
    # 추가 물리량
    mass: np.ndarray,
    # Trajectory Frame (변환된 값)
    V: np.ndarray = None,      # 속도 크기 (m/s)
    gamma: np.ndarray = None,  # 비행경로각 (rad)
    chi: np.ndarray = None,    # 방위각 (rad)
    # 메타데이터
    missile_type: str = "Unknown",
    launch_angle: float = 0.0,
    **kwargs
):
    """
    표준 NPZ 포맷으로 궤적 데이터 저장 (True6DOF 호환)
    
    Parameters:
    -----------
    filepath : str
        저장 경로 (예: "results/SCUD-B_45deg.npz")
    time : array
        시간 배열 (s)
    position_x, y, z : array
        Inertial Frame 위치 (NED: North, East, Altitude) (m)
    u, v, w : array
        Body Frame 속도 (Forward, Right, Down) (m/s)
    phi, theta, psi : array
        Euler 각도 (Roll, Pitch, Yaw) (rad)
    p, q, r : array
        Body Frame 각속도 (rad/s)
    mass : array
        질량 (kg)
    V, gamma, chi : array, optional
        Trajectory Frame 속도 성분 (자동 계산 가능)
    missile_type : str
        미사일 종류 ("SCUD-B", "Nodong", "KN-23")
    launch_angle : float
        발사각 (deg)
    **kwargs : dict
        추가 메타데이터
        
    Notes:
    ------
    True6DOF (missile_6dof_true.py)는 다음 형식으로 데이터를 반환:
    - 위치: (x, y, z) where z = altitude (not down!)
    - 속도: (u, v, w) in Body Frame
    - 자세: Quaternion → Euler (phi, theta, psi) 변환됨
    """
    
    # 데이터 무결성 검증
    arrays = {
        'time': time, 'position_x': position_x, 'position_y': position_y,
        'position_z': position_z, 'u': u, 'v': v, 'w': w,
        'phi': phi, 'theta': theta, 'psi': psi,
        'p': p, 'q': q, 'r': r, 'mass': mass
    }
    
    n_points = len(time)
    for name, arr in arrays.items():
        if len(arr) != n_points:
            raise ValueError(f"배열 길이 불일치: {name} ({len(arr)}) != time ({n_points})")
        if np.any(np.isnan(arr)):
            raise ValueError(f"NaN 값 발견: {name}")
        if np.any(np.isinf(arr)):
            raise ValueError(f"Inf 값 발견: {name}")
    
    # Trajectory Frame 자동 계산 (제공되지 않은 경우)
    if V is None or gamma is None or chi is None:
        try:
            from coordinate_transforms import body_to_trajectory
            V_calc, gamma_calc, chi_calc = body_to_trajectory(u, v, w, phi, theta, psi)
            V = V_calc if V is None else V
            gamma = gamma_calc if gamma is None else gamma
            chi = chi_calc if chi is None else chi
        except ImportError:
            # 변환 함수 없으면 직접 계산
            V = np.sqrt(u**2 + v**2 + w**2) if V is None else V
            gamma = np.zeros_like(u) if gamma is None else gamma
            chi = np.zeros_like(u) if chi is None else chi
    
    # 저장 데이터 구성
    save_dict = {
        # 시간
        'time': time,
        # Inertial Frame
        'position_x': position_x,
        'position_y': position_y,
        'position_z': position_z,
        # Body Frame
        'u': u, 'v': v, 'w': w,
        # Euler 각도
        'phi': phi, 'theta': theta, 'psi': psi,
        # 각속도
        'p': p, 'q': q, 'r': r,
        # 질량
        'mass': mass,
        # Trajectory Frame
        'V': V, 'gamma': gamma, 'chi': chi,
        # 메타데이터
        'missile_type': missile_type,
        'launch_angle': launch_angle,
        'generation_time': datetime.now().isoformat(),
        'n_points': n_points,
        'duration': time[-1] - time[0],
    }
    
    # 추가 메타데이터 병합
    save_dict.update(kwargs)
    
    # 디렉토리 생성
    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    
    # NPZ 저장
    np.savez_compressed(filepath, **save_dict)
    print(f"✓ 궤적 데이터 저장 완료: {filepath}")
    print(f"  - 미사일: {missile_type}, 발사각: {launch_angle}°")
    print(f"  - 데이터 포인트: {n_points}, 시간: {time[-1]:.1f}s")


def load_trajectory(filepath: str) -> dict:
    """
    NPZ 파일에서 궤적 데이터 로드 (True6DOF 호환)
    
    Returns:
    --------
    data : dict
        모든 배열 및 메타데이터를 포함하는 딕셔너리
        
    Notes:
    ------
    로드된 데이터는 표준 6DOF 형식:
    - position_x, y, z: Inertial Frame 위치
    - u, v, w: Body Frame 속도
    - phi, theta, psi: Euler 각도
    - p, q, r: 각속도
    """
    if not Path(filepath).exists():
        raise FileNotFoundError(f"파일을 찾을 수 없음: {filepath}")
    
    npz_file = np.load(filepath, allow_pickle=True)
    data = {key: npz_file[key] for key in npz_file.files}
    
    # 필수 필드 검증
    required_fields = [
        'time', 'position_x', 'position_y', 'position_z',
        'u', 'v', 'w', 'phi', 'theta', 'psi',
        'p', 'q', 'r', 'mass', 'V', 'gamma', 'chi'
    ]
    
    missing = [f for f in required_fields if f not in data]
    if missing:
        raise ValueError(f"필수 필드 누락: {missing}")
    
    print(f"✓ 궤적 데이터 로드 완료: {filepath}")
    print(f"  - 미사일: {data.get('missile_type', 'Unknown')}")
    print(f"  - 데이터 포인트: {data['n_points']}")
    
    return data


def validate_trajectory(data: dict) -> bool:
    """
    로드된 데이터의 물리적 정합성 검증 (True6DOF)
    
    검증 항목:
    1. 속도 크기 일치 (Body vs Trajectory)
    2. 각도 범위 검증
    3. 고도 음수 체크
    4. 시간 간격 균일성
    
    Returns:
    --------
    valid : bool
        검증 통과 여부
    """
    issues = []
    
    # 1. 속도 크기 일치 확인
    V_body = np.sqrt(data['u']**2 + data['v']**2 + data['w']**2)
    V_traj = data['V']
    velocity_error = np.abs(V_body - V_traj).max()
    if velocity_error > 1.0:  # 1 m/s 이상 차이
        issues.append(f"속도 크기 불일치: 최대 오차 {velocity_error:.2f} m/s")
    
    # 2. 각도 범위 확인
    gamma = data['gamma']
    if np.any(np.abs(gamma) > np.pi/2 + 0.1):
        issues.append(f"비행경로각 범위 초과: {np.rad2deg(gamma.max()):.1f}°")
    
    # 3. 고도 음수 확인
    if np.any(data['position_z'] < -100):
        issues.append("고도가 음수로 발산")
    
    # 4. 시간 간격 균일성
    dt = np.diff(data['time'])
    dt_std = dt.std()
    if dt_std > dt.mean() * 0.1:  # 10% 이상 변동
        issues.append(f"시간 간격 불균일: std={dt_std:.4f}s")
    
    if issues:
        print("⚠ 데이터 검증 경고:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    else:
        print("✓ 데이터 검증 통과")
        return True
