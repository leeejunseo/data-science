"""
NPZ 궤적 데이터 포맷 어댑터
서로 다른 저장 포맷을 trajectory_io.py 표준 포맷으로 자동 변환

지원 포맷:
1. 표준 포맷 (trajectory_io.py): position_x/y/z, u/v/w, phi/theta/psi, p/q/r
2. 3DOF 포맷 (missile_6dof.py): V/gamma/psi, altitude (position_z 대신)
3. 6DOF v2 포맷 (missile_6dof_v2.py): Body Frame 완전 지원
"""

import numpy as np
from typing import Dict, Tuple
from pathlib import Path


class TrajectoryFormatAdapter:
    """궤적 데이터 포맷 변환 어댑터"""
    
    @staticmethod
    def detect_format(data: dict) -> str:
        """
        NPZ 데이터 포맷 자동 감지
        
        Returns:
        --------
        format_type : str
            'standard' | '3dof' | '6dof_v2' | 'unknown'
        """
        has_position_z = 'position_z' in data
        has_altitude = 'altitude' in data
        has_uvw = 'u' in data and 'v' in data and 'w' in data
        has_V_gamma = 'V' in data and 'gamma' in data
        has_pqr = 'p' in data and 'q' in data and 'r' in data
        
        # 표준 포맷 (trajectory_io.py)
        if has_position_z and has_uvw and has_pqr:
            return 'standard'
        
        # 6DOF v2 포맷 (missile_6dof_v2.py)
        # position_x/y/z는 있지만 altitude는 없음
        elif has_position_z and has_uvw and not has_altitude:
            return '6dof_v2'
        
        # 3DOF 포맷 (missile_6dof.py)
        # altitude, V/gamma/psi, u/v/w 없음
        elif has_altitude and has_V_gamma and not has_uvw:
            return '3dof'
        
        else:
            return 'unknown'
    
    @staticmethod
    def convert_to_standard(data: dict) -> dict:
        """
        임의 포맷을 표준 포맷으로 변환
        
        Parameters:
        -----------
        data : dict
            로드된 NPZ 데이터
        
        Returns:
        --------
        standard_data : dict
            표준 포맷으로 변환된 데이터
        """
        format_type = TrajectoryFormatAdapter.detect_format(data)
        
        if format_type == 'standard':
            print("✓ 이미 표준 포맷")
            return data
        
        elif format_type == '3dof':
            print("✓ 3DOF → 표준 포맷 변환 중...")
            return TrajectoryFormatAdapter._convert_3dof_to_standard(data)
        
        elif format_type == '6dof_v2':
            print("✓ 6DOF v2 → 표준 포맷 변환 중...")
            return TrajectoryFormatAdapter._convert_6dof_v2_to_standard(data)
        
        else:
            raise ValueError(f"알 수 없는 포맷: {list(data.keys())}")
    
    @staticmethod
    def _convert_3dof_to_standard(data: dict) -> dict:
        """
        3DOF 포맷 → 표준 포맷
        
        변환:
        - altitude → position_z
        - V, gamma, psi → u, v, w (Body Frame 근사)
        - theta, phi 없으면 생성
        """
        standard = data.copy()
        
        # 1. altitude → position_z
        if 'altitude' in data:
            standard['position_z'] = data['altitude']
            print(f"  - altitude → position_z")
        
        # 2. u, v, w 계산 (Trajectory Frame → Body Frame 근사)
        if 'V' in data and 'gamma' in data and 'psi' in data:
            V = data['V']
            gamma = data['gamma']
            psi = data['psi']
            
            # 간단한 근사 (theta ≈ gamma 가정)
            # Body Frame: u ≈ V*cos(gamma)*cos(psi), v ≈ V*cos(gamma)*sin(psi), w ≈ V*sin(gamma)
            theta = data.get('theta', gamma)  # theta 없으면 gamma 사용
            
            standard['u'] = V * np.cos(gamma)
            standard['v'] = np.zeros_like(V)  # 측면 속도 0으로 근사
            standard['w'] = V * np.sin(gamma)
            print(f"  - V/gamma/psi → u/v/w (근사)")
        
        # 3. phi, theta 없으면 생성
        if 'phi' not in standard:
            standard['phi'] = np.zeros_like(data['time'])
            print(f"  - phi 생성 (0으로 초기화)")
        
        if 'theta' not in standard:
            if 'gamma' in data:
                standard['theta'] = data['gamma']  # theta ≈ gamma
                print(f"  - theta = gamma (근사)")
            else:
                standard['theta'] = np.zeros_like(data['time'])
        
        # 4. psi 확인
        if 'psi' not in standard:
            standard['psi'] = np.zeros_like(data['time'])
            print(f"  - psi 생성 (0으로 초기화)")
        
        # 5. p, q, r 없으면 생성
        for angular_rate in ['p', 'q', 'r']:
            if angular_rate not in standard:
                standard[angular_rate] = np.zeros_like(data['time'])
                print(f"  - {angular_rate} 생성 (0으로 초기화)")
        
        # 6. V, gamma, chi 확인 (표준 포맷 요구사항)
        if 'V' not in standard:
            if 'u' in standard and 'v' in standard and 'w' in standard:
                standard['V'] = np.sqrt(standard['u']**2 + standard['v']**2 + standard['w']**2)
        
        if 'chi' not in standard:
            standard['chi'] = standard.get('psi', np.zeros_like(data['time']))
        
        return standard
    
    @staticmethod
    def _convert_6dof_v2_to_standard(data: dict) -> dict:
        """
        6DOF v2 포맷 → 표준 포맷
        
        이미 대부분 표준이지만, 누락된 필드 보완
        """
        standard = data.copy()
        
        # V, gamma, chi 자동 계산
        if 'V' not in standard and 'u' in data and 'v' in data and 'w' in data:
            u, v, w = data['u'], data['v'], data['w']
            standard['V'] = np.sqrt(u**2 + v**2 + w**2)
            print(f"  - V 계산")
        
        if 'gamma' not in standard:
            # gamma 계산 (Inertial Frame 필요)
            if 'theta' in data and 'alpha' in data:
                # gamma ≈ theta - alpha (근사)
                standard['gamma'] = data['theta'] - data.get('alpha', 0)
                print(f"  - gamma ≈ theta - alpha")
            else:
                standard['gamma'] = np.zeros_like(data['time'])
                print(f"  - gamma 생성 (0으로 초기화)")
        
        if 'chi' not in standard:
            standard['chi'] = data.get('psi', np.zeros_like(data['time']))
            print(f"  - chi = psi")
        
        return standard
    
    @staticmethod
    def load_and_convert(filepath: str) -> dict:
        """
        NPZ 파일 로드 + 자동 포맷 변환
        
        Parameters:
        -----------
        filepath : str
            NPZ 파일 경로
        
        Returns:
        --------
        standard_data : dict
            표준 포맷 데이터
        """
        if not Path(filepath).exists():
            raise FileNotFoundError(f"파일을 찾을 수 없음: {filepath}")
        
        # 로드
        npz_file = np.load(filepath, allow_pickle=True)
        data = {key: npz_file[key] for key in npz_file.files}
        
        print(f"✓ NPZ 로드: {filepath}")
        print(f"  - 포맷 감지 중...")
        
        # 포맷 변환
        standard_data = TrajectoryFormatAdapter.convert_to_standard(data)
        
        print(f"✓ 표준 포맷 변환 완료")
        return standard_data
    
    @staticmethod
    def validate_standard_format(data: dict) -> Tuple[bool, list]:
        """
        표준 포맷 필수 필드 검증
        
        Returns:
        --------
        is_valid : bool
            검증 통과 여부
        missing_fields : list
            누락된 필드 목록
        """
        required_fields = [
            'time', 'position_x', 'position_y', 'position_z',
            'u', 'v', 'w', 'phi', 'theta', 'psi',
            'p', 'q', 'r', 'mass', 'V', 'gamma', 'chi'
        ]
        
        missing = [f for f in required_fields if f not in data]
        
        if missing:
            print(f"⚠ 필수 필드 누락: {missing}")
            return False, missing
        else:
            print(f"✓ 표준 포맷 검증 통과 ({len(required_fields)}개 필드)")
            return True, []


def demo():
    """어댑터 데모"""
    print("="*60)
    print("NPZ 포맷 어댑터 데모")
    print("="*60)
    
    # 테스트 데이터 생성
    print("\n[테스트 1] 3DOF 포맷 변환")
    test_3dof = {
        'time': np.linspace(0, 100, 50),
        'V': np.ones(50) * 1000,
        'gamma': np.linspace(np.pi/2, 0, 50),
        'psi': np.zeros(50),
        'position_x': np.linspace(0, 50000, 50),
        'position_y': np.zeros(50),
        'altitude': np.linspace(0, 30000, 50),  # ← altitude!
        'phi': np.zeros(50),
        'theta': np.linspace(np.pi/2, 0, 50),
        'p': np.zeros(50),
        'q': np.zeros(50),
        'r': np.zeros(50),
        'mass': np.linspace(5000, 1000, 50),
    }
    
    adapter = TrajectoryFormatAdapter()
    fmt = adapter.detect_format(test_3dof)
    print(f"감지된 포맷: {fmt}")
    
    converted = adapter.convert_to_standard(test_3dof)
    is_valid, missing = adapter.validate_standard_format(converted)
    
    print(f"\n변환 결과:")
    print(f"  - 'altitude' → 'position_z': {'position_z' in converted}")
    print(f"  - 'u/v/w' 생성: {'u' in converted}")
    print(f"  - 검증: {is_valid}")


if __name__ == "__main__":
    demo()
