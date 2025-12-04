"""
좌표계 변환 모듈
Body Frame ↔ Inertial Frame ↔ Trajectory Frame
"""
import numpy as np


def body_to_inertial(u, v, w, phi, theta, psi):
    """
    Body Frame 속도를 Inertial Frame 속도로 변환
    
    Parameters:
    -----------
    u, v, w : array
        Body Frame 속도 성분 (m/s)
    phi, theta, psi : array
        Euler 각도 (rad)
    
    Returns:
    --------
    vx, vy, vz : array
        Inertial Frame 속도 (North, East, Up)
    """
    # DCM (Direction Cosine Matrix) 구성
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_psi = np.cos(psi)
    sin_psi = np.sin(psi)
    
    # Body → Inertial 변환 행렬 적용
    vx = (cos_theta * cos_psi) * u + \
         (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * v + \
         (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * w
    
    vy = (cos_theta * sin_psi) * u + \
         (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * v + \
         (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * w
    
    vz = (-sin_theta) * u + \
         (sin_phi * cos_theta) * v + \
         (cos_phi * cos_theta) * w
    
    return vx, vy, vz


def body_to_trajectory(u, v, w, phi, theta, psi):
    """
    Body Frame 속도를 Trajectory Frame 파라미터로 변환
    
    Parameters:
    -----------
    u, v, w : array
        Body Frame 속도 성분 (m/s)
    phi, theta, psi : array
        Euler 각도 (rad)
    
    Returns:
    --------
    V : array
        속도 크기 (m/s)
    gamma : array
        비행경로각 (rad), 범위: [-π/2, π/2]
    chi : array
        방위각 (rad), 범위: [-π, π]
    """
    # 먼저 Inertial Frame으로 변환
    vx, vy, vz = body_to_inertial(u, v, w, phi, theta, psi)
    
    # 속도 크기
    V = np.sqrt(vx**2 + vy**2 + vz**2)
    
    # 비행경로각 (Flight Path Angle)
    # gamma = arcsin(vz / V)
    # 안전한 계산을 위해 클리핑
    gamma = np.arcsin(np.clip(vz / (V + 1e-10), -1.0, 1.0))
    
    # 방위각 (Heading Angle)
    # chi = arctan2(vy, vx)
    chi = np.arctan2(vy, vx)
    
    return V, gamma, chi


def inertial_to_body(vx, vy, vz, phi, theta, psi):
    """
    Inertial Frame 속도를 Body Frame 속도로 변환 (역변환)
    
    Parameters:
    -----------
    vx, vy, vz : array
        Inertial Frame 속도 (m/s)
    phi, theta, psi : array
        Euler 각도 (rad)
    
    Returns:
    --------
    u, v, w : array
        Body Frame 속도 성분
    """
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_psi = np.cos(psi)
    sin_psi = np.sin(psi)
    
    # Inertial → Body 변환 (DCM의 전치)
    u = (cos_theta * cos_psi) * vx + \
        (cos_theta * sin_psi) * vy + \
        (-sin_theta) * vz
    
    v = (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * vx + \
        (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * vy + \
        (sin_phi * cos_theta) * vz
    
    w = (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * vx + \
        (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * vy + \
        (cos_phi * cos_theta) * vz
    
    return u, v, w


def verify_transformation(u, v, w, phi, theta, psi, tolerance=1e-6):
    """
    좌표 변환의 정합성 검증 (왕복 변환 후 원본과 비교)
    
    Returns:
    --------
    valid : bool
        검증 통과 여부
    max_error : float
        최대 오차 (m/s)
    """
    # Body → Inertial → Body
    vx, vy, vz = body_to_inertial(u, v, w, phi, theta, psi)
    u_back, v_back, w_back = inertial_to_body(vx, vy, vz, phi, theta, psi)
    
    # 오차 계산
    error_u = np.abs(u - u_back)
    error_v = np.abs(v - v_back)
    error_w = np.abs(w - w_back)
    max_error = max(error_u.max(), error_v.max(), error_w.max())
    
    valid = max_error < tolerance
    
    if not valid:
        print(f"⚠ 좌표 변환 검증 실패: 최대 오차 {max_error:.2e} m/s")
    
    return valid, max_error


def calculate_alpha_beta(u, v, w):
    """
    받음각(α)과 옆미끄럼각(β) 계산
    
    Parameters:
    -----------
    u, v, w : array
        Body Frame 속도 (m/s)
    
    Returns:
    --------
    alpha : array
        받음각 (rad)
    beta : array
        옆미끄럼각 (rad)
    """
    V = np.sqrt(u**2 + v**2 + w**2)
    
    # 받음각: α = arctan(w / u)
    alpha = np.arctan2(w, u)
    
    # 옆미끄럼각: β = arcsin(v / V)
    beta = np.arcsin(np.clip(v / (V + 1e-10), -1.0, 1.0))
    
    return alpha, beta


def euler_rates(phi, theta, psi, p, q, r):
    """
    각속도를 Euler 각도 변화율로 변환
    
    Parameters:
    -----------
    phi, theta, psi : float or array
        Euler 각도 (rad)
    p, q, r : float or array
        각속도 (rad/s)
    
    Returns:
    --------
    phi_dot, theta_dot, psi_dot : float or array
        Euler 각도 변화율 (rad/s)
    """
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    tan_theta = np.tan(theta)
    
    phi_dot = p + sin_phi * tan_theta * q + cos_phi * tan_theta * r
    theta_dot = cos_phi * q - sin_phi * r
    psi_dot = (sin_phi / cos_theta) * q + (cos_phi / cos_theta) * r
    
    return phi_dot, theta_dot, psi_dot


# CoordinateTransform 클래스 (하위 호환성)
class CoordinateTransform:
    """좌표 변환 유틸리티 클래스 (정적 메서드)"""
    
    @staticmethod
    def body_to_inertial(v_body, phi, theta, psi):
        """
        Body Frame 벡터를 Inertial Frame으로 변환
        
        Parameters:
        -----------
        v_body : array-like
            Body Frame 벡터 [u, v, w]
        phi, theta, psi : float
            Euler 각도 (rad)
        
        Returns:
        --------
        v_inertial : ndarray
            Inertial Frame 벡터 [vx, vy, vz]
        """
        if isinstance(v_body, (list, tuple)):
            v_body = np.array(v_body)
        
        vx, vy, vz = body_to_inertial(v_body[0], v_body[1], v_body[2], phi, theta, psi)
        return np.array([vx, vy, vz])
    
    @staticmethod
    def inertial_to_body(v_inertial, phi, theta, psi):
        """
        Inertial Frame 벡터를 Body Frame으로 변환
        
        Parameters:
        -----------
        v_inertial : array-like
            Inertial Frame 벡터 [vx, vy, vz]
        phi, theta, psi : float
            Euler 각도 (rad)
        
        Returns:
        --------
        v_body : ndarray
            Body Frame 벡터 [u, v, w]
        """
        if isinstance(v_inertial, (list, tuple)):
            v_inertial = np.array(v_inertial)
        
        u, v, w = inertial_to_body(v_inertial[0], v_inertial[1], v_inertial[2], phi, theta, psi)
        return np.array([u, v, w])
    
    @staticmethod
    def euler_rates(phi, theta, psi, p, q, r):
        """
        각속도를 Euler 각도 변화율로 변환
        
        Parameters:
        -----------
        phi, theta, psi : float
            Euler 각도 (rad)
        p, q, r : float
            각속도 (rad/s)
        
        Returns:
        --------
        phi_dot, theta_dot, psi_dot : float
            Euler 각도 변화율 (rad/s)
        """
        return euler_rates(phi, theta, psi, p, q, r)
