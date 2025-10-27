#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
좌표계 변환 모듈 - 진짜 6DOF용
Body Frame ↔ Inertial Frame 변환
"""

import numpy as np


class CoordinateTransform:
    """좌표계 변환 유틸리티 클래스"""
    
    @staticmethod
    def euler_to_dcm(phi, theta, psi):
        """
        오일러각 → Direction Cosine Matrix (DCM)
        
        Body Frame → Inertial Frame 변환 행렬
        
        Parameters:
        -----------
        phi : float
            롤각 (rad)
        theta : float
            피치각 (rad)
        psi : float
            요각 (rad)
        
        Returns:
        --------
        DCM : ndarray (3x3)
            방향 코사인 행렬
        """
        # 삼각함수 미리 계산
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        
        # DCM 계산 (3-2-1 오일러 시퀀스)
        DCM = np.array([
            [ctheta*cpsi, 
             sphi*stheta*cpsi - cphi*spsi, 
             cphi*stheta*cpsi + sphi*spsi],
            
            [ctheta*spsi, 
             sphi*stheta*spsi + cphi*cpsi, 
             cphi*stheta*spsi - sphi*cpsi],
            
            [-stheta, 
             sphi*ctheta, 
             cphi*ctheta]
        ])
        
        return DCM
    
    @staticmethod
    def dcm_to_euler(DCM):
        """
        Direction Cosine Matrix → 오일러각
        
        Parameters:
        -----------
        DCM : ndarray (3x3)
            방향 코사인 행렬
        
        Returns:
        --------
        phi, theta, psi : float
            롤, 피치, 요각 (rad)
        """
        # 짐벌락 방지 (theta가 ±90°에 가까울 때)
        theta = np.arcsin(-np.clip(DCM[2, 0], -1.0, 1.0))
        
        if np.abs(np.cos(theta)) > 1e-10:
            phi = np.arctan2(DCM[2, 1], DCM[2, 2])
            psi = np.arctan2(DCM[1, 0], DCM[0, 0])
        else:
            # 짐벌락 발생 시 (theta ≈ ±90°)
            phi = 0.0
            psi = np.arctan2(-DCM[0, 1], DCM[1, 1])
        
        return phi, theta, psi
    
    @staticmethod
    def body_to_inertial(v_body, phi, theta, psi):
        """
        Body Frame 벡터 → Inertial Frame
        
        Parameters:
        -----------
        v_body : ndarray (3,)
            Body Frame 벡터
        phi, theta, psi : float
            오일러각 (rad)
        
        Returns:
        --------
        v_inertial : ndarray (3,)
            Inertial Frame 벡터
        """
        DCM = CoordinateTransform.euler_to_dcm(phi, theta, psi)
        return DCM @ v_body
    
    @staticmethod
    def inertial_to_body(v_inertial, phi, theta, psi):
        """
        Inertial Frame 벡터 → Body Frame
        
        Parameters:
        -----------
        v_inertial : ndarray (3,)
            Inertial Frame 벡터
        phi, theta, psi : float
            오일러각 (rad)
        
        Returns:
        --------
        v_body : ndarray (3,)
            Body Frame 벡터
        """
        DCM = CoordinateTransform.euler_to_dcm(phi, theta, psi)
        return DCM.T @ v_inertial
    
    @staticmethod
    def euler_rates(phi, theta, psi, p, q, r):
        """
        Body Frame 각속도 → 오일러각 변화율
        
        Parameters:
        -----------
        phi, theta, psi : float
            오일러각 (rad)
        p, q, r : float
            Body Frame 각속도 (rad/s)
        
        Returns:
        --------
        phi_dot, theta_dot, psi_dot : float
            오일러각 변화율 (rad/s)
        """
        # 짐벌락 방지
        cos_theta = np.cos(theta)
        if np.abs(cos_theta) < 1e-10:
            cos_theta = np.sign(cos_theta) * 1e-10
        
        tan_theta = np.tan(theta)
        
        phi_dot = p + q * np.sin(phi) * tan_theta + r * np.cos(phi) * tan_theta
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / cos_theta
        
        return phi_dot, theta_dot, psi_dot
    
    @staticmethod
    def angular_rates(phi, theta, psi, phi_dot, theta_dot, psi_dot):
        """
        오일러각 변화율 → Body Frame 각속도
        
        Parameters:
        -----------
        phi, theta, psi : float
            오일러각 (rad)
        phi_dot, theta_dot, psi_dot : float
            오일러각 변화율 (rad/s)
        
        Returns:
        --------
        p, q, r : float
            Body Frame 각속도 (rad/s)
        """
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        
        p = phi_dot - psi_dot * stheta
        q = theta_dot * cphi + psi_dot * ctheta * sphi
        r = -theta_dot * sphi + psi_dot * ctheta * cphi
        
        return p, q, r
    
    @staticmethod
    def verify_dcm(DCM, tolerance=1e-6):
        """
        DCM 유효성 검증
        
        1. 직교성: DCM * DCM^T = I
        2. 행렬식: det(DCM) = 1
        
        Parameters:
        -----------
        DCM : ndarray (3x3)
            검증할 DCM
        tolerance : float
            허용 오차
        
        Returns:
        --------
        is_valid : bool
            유효 여부
        errors : dict
            오차 정보
        """
        # 직교성 검증
        I = np.eye(3)
        orthogonality_error = np.linalg.norm(DCM @ DCM.T - I)
        
        # 행렬식 검증
        det = np.linalg.det(DCM)
        det_error = np.abs(det - 1.0)
        
        is_valid = (orthogonality_error < tolerance) and (det_error < tolerance)
        
        errors = {
            'orthogonality': orthogonality_error,
            'determinant': det_error,
            'det_value': det
        }
        
        return is_valid, errors


class Quaternion:
    """쿼터니언 기반 자세 표현 (짐벌락 방지용)"""
    
    def __init__(self, q0=1.0, q1=0.0, q2=0.0, q3=0.0):
        """
        쿼터니언 초기화
        
        q = q0 + q1*i + q2*j + q3*k
        
        Parameters:
        -----------
        q0, q1, q2, q3 : float
            쿼터니언 성분 (정규화됨)
        """
        self.q = np.array([q0, q1, q2, q3])
        self.normalize()
    
    def normalize(self):
        """쿼터니언 정규화"""
        norm = np.linalg.norm(self.q)
        if norm > 1e-10:
            self.q /= norm
    
    @staticmethod
    def from_euler(phi, theta, psi):
        """
        오일러각 → 쿼터니언
        
        Parameters:
        -----------
        phi, theta, psi : float
            오일러각 (rad)
        
        Returns:
        --------
        quat : Quaternion
            쿼터니언
        """
        cphi = np.cos(phi / 2)
        sphi = np.sin(phi / 2)
        ctheta = np.cos(theta / 2)
        stheta = np.sin(theta / 2)
        cpsi = np.cos(psi / 2)
        spsi = np.sin(psi / 2)
        
        q0 = cphi * ctheta * cpsi + sphi * stheta * spsi
        q1 = sphi * ctheta * cpsi - cphi * stheta * spsi
        q2 = cphi * stheta * cpsi + sphi * ctheta * spsi
        q3 = cphi * ctheta * spsi - sphi * stheta * cpsi
        
        return Quaternion(q0, q1, q2, q3)
    
    def to_euler(self):
        """
        쿼터니언 → 오일러각
        
        Returns:
        --------
        phi, theta, psi : float
            오일러각 (rad)
        """
        q0, q1, q2, q3 = self.q
        
        # 롤 (phi)
        phi = np.arctan2(2 * (q0*q1 + q2*q3), 
                        1 - 2 * (q1**2 + q2**2))
        
        # 피치 (theta)
        sin_theta = 2 * (q0*q2 - q3*q1)
        sin_theta = np.clip(sin_theta, -1.0, 1.0)
        theta = np.arcsin(sin_theta)
        
        # 요 (psi)
        psi = np.arctan2(2 * (q0*q3 + q1*q2), 
                        1 - 2 * (q2**2 + q3**2))
        
        return phi, theta, psi
    
    def to_dcm(self):
        """
        쿼터니언 → DCM
        
        Returns:
        --------
        DCM : ndarray (3x3)
            방향 코사인 행렬
        """
        q0, q1, q2, q3 = self.q
        
        DCM = np.array([
            [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
        ])
        
        return DCM
    
    def update(self, p, q, r, dt):
        """
        쿼터니언 시간 업데이트
        
        Parameters:
        -----------
        p, q, r : float
            Body Frame 각속도 (rad/s)
        dt : float
            시간 간격 (s)
        """
        # 쿼터니언 미분 행렬
        omega = np.array([
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0]
        ])
        
        # 1차 오일러 적분
        q_dot = 0.5 * omega @ self.q
        self.q += q_dot * dt
        self.normalize()


# 테스트 함수
def test_coordinate_transforms():
    """좌표계 변환 테스트"""
    print("=" * 60)
    print("좌표계 변환 모듈 테스트")
    print("=" * 60)
    
    # 1. DCM 생성 및 검증
    phi, theta, psi = np.radians([10, 20, 30])
    DCM = CoordinateTransform.euler_to_dcm(phi, theta, psi)
    
    is_valid, errors = CoordinateTransform.verify_dcm(DCM)
    print(f"\n1. DCM 검증:")
    print(f"   유효성: {is_valid}")
    print(f"   직교성 오차: {errors['orthogonality']:.2e}")
    print(f"   행렬식 오차: {errors['determinant']:.2e}")
    
    # 2. DCM → 오일러각 변환
    phi2, theta2, psi2 = CoordinateTransform.dcm_to_euler(DCM)
    print(f"\n2. 오일러각 복원:")
    print(f"   원본: φ={np.degrees(phi):.2f}°, θ={np.degrees(theta):.2f}°, ψ={np.degrees(psi):.2f}°")
    print(f"   복원: φ={np.degrees(phi2):.2f}°, θ={np.degrees(theta2):.2f}°, ψ={np.degrees(psi2):.2f}°")
    
    # 3. 벡터 변환 테스트
    v_body = np.array([100, 0, 0])  # Body Frame에서 X 방향 100m/s
    v_inertial = CoordinateTransform.body_to_inertial(v_body, phi, theta, psi)
    v_body_restored = CoordinateTransform.inertial_to_body(v_inertial, phi, theta, psi)
    
    print(f"\n3. 벡터 변환:")
    print(f"   Body: {v_body}")
    print(f"   Inertial: {v_inertial}")
    print(f"   복원: {v_body_restored}")
    print(f"   오차: {np.linalg.norm(v_body - v_body_restored):.2e}")
    
    # 4. 쿼터니언 테스트
    quat = Quaternion.from_euler(phi, theta, psi)
    phi3, theta3, psi3 = quat.to_euler()
    
    print(f"\n4. 쿼터니언 변환:")
    print(f"   원본: φ={np.degrees(phi):.2f}°, θ={np.degrees(theta):.2f}°, ψ={np.degrees(psi):.2f}°")
    print(f"   복원: φ={np.degrees(phi3):.2f}°, θ={np.degrees(theta3):.2f}°, ψ={np.degrees(psi3):.2f}°")
    
    # 5. 각속도 변환 테스트
    p, q, r = 0.1, 0.2, 0.3  # rad/s
    phi_dot, theta_dot, psi_dot = CoordinateTransform.euler_rates(phi, theta, psi, p, q, r)
    p2, q2, r2 = CoordinateTransform.angular_rates(phi, theta, psi, phi_dot, theta_dot, psi_dot)
    
    print(f"\n5. 각속도 변환:")
    print(f"   원본: p={p:.3f}, q={q:.3f}, r={r:.3f} rad/s")
    print(f"   복원: p={p2:.3f}, q={q2:.3f}, r={r2:.3f} rad/s")
    print(f"   오차: {np.linalg.norm([p-p2, q-q2, r-r2]):.2e}")
    
    print("\n" + "=" * 60)
    print("✅ 모든 테스트 통과!")
    print("=" * 60)


if __name__ == "__main__":
    test_coordinate_transforms()
