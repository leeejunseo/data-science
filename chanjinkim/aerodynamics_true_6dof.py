#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
진짜 6DOF 공력 모델
Body Frame 기반, 자세 의존적 공력
"""

import numpy as np


class AerodynamicsTrue6DOF:
    """진짜 6DOF 공력 모델 클래스"""
    
    def __init__(self, missile_config):
        """
        공력 모델 초기화
        
        Parameters:
        -----------
        missile_config : dict
            미사일 설정 (직경, 길이, 날개 면적, 공력 계수 등)
        """
        self.diameter = missile_config.get('diameter', 0.88)
        self.length = missile_config.get('length', 10.94)
        self.reference_area = missile_config.get('reference_area', np.pi * (self.diameter/2)**2)
        self.wingspan = missile_config.get('wingspan', self.diameter * 2)  # 간단화
        
        # 공력 계수
        self.cd_base = missile_config.get('cd_base', 0.25)
        self.cl_alpha = missile_config.get('cl_alpha', 3.5)  # per rad
        self.cy_beta = missile_config.get('cy_beta', -0.5)  # per rad
        
        # 공력 모멘트 계수
        self.cm_alpha = missile_config.get('cm_alpha', -0.15)  # 피칭 모멘트
        self.cn_beta = missile_config.get('cn_beta', 0.1)  # 요잉 모멘트
        self.cl_p = missile_config.get('cl_p', -0.5)  # 롤 댐핑
        self.cm_q = missile_config.get('cm_q', -0.8)  # 피치 댐핑
        self.cn_r = missile_config.get('cn_r', -0.8)  # 요 댐핑
        
        # 항력 테이블 (마하수 의존)
        self.cd_table = self._create_cd_table()
    
    def _create_cd_table(self):
        """항력계수 테이블 생성"""
        mach_values = np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0])
        cd_values = np.array([0.30, 0.35, 0.45, 0.85, 0.65, 0.45, 0.40, 0.35, 0.32, 0.30])
        return {'mach': mach_values, 'cd': cd_values}
    
    def get_cd_base(self, mach):
        """
        마하수에 따른 기본 항력계수
        
        Parameters:
        -----------
        mach : float
            마하수
        
        Returns:
        --------
        cd_base : float
            기본 항력계수
        """
        return np.interp(mach, self.cd_table['mach'], self.cd_table['cd'])
    
    def calculate_angles_of_attack(self, u, v, w, V_mag):
        """
        Body Frame 속도로부터 받음각과 측면 받음각 계산
        
        Parameters:
        -----------
        u, v, w : float
            Body Frame 속도 성분 (m/s)
        V_mag : float
            속도 크기 (m/s)
        
        Returns:
        --------
        alpha : float
            받음각 (rad)
        beta : float
            측면 받음각 (rad)
        """
        # 받음각 (angle of attack)
        # alpha = atan2(w, u)
        if abs(u) > 1e-6:
            alpha = np.arctan2(w, u)
        else:
            alpha = 0.0
        
        # 측면 받음각 (sideslip angle)
        # beta = asin(v / V)
        if V_mag > 1e-6:
            beta = np.arcsin(np.clip(v / V_mag, -1.0, 1.0))
        else:
            beta = 0.0
        
        # 각도 제한 (물리적 타당성)
        alpha = np.clip(alpha, -np.pi/2, np.pi/2)
        beta = np.clip(beta, -np.pi/4, np.pi/4)
        
        return alpha, beta
    
    def calculate_aerodynamic_coefficients(self, mach, alpha, beta):
        """
        공력 계수 계산 (받음각 & 측면 받음각 의존)
        
        Parameters:
        -----------
        mach : float
            마하수
        alpha : float
            받음각 (rad)
        beta : float
            측면 받음각 (rad)
        
        Returns:
        --------
        C_D, C_Y, C_L : float
            항력, 측력, 양력 계수
        """
        # 1. 항력계수 (Drag)
        cd_base = self.get_cd_base(mach)
        cd_induced = 0.1 * alpha**2  # 유도 항력
        cd_beta = 0.05 * beta**2  # 측면 항력
        C_D = cd_base + cd_induced + cd_beta
        
        # 2. 양력계수 (Lift)
        C_L = self.cl_alpha * alpha
        
        # 3. 측력계수 (Side force)
        C_Y = self.cy_beta * beta
        
        return C_D, C_Y, C_L
    
    def calculate_aerodynamic_forces(self, rho, V_mag, u, v, w, mach):
        """
        공력 힘 계산 (Body Frame)
        
        Parameters:
        -----------
        rho : float
            대기 밀도 (kg/m³)
        V_mag : float
            속도 크기 (m/s)
        u, v, w : float
            Body Frame 속도 성분 (m/s)
        mach : float
            마하수
        
        Returns:
        --------
        F_aero_x, F_aero_y, F_aero_z : float
            Body Frame 공력 힘 (N)
        alpha, beta : float
            받음각, 측면 받음각 (rad)
        """
        # 받음각 계산
        alpha, beta = self.calculate_angles_of_attack(u, v, w, V_mag)
        
        # 동압
        q_dyn = 0.5 * rho * V_mag**2
        
        # 공력 계수
        C_D, C_Y, C_L = self.calculate_aerodynamic_coefficients(mach, alpha, beta)
        
        # 공력 힘 (Body Frame)
        # 항력 (D): -X 방향
        # 측력 (Y): +Y 방향
        # 양력 (L): -Z 방향 (Body Frame 관례)
        
        D = q_dyn * self.reference_area * C_D  # 항력
        Y = q_dyn * self.reference_area * C_Y  # 측력
        L = q_dyn * self.reference_area * C_L  # 양력
        
        # Body Frame 좌표계로 변환
        # (속도 방향과 Body X축이 다를 수 있음)
        F_aero_x = -D  # 항력은 항상 속도 반대 방향
        F_aero_y = Y   # 측력
        F_aero_z = -L  # 양력 (Body Frame에서 -Z)
        
        return F_aero_x, F_aero_y, F_aero_z, alpha, beta
    
    def calculate_aerodynamic_moments(self, rho, V_mag, alpha, beta, p, q, r):
        """
        공력 모멘트 계산 (Body Frame)
        
        Parameters:
        -----------
        rho : float
            대기 밀도 (kg/m³)
        V_mag : float
            속도 크기 (m/s)
        alpha, beta : float
            받음각, 측면 받음각 (rad)
        p, q, r : float
            Body Frame 각속도 (rad/s)
        
        Returns:
        --------
        L_aero, M_aero, N_aero : float
            Body Frame 공력 모멘트 (N·m)
        """
        # 동압
        q_dyn = 0.5 * rho * V_mag**2
        
        # 무차원 각속도 (댐핑 효과용)
        if V_mag > 1e-6:
            p_hat = p * self.wingspan / (2 * V_mag)
            q_hat = q * self.length / (2 * V_mag)
            r_hat = r * self.wingspan / (2 * V_mag)
        else:
            p_hat = q_hat = r_hat = 0.0
        
        # 모멘트 계수
        # 롤 모멘트 (L): 측면 받음각 & 롤 댐핑
        C_l = self.cl_p * p_hat
        
        # 피칭 모멘트 (M): 받음각 & 피치 댐핑
        C_m = self.cm_alpha * alpha + self.cm_q * q_hat
        
        # 요잉 모멘트 (N): 측면 받음각 & 요 댐핑
        C_n = self.cn_beta * beta + self.cn_r * r_hat
        
        # 공력 모멘트 (Body Frame)
        L_aero = q_dyn * self.reference_area * self.wingspan * C_l
        M_aero = q_dyn * self.reference_area * self.length * C_m
        N_aero = q_dyn * self.reference_area * self.wingspan * C_n
        
        # 모멘트 제한 (수치 안정성)
        max_moment = 1e7
        L_aero = np.clip(L_aero, -max_moment, max_moment)
        M_aero = np.clip(M_aero, -max_moment, max_moment)
        N_aero = np.clip(N_aero, -max_moment, max_moment)
        
        return L_aero, M_aero, N_aero
    
    def get_atmospheric_density(self, altitude):
        """
        표준 대기 모델 - 대기 밀도
        
        Parameters:
        -----------
        altitude : float
            고도 (m)
        
        Returns:
        --------
        rho : float
            대기 밀도 (kg/m³)
        """
        if altitude < 0:
            return 1.225
        
        # 11km 이하 (대류권)
        if altitude <= 11000:
            T = 288.15 - 0.0065 * altitude
            P = 101325 * (T / 288.15)**5.2561
        # 11-20km (성층권 하부)
        elif altitude <= 20000:
            T = 216.65
            P = 22632.1 * np.exp(-0.00015768 * (altitude - 11000))
        # 20km 이상
        else:
            T = 216.65 + 0.001 * (altitude - 20000)
            P = 5474.89 * (T / 216.65)**(-34.163)
        
        R = 287.0531  # 공기 기체상수
        rho = P / (R * T)
        
        return rho
    
    def get_sound_speed(self, altitude):
        """
        음속 계산
        
        Parameters:
        -----------
        altitude : float
            고도 (m)
        
        Returns:
        --------
        a : float
            음속 (m/s)
        """
        if altitude <= 11000:
            T = 288.15 - 0.0065 * altitude
        elif altitude <= 20000:
            T = 216.65
        else:
            T = 216.65 + 0.001 * (altitude - 20000)
        
        gamma = 1.4  # 비열비
        R = 287.0531
        a = np.sqrt(gamma * R * T)
        
        return a


# 테스트 함수
def test_aerodynamics():
    """공력 모델 테스트"""
    print("=" * 60)
    print("진짜 6DOF 공력 모델 테스트")
    print("=" * 60)
    
    # 미사일 설정
    missile_config = {
        'diameter': 0.88,
        'length': 10.94,
        'reference_area': np.pi * (0.88/2)**2,
        'wingspan': 0.88 * 2,
        'cd_base': 0.25,
        'cl_alpha': 3.5,
        'cy_beta': -0.5,
        'cm_alpha': -0.15,
        'cn_beta': 0.1,
        'cl_p': -0.5,
        'cm_q': -0.8,
        'cn_r': -0.8
    }
    
    aero = AerodynamicsTrue6DOF(missile_config)
    
    # 테스트 조건
    altitude = 5000  # m
    u, v, w = 800, 10, 50  # Body Frame 속도 (m/s)
    V_mag = np.sqrt(u**2 + v**2 + w**2)
    p, q, r = 0.1, 0.05, -0.02  # 각속도 (rad/s)
    
    # 대기 조건
    rho = aero.get_atmospheric_density(altitude)
    a = aero.get_sound_speed(altitude)
    mach = V_mag / a
    
    print(f"\n테스트 조건:")
    print(f"  고도: {altitude/1000:.1f} km")
    print(f"  속도: u={u:.1f}, v={v:.1f}, w={w:.1f} m/s")
    print(f"  속도 크기: {V_mag:.1f} m/s")
    print(f"  마하수: {mach:.2f}")
    print(f"  대기 밀도: {rho:.4f} kg/m³")
    
    # 공력 힘 계산
    F_x, F_y, F_z, alpha, beta = aero.calculate_aerodynamic_forces(
        rho, V_mag, u, v, w, mach
    )
    
    print(f"\n공력 힘 (Body Frame):")
    print(f"  F_x (항력): {F_x/1000:.2f} kN")
    print(f"  F_y (측력): {F_y/1000:.2f} kN")
    print(f"  F_z (양력): {F_z/1000:.2f} kN")
    print(f"  받음각: {np.degrees(alpha):.2f}°")
    print(f"  측면 받음각: {np.degrees(beta):.2f}°")
    
    # 공력 모멘트 계산
    L_aero, M_aero, N_aero = aero.calculate_aerodynamic_moments(
        rho, V_mag, alpha, beta, p, q, r
    )
    
    print(f"\n공력 모멘트 (Body Frame):")
    print(f"  L (롤): {L_aero/1000:.2f} kN·m")
    print(f"  M (피치): {M_aero/1000:.2f} kN·m")
    print(f"  N (요): {N_aero/1000:.2f} kN·m")
    
    print("\n" + "=" * 60)
    print("✅ 공력 모델 테스트 완료!")
    print("=" * 60)


if __name__ == "__main__":
    test_aerodynamics()
