#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
진짜 6DOF 동역학 모델
Body Frame 기반, 병진-회전 커플링
"""

import numpy as np
from aerodynamics_true_6dof import AerodynamicsTrue6DOF
from coordinate_transforms import CoordinateTransform


class DynamicsTrue6DOF:
    """진짜 6DOF 동역학 클래스"""
    
    def __init__(self, missile_config):
        """
        동역학 모델 초기화
        
        Parameters:
        -----------
        missile_config : dict
            미사일 설정
        """
        # 질량 특성
        self.mass_initial = missile_config['launch_weight']
        self.propellant_mass = missile_config['propellant_mass']
        self.mass_empty = self.mass_initial - self.propellant_mass
        
        # 관성 모멘트
        self.Ixx = missile_config['inertia_xx']
        self.Iyy = missile_config['inertia_yy']
        self.Izz = missile_config['inertia_zz']
        
        # 추진 특성
        self.isp_sea = missile_config['isp_sea']
        self.isp_vac = missile_config.get('isp_vac', self.isp_sea * 1.1)
        self.burn_time = missile_config['burn_time']
        self.thrust_profile = missile_config.get('thrust_profile', None)
        
        # 기하학적 특성
        self.diameter = missile_config['diameter']
        self.length = missile_config['length']
        
        # 공력 모델
        self.aero = AerodynamicsTrue6DOF(missile_config)
        
        # 중력 상수
        self.g0 = 9.80665
        self.R_earth = 6371000
    
    def get_gravity(self, altitude):
        """
        고도에 따른 중력 가속도
        
        Parameters:
        -----------
        altitude : float
            고도 (m)
        
        Returns:
        --------
        g : float
            중력 가속도 (m/s²)
        """
        g = self.g0 * (self.R_earth / (self.R_earth + altitude))**2
        return g
    
    def get_thrust(self, t, altitude):
        """
        추력 계산 (고도 보상)
        
        Parameters:
        -----------
        t : float
            시간 (s)
        altitude : float
            고도 (m)
        
        Returns:
        --------
        T : float
            추력 (N)
        """
        if t >= self.burn_time:
            return 0.0
        
        # 고도에 따른 비추력 변화
        if altitude < 11000:
            isp = self.isp_sea + (self.isp_vac - self.isp_sea) * (altitude / 11000)
        else:
            isp = self.isp_vac
        
        # 연료 소모율
        mdot = self.propellant_mass / self.burn_time
        
        # 추력 프로파일 적용
        if self.thrust_profile is not None:
            T = self.thrust_profile(t)
        else:
            # 일정한 추력
            T = isp * mdot * self.g0
        
        return T
    
    def get_mass(self, t):
        """
        시간에 따른 질량
        
        Parameters:
        -----------
        t : float
            시간 (s)
        
        Returns:
        --------
        m : float
            질량 (kg)
        """
        if t >= self.burn_time:
            return self.mass_empty
        
        # 선형 연료 소모
        fuel_consumed = (self.propellant_mass / self.burn_time) * t
        m = self.mass_initial - fuel_consumed
        
        return max(m, self.mass_empty)
    
    def dynamics_equations(self, t, state):
        """
        진짜 6DOF 동역학 방정식
        
        상태 벡터: [u, v, w, p, q, r, φ, θ, ψ, X, Y, Z]
        - u, v, w: Body Frame 속도 (m/s)
        - p, q, r: Body Frame 각속도 (rad/s)
        - φ, θ, ψ: 오일러각 (rad)
        - X, Y, Z: Inertial Frame 위치 (m)
        
        Parameters:
        -----------
        t : float
            시간 (s)
        state : ndarray (12,)
            상태 벡터
        
        Returns:
        --------
        state_dot : ndarray (12,)
            상태 미분값
        """
        # 상태 언패킹
        u, v, w = state[0:3]  # Body Frame 속도
        p, q, r = state[3:6]  # Body Frame 각속도
        phi, theta, psi = state[6:9]  # 오일러각
        X, Y, Z = state[9:12]  # Inertial Frame 위치
        
        # 고도 (Z가 위로 향한다고 가정)
        altitude = Z
        
        # 디버그 출력 제거 (인코딩 문제)
        
        # 질량 및 중력
        m = self.get_mass(t)
        g = self.get_gravity(altitude)
        
        # 속도 크기
        V_mag = np.sqrt(u**2 + v**2 + w**2)
        V_mag = max(V_mag, 1e-6)  # 0으로 나누기 방지
        
        # 대기 조건
        rho = self.aero.get_atmospheric_density(altitude)
        a = self.aero.get_sound_speed(altitude)
        mach = V_mag / a
        
        # === 힘 계산 (Body Frame) ===
        
        # 1. 추력 (Body X축 방향)
        T = self.get_thrust(t, altitude)
        F_thrust_x = T
        F_thrust_y = 0.0
        F_thrust_z = 0.0
        
        # 2. 공력 힘 (Body Frame)
        F_aero_x, F_aero_y, F_aero_z, alpha, beta = self.aero.calculate_aerodynamic_forces(
            rho, V_mag, u, v, w, mach
        )
        
        # 3. 중력 (Inertial Frame → Body Frame 변환)
        g_inertial = np.array([0, 0, -g])  # 중력은 -Z 방향
        g_body = CoordinateTransform.inertial_to_body(g_inertial, phi, theta, psi)
        F_gravity_x = m * g_body[0]
        F_gravity_y = m * g_body[1]
        F_gravity_z = m * g_body[2]
        
        # 전체 힘 (Body Frame)
        F_x = F_thrust_x + F_aero_x + F_gravity_x
        F_y = F_thrust_y + F_aero_y + F_gravity_y
        F_z = F_thrust_z + F_aero_z + F_gravity_z
        
        # === 모멘트 계산 (Body Frame) ===
        
        L_aero, M_aero, N_aero = self.aero.calculate_aerodynamic_moments(
            rho, V_mag, alpha, beta, p, q, r
        )
        
        # === 병진 운동 방정식 (Body Frame) ===
        # 핵심: 각속도가 직접 병진 속도에 영향!
        
        u_dot = r*v - q*w + F_x / m
        v_dot = p*w - r*u + F_y / m
        w_dot = q*u - p*v + F_z / m
        
        # === 회전 운동 방정식 (Body Frame) ===
        # 오일러 각운동 방정식 + 자이로 효과
        
        p_dot = (L_aero + (self.Iyy - self.Izz) * q * r) / self.Ixx
        q_dot = (M_aero + (self.Izz - self.Ixx) * p * r) / self.Iyy
        r_dot = (N_aero + (self.Ixx - self.Iyy) * p * q) / self.Izz
        
        # === 오일러각 변화율 ===
        
        phi_dot, theta_dot, psi_dot = CoordinateTransform.euler_rates(
            phi, theta, psi, p, q, r
        )
        
        # === 위치 변화율 (Inertial Frame) ===
        # Body Frame 속도 → Inertial Frame 변환
        
        v_body = np.array([u, v, w])
        v_inertial = CoordinateTransform.body_to_inertial(v_body, phi, theta, psi)
        
        X_dot = v_inertial[0]
        Y_dot = v_inertial[1]
        Z_dot = v_inertial[2]
        
        # === 상태 미분값 반환 ===
        
        state_dot = np.array([
            u_dot, v_dot, w_dot,  # Body Frame 속도 변화율
            p_dot, q_dot, r_dot,  # Body Frame 각속도 변화율
            phi_dot, theta_dot, psi_dot,  # 오일러각 변화율
            X_dot, Y_dot, Z_dot  # Inertial Frame 위치 변화율
        ])
        
        return state_dot
    
    def create_initial_state(self, launch_angle_deg, azimuth_deg=90):
        """
        초기 상태 벡터 생성
        
        Parameters:
        -----------
        launch_angle_deg : float
            발사각 (도)
        azimuth_deg : float
            방위각 (도, 0=북쪽, 90=동쪽)
        
        Returns:
        --------
        state0 : ndarray (12,)
            초기 상태 벡터
        """
        # 초기 속도 (발사 직후 상태 가정)
        # 실제 미사일은 발사 후 수 초간 가속하여 안정화
        V0_initial = 50.0  # 초기 속도 50 m/s
        u0 = V0_initial  # Body X축 방향
        v0 = 0.0
        w0 = 0.0
        
        # 초기 각속도 (0)
        p0 = 0.0
        q0 = 0.0
        r0 = 0.0
        
        # 초기 자세 (발사각과 방위각에 맞춤)
        phi0 = 0.0  # 롤 0
        theta0 = np.radians(launch_angle_deg)  # 피치 = 발사각
        psi0 = np.radians(azimuth_deg)  # 요 = 방위각
        
        # 초기 위치
        X0 = 0.0
        Y0 = 0.0
        Z0 = 10.0  # 발사대 높이 (10m)
        
        state0 = np.array([u0, v0, w0, p0, q0, r0, phi0, theta0, psi0, X0, Y0, Z0])
        
        return state0
    
    def get_outputs(self, t, state):
        """
        상태로부터 유용한 출력 계산
        
        Parameters:
        -----------
        t : float
            시간 (s)
        state : ndarray (12,)
            상태 벡터
        
        Returns:
        --------
        outputs : dict
            계산된 출력 값들
        """
        u, v, w = state[0:3]
        p, q, r = state[3:6]
        phi, theta, psi = state[6:9]
        X, Y, Z = state[9:12]
        
        # 속도
        V_mag = np.sqrt(u**2 + v**2 + w**2)
        
        # Inertial Frame 속도
        v_body = np.array([u, v, w])
        v_inertial = CoordinateTransform.body_to_inertial(v_body, phi, theta, psi)
        
        # 받음각
        if abs(u) > 1e-6:
            alpha = np.arctan2(w, u)
            beta = np.arcsin(np.clip(v / V_mag, -1.0, 1.0)) if V_mag > 1e-6 else 0.0
        else:
            alpha = 0.0
            beta = 0.0
        
        # 대기 조건
        altitude = Z
        rho = self.aero.get_atmospheric_density(altitude)
        a = self.aero.get_sound_speed(altitude)
        mach = V_mag / a if a > 0 else 0
        
        # 동압
        q_dyn = 0.5 * rho * V_mag**2
        
        # 질량
        m = self.get_mass(t)
        
        outputs = {
            'time': t,
            'u': u, 'v': v, 'w': w,
            'p': p, 'q': q, 'r': r,
            'phi': phi, 'theta': theta, 'psi': psi,
            'X': X, 'Y': Y, 'Z': Z,
            'V_mag': V_mag,
            'V_north': v_inertial[0],
            'V_east': v_inertial[1],
            'V_up': v_inertial[2],
            'alpha': alpha,
            'beta': beta,
            'mach': mach,
            'q_dyn': q_dyn,
            'rho': rho,
            'mass': m,
            'altitude': altitude
        }
        
        return outputs


# 테스트 함수
def test_dynamics():
    """동역학 모델 테스트"""
    print("=" * 60)
    print("진짜 6DOF 동역학 모델 테스트")
    print("=" * 60)
    
    # 미사일 설정 (SCUD-B 예시)
    missile_config = {
        'launch_weight': 5860,
        'propellant_mass': 4875,
        'diameter': 0.88,
        'length': 10.94,
        'reference_area': np.pi * (0.88/2)**2,
        'wingspan': 0.88 * 2,
        'inertia_xx': 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),
        'inertia_yy': 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),
        'inertia_zz': 5860 * ((0.88/2)**2 / 2),
        'isp_sea': 230,
        'isp_vac': 258,
        'burn_time': 65,
        'cd_base': 0.25,
        'cl_alpha': 3.5,
        'cy_beta': -0.5,
        'cm_alpha': -0.15,
        'cn_beta': 0.1,
        'cl_p': -0.5,
        'cm_q': -0.8,
        'cn_r': -0.8
    }
    
    dyn = DynamicsTrue6DOF(missile_config)
    
    # 초기 상태 (45도 발사)
    state0 = dyn.create_initial_state(launch_angle_deg=45, azimuth_deg=90)
    
    print(f"\n초기 상태:")
    print(f"  Body 속도: u={state0[0]:.1f}, v={state0[1]:.1f}, w={state0[2]:.1f} m/s")
    print(f"  각속도: p={state0[3]:.3f}, q={state0[4]:.3f}, r={state0[5]:.3f} rad/s")
    print(f"  오일러각: φ={np.degrees(state0[6]):.1f}°, θ={np.degrees(state0[7]):.1f}°, ψ={np.degrees(state0[8]):.1f}°")
    print(f"  위치: X={state0[9]:.1f}, Y={state0[10]:.1f}, Z={state0[11]:.1f} m")
    
    # 동역학 방정식 평가 (t=1초)
    t = 1.0
    state_dot = dyn.dynamics_equations(t, state0)
    
    print(f"\n시간 t={t}s에서의 미분값:")
    print(f"  du/dt={state_dot[0]:.2f}, dv/dt={state_dot[1]:.2f}, dw/dt={state_dot[2]:.2f} m/s²")
    print(f"  dp/dt={state_dot[3]:.4f}, dq/dt={state_dot[4]:.4f}, dr/dt={state_dot[5]:.4f} rad/s²")
    print(f"  dφ/dt={state_dot[6]:.4f}, dθ/dt={state_dot[7]:.4f}, dψ/dt={state_dot[8]:.4f} rad/s")
    print(f"  dX/dt={state_dot[9]:.2f}, dY/dt={state_dot[10]:.2f}, dZ/dt={state_dot[11]:.2f} m/s")
    
    # 출력 계산
    outputs = dyn.get_outputs(t, state0)
    
    print(f"\n계산된 출력:")
    print(f"  속도 크기: {outputs['V_mag']:.2f} m/s")
    print(f"  마하수: {outputs['mach']:.3f}")
    print(f"  받음각: {np.degrees(outputs['alpha']):.2f}°")
    print(f"  측면 받음각: {np.degrees(outputs['beta']):.2f}°")
    print(f"  동압: {outputs['q_dyn']:.2f} Pa")
    print(f"  질량: {outputs['mass']:.1f} kg")
    
    print("\n" + "=" * 60)
    print("✅ 동역학 모델 테스트 완료!")
    print("=" * 60)


if __name__ == "__main__":
    test_dynamics()
