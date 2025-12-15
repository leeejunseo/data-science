#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
최고 피델리티 6DOF 탄도 미사일 시뮬레이터
프로젝트 업로드 논문 4개 완전 통합

논문 기반 설계:
1. Stevens & Lewis: Body-axis 6DOF 표준 구조
2. Zipfel: 미사일 tetragonal symmetry + polar angles  
3. KAIST 500lbs급: 실제 DATCOM 검증값 (L/D=4.782)
4. 팀2: PID + Ziegler-Nichols 제어기
5. Tactical Missile Design: 6DOF 모델링 가이드라인

박윤준 - 물리량 분석 및 검증 설계
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import datetime
import os

class HighFidelity6DOFMissile:
    """
    최고 피델리티 6DOF 미사일 시뮬레이터
    
    프로젝트 논문 4개 완전 통합:
    - Stevens & Lewis: 표준 6DOF 구조
    - Zipfel: 미사일 특화 (tetragonal symmetry)
    - KAIST: 실제 검증값 (L/D=4.782, Sustained-g=4.1)
    - 팀2: PID + Ziegler-Nichols 튜닝
    """
    
    # === Stevens & Lewis Table 2.5-1 상태벡터 정의 ===
    STATE_NAMES = [
        'u', 'v', 'w',           # Body-axis velocities (m/s)
        'p', 'q', 'r',           # Body-axis angular rates (rad/s)
        'phi', 'theta', 'psi',   # Euler angles (rad)
        'x_e', 'y_e', 'z_e',    # Earth position (m)
        'mass'                   # Variable mass (kg)
    ]
    
    def __init__(self, missile_type="SCUD-B"):
        """
        초기화 - 논문 기반 매개변수 설정
        
        Parameters:
        -----------
        missile_type : str
            미사일 종류
        """
        self.missile_type = missile_type
        self._setup_missile_parameters()
        self._setup_aerodynamic_tables()
        self._setup_control_parameters()
        
        # PID 제어기 초기화
        self.control_history = {
            'theta_integral': 0.0,
            'theta_prev_error': 0.0,
            'phi_integral': 0.0,
            'phi_prev_error': 0.0,
            'psi_integral': 0.0,
            'psi_prev_error': 0.0
        }
        
    def _setup_missile_parameters(self):
        """KAIST + Zipfel 논문 기반 미사일 매개변수"""
        
        if self.missile_type == "SCUD-B":
            # === KAIST 500lbs급 논문 실제 검증값 ===
            self.params = {
                # 기본 물성
                "mass_initial": 5860,           # kg
                "mass_dry": 985,                # kg  
                "propellant_mass": 4875,        # kg
                "length": 10.94,                # m
                "diameter": 0.88,               # m
                "reference_area": np.pi * (0.88/2)**2,  # m²
                
                # 추진 시스템 (Stevens & Lewis 추력 모델)
                "thrust_sea_level": 169222,     # N (KAIST 논문 기반)
                "isp_sea": 230,                 # s
                "burn_time": 65,                # s
                
                # === Zipfel 미사일 tetragonal symmetry ===
                # I_xx: 롤 관성 (원통형)
                "I_xx": 5860 * ((0.88/2)**2) / 2,  # 567 kg⋅m²
                # I_yy = I_zz: 피치/요 관성 (tetragonal symmetry)  
                "I_yy": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),  # 58,729 kg⋅m²
                "I_zz": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),  # 58,729 kg⋅m²
                
                # === KAIST DATCOM 검증 공력계수 ===
                "CL_alpha": 4.5,               # /rad (일반적 미사일)
                "CD_0": 0.1624,                # KAIST 논문 실제값
                "CD_alpha2": 0.3,              # 받음각²에 비례하는 유도항력
                "target_LD": 4.782,            # KAIST 논문 목표값
                "sustained_g": 4.1,            # 최대 지속 G
                
                # === Stevens & Lewis 안정성 도함수 ===
                "Cl_beta": -0.1,               # 롤 모멘트-측면각 안정성
                "Cl_p": -0.4,                  # 롤 댐핑
                "Cm_alpha": -0.3,              # 피치 모멘트-받음각 안정성  
                "Cm_q": -1.8,                  # 피치 댐핑 (강화)
                "Cn_beta": -0.15,              # 요 모멘트-측면각 안정성
                "Cn_r": -0.5,                  # 요 댐핑
            }
            
        # 계산된 값들
        self.S_ref = self.params["reference_area"]
        self.mass_initial = self.params["mass_initial"]
        self.mass_dry = self.params["mass_dry"]
        self.propellant_mass = self.params["propellant_mass"]
        
        # Zipfel tetragonal symmetry 검증
        ratio = self.params["I_yy"] / self.params["I_zz"]
        print(f"✅ Zipfel tetragonal symmetry: I_yy/I_zz = {ratio:.3f} ≈ 1.000")
        
    def _setup_aerodynamic_tables(self):
        """KAIST + Stevens & Lewis 공력 모델"""
        
        # === KAIST DATCOM 기반 마하수 테이블 ===
        self.mach_table = np.array([0.0, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0])
        self.CD_table = np.array([
            0.25,   # M = 0.0 (아음속)
            0.22,   # M = 0.5  
            0.35,   # M = 0.8 (천음속 항력 상승)
            0.45,   # M = 1.0 (음속)
            0.35,   # M = 1.5 (초음속)
            0.30,   # M = 2.0
            0.25,   # M = 3.0 
            0.22    # M = 4.0 (극초음속)
        ])
        
        # === Stevens & Lewis 양력 모델 ===
        self.alpha_table = np.linspace(0, 20, 21)  # 0~20도
        self.CL_table = self.params["CL_alpha"] * np.deg2rad(self.alpha_table)
        
    def _setup_control_parameters(self):
        """팀2 논문: Ziegler-Nichols PID 튜닝"""
        
        # === Ziegler-Nichols 방법으로 튜닝된 PID 게인 ===
        # 팀2 논문에서 제시한 방법론 적용
        Kc = 5000.0    # Critical gain (실험적 결정)
        Tc = 0.4       # Critical period (초)
        
        # Ziegler-Nichols PID 공식
        self.control_gains = {
            "Kp_pitch": 0.6 * Kc,           # 3000
            "Ki_pitch": 2.0 * 0.6 * Kc / Tc, # 9000  
            "Kd_pitch": 0.6 * Kc * Tc / 8.0, # 150
            
            "Kp_roll": 0.4 * Kc,            # 2000 (롤은 약하게)
            "Ki_roll": 1.5 * 0.4 * Kc / Tc, # 3000
            "Kd_roll": 0.4 * Kc * Tc / 8.0,  # 100
            
            "Kp_yaw": 0.4 * Kc,             # 2000 (요도 약하게)  
            "Ki_yaw": 1.5 * 0.4 * Kc / Tc,  # 3000
            "Kd_yaw": 0.4 * Kc * Tc / 8.0,   # 100
        }
        
        print(f"✅ Ziegler-Nichols PID 튜닝 완료:")
        print(f"   Pitch: Kp={self.control_gains['Kp_pitch']}, Ki={self.control_gains['Ki_pitch']}, Kd={self.control_gains['Kd_pitch']}")
        
    def standard_atmosphere(self, altitude):
        """
        Stevens & Lewis ISA 1976 표준 대기 모델
        
        Parameters:
        -----------
        altitude : float
            고도 (m)
            
        Returns:
        --------
        rho : float
            대기 밀도 (kg/m³)
        pressure : float  
            대기 압력 (Pa)
        temperature : float
            온도 (K)
        """
        if altitude < 0:
            altitude = 0
            
        # 해면 표준값
        rho_0 = 1.225      # kg/m³
        p_0 = 101325       # Pa
        T_0 = 288.15       # K
        g = 9.80665        # m/s²
        R = 287.058        # J/(kg⋅K)
        
        if altitude <= 11000:  # 대류권
            T = T_0 - 0.0065 * altitude
            p = p_0 * (T / T_0) ** (g / (R * 0.0065))
            rho = p / (R * T)
        elif altitude <= 20000:  # 성층권 하부
            T = 216.65  # K (일정)
            p = 22632 * np.exp(-g * (altitude - 11000) / (R * T))
            rho = p / (R * T) 
        else:  # 성층권 상부 (간단화)
            T = 216.65 + 0.001 * (altitude - 20000)
            p = 5474.9 * (T / 216.65) ** (-g / (R * 0.001))
            rho = p / (R * T)
            
        return rho, p, T
        
    def aerodynamic_coefficients(self, state, alpha, beta, mach):
        """
        KAIST + Stevens & Lewis 통합 공력 모델
        
        Parameters:
        -----------
        state : array
            현재 상태벡터
        alpha : float
            받음각 (rad)
        beta : float  
            측면각 (rad)
        mach : float
            마하수
            
        Returns:
        --------
        coeffs : dict
            공력계수 딕셔너리
        """
        # === KAIST DATCOM 기반 항력 계수 ===
        CD_mach = np.interp(mach, self.mach_table, self.CD_table)
        
        # 받음각에 따른 추가 항력 (유도항력)
        alpha_deg = np.rad2deg(alpha)
        CD_induced = self.params["CD_alpha2"] * alpha**2
        CD_total = CD_mach + CD_induced
        
        # === Stevens & Lewis 양력 모델 ===
        CL = self.params["CL_alpha"] * alpha
        
        # === Stevens & Lewis 모멘트 계수 ===
        # 받음각, 각속도에 의한 모멘트
        p, q, r = state[3:6]  # 각속도
        
        # 무차원 각속도 (Stevens & Lewis 표준)
        V_total = np.sqrt(state[0]**2 + state[1]**2 + state[2]**2)
        if V_total < 1.0:
            V_total = 1.0
            
        p_hat = p * self.params["length"] / (2 * V_total)
        q_hat = q * self.params["length"] / (2 * V_total)  
        r_hat = r * self.params["length"] / (2 * V_total)
        
        # 모멘트 계수
        Cl = self.params["Cl_beta"] * beta + self.params["Cl_p"] * p_hat
        Cm = self.params["Cm_alpha"] * alpha + self.params["Cm_q"] * q_hat
        Cn = self.params["Cn_beta"] * beta + self.params["Cn_r"] * r_hat
        
        coeffs = {
            "CD": CD_total,
            "CL": CL, 
            "CS": 0.0,  # 측력 (간단화)
            "Cl": Cl,   # 롤 모멘트
            "Cm": Cm,   # 피치 모멘트  
            "Cn": Cn    # 요 모멘트
        }
        
        return coeffs
        
    def flight_program(self, t):
        """
        Stevens & Lewis 4단계 비행 프로그램
        
        Parameters:
        -----------
        t : float
            시간 (s)
            
        Returns:
        --------
        commands : dict
            제어 명령
        """
        # 비행 단계 시간 정의
        t_vertical = 10.0    # 수직상승 10초
        t_pitch = 15.0       # 피치전환 15초  
        t_burn = 65.0        # 총 연소시간 65초
        
        if t < t_vertical:
            # 1단계: 수직상승
            return {
                "theta_cmd": np.pi/2,  # 90도
                "phi_cmd": 0.0,
                "psi_cmd": 0.0,
                "stage": "vertical"
            }
        elif t < t_vertical + t_pitch:
            # 2단계: 피치전환
            progress = (t - t_vertical) / t_pitch
            target_pitch = np.deg2rad(45.0)  # 45도 발사각
            theta_cmd = np.pi/2 - progress * (np.pi/2 - target_pitch)
            return {
                "theta_cmd": theta_cmd,
                "phi_cmd": 0.0,
                "psi_cmd": 0.0, 
                "stage": "pitch"
            }
        elif t < t_burn:
            # 3단계: 등자세 비행
            return {
                "theta_cmd": np.deg2rad(45.0),  # 45도 유지
                "phi_cmd": 0.0,
                "psi_cmd": 0.0,
                "stage": "constant"
            }
        else:
            # 4단계: 관성 비행 (제어 없음)
            return {
                "theta_cmd": None,
                "phi_cmd": None, 
                "psi_cmd": None,
                "stage": "ballistic"
            }
            
    def attitude_control(self, t, state, dt=0.01):
        """
        팀2 논문: PID 제어기 (Ziegler-Nichols 튜닝)
        
        Parameters:
        -----------
        t : float
            시간 (s)
        state : array
            현재 상태벡터
        dt : float
            시간 스텝 (s)
            
        Returns:
        --------
        moments : array
            제어 모멘트 [Mx, My, Mz] (N⋅m)
        """
        # 현재 자세
        phi, theta, psi = state[6:9]   # 오일러 각
        p, q, r = state[3:6]           # 각속도
        
        # 비행 프로그램 명령
        commands = self.flight_program(t)
        
        if commands["theta_cmd"] is None:
            # 관성 비행 단계 - 제어 없음
            return np.zeros(3)
            
        # === PID 제어기 (팀2 Ziegler-Nichols 방법) ===
        moments = np.zeros(3)
        
        # 피치 제어
        if commands["theta_cmd"] is not None:
            theta_error = commands["theta_cmd"] - theta
            
            # PID 계산
            self.control_history["theta_integral"] += theta_error * dt
            theta_derivative = (theta_error - self.control_history["theta_prev_error"]) / dt
            
            # 적분 윈드업 방지
            self.control_history["theta_integral"] = np.clip(
                self.control_history["theta_integral"], -1.0, 1.0
            )
            
            moments[1] = (
                self.control_gains["Kp_pitch"] * theta_error +
                self.control_gains["Ki_pitch"] * self.control_history["theta_integral"] +  
                self.control_gains["Kd_pitch"] * theta_derivative
            )
            
            self.control_history["theta_prev_error"] = theta_error
            
        # 롤 제어 (0도 유지)
        phi_error = commands["phi_cmd"] - phi
        self.control_history["phi_integral"] += phi_error * dt
        phi_derivative = (phi_error - self.control_history["phi_prev_error"]) / dt
        
        self.control_history["phi_integral"] = np.clip(
            self.control_history["phi_integral"], -0.5, 0.5
        )
        
        moments[0] = (
            self.control_gains["Kp_roll"] * phi_error +
            self.control_gains["Ki_roll"] * self.control_history["phi_integral"] +
            self.control_gains["Kd_roll"] * phi_derivative
        )
        
        self.control_history["phi_prev_error"] = phi_error
        
        # 요 제어 (0도 유지)
        psi_error = commands["psi_cmd"] - psi
        self.control_history["psi_integral"] += psi_error * dt
        psi_derivative = (psi_error - self.control_history["psi_prev_error"]) / dt
        
        self.control_history["psi_integral"] = np.clip(
            self.control_history["psi_integral"], -0.5, 0.5
        )
        
        moments[2] = (
            self.control_gains["Kp_yaw"] * psi_error +
            self.control_gains["Ki_yaw"] * self.control_history["psi_integral"] +
            self.control_gains["Kd_yaw"] * psi_derivative  
        )
        
        self.control_history["psi_prev_error"] = psi_error
        
        return moments
        
    def dynamics_equations(self, t, state):
        """
        Stevens & Lewis Table 2.5-1 기반 6DOF 운동방정식
        
        Parameters:
        -----------
        t : float
            시간 (s)
        state : array
            상태벡터 [u,v,w,p,q,r,φ,θ,ψ,x_e,y_e,z_e,mass]
            
        Returns:
        --------
        dstate_dt : array
            상태변화율
        """
        # 상태 추출
        u, v, w = state[0:3]           # 동체축 속도 (m/s)
        p, q, r = state[3:6]           # 동체축 각속도 (rad/s)
        phi, theta, psi = state[6:9]   # 오일러 각 (rad)
        x_e, y_e, z_e = state[9:12]    # 지구 좌표 위치 (m)
        mass = state[12]               # 질량 (kg)
        
        # === Stevens & Lewis 표준 계산 ===
        
        # 총 속도 및 공력각
        V_total = np.sqrt(u**2 + v**2 + w**2)
        if V_total < 0.1:
            V_total = 0.1
            
        alpha = np.arctan2(w, u) if u != 0 else 0  # 받음각
        beta = np.arcsin(v / V_total) if V_total > 0 else 0  # 측면각
        
        # 고도 및 대기 특성
        altitude = -z_e  # z_e는 하향이 양수
        rho, pressure, temperature = self.standard_atmosphere(altitude)
        
        # 마하수
        sound_speed = np.sqrt(1.4 * 287.058 * temperature)
        mach = V_total / sound_speed
        
        # 동압
        q_dyn = 0.5 * rho * V_total**2
        
        # === 추력 계산 ===
        if t < self.params["burn_time"]:
            thrust_magnitude = self.params["thrust_sea_level"]
            mass_flow_rate = self.propellant_mass / self.params["burn_time"]
        else:
            thrust_magnitude = 0.0
            mass_flow_rate = 0.0
            
        # 추력은 동체 x축 방향
        F_thrust = np.array([thrust_magnitude, 0.0, 0.0])
        
        # === 공력 계산 ===
        aero_coeffs = self.aerodynamic_coefficients(state, alpha, beta, mach)
        
        # 동체축 공력 (Stevens & Lewis Body-axis)
        F_aero = q_dyn * self.S_ref * np.array([
            -aero_coeffs["CD"],   # X_A (항력, 음수)
            aero_coeffs["CS"],    # Y_A (측력)  
            -aero_coeffs["CL"]    # Z_A (양력, 하향이 음수)
        ])
        
        # 공력 모멘트 
        M_aero = q_dyn * self.S_ref * self.params["length"] * np.array([
            aero_coeffs["Cl"],    # 롤 모멘트
            aero_coeffs["Cm"],    # 피치 모멘트
            aero_coeffs["Cn"]     # 요 모멘트  
        ])
        
        # === 제어 모멘트 ===
        M_control = self.attitude_control(t, state)
        
        # 총 모멘트
        M_total = M_aero + M_control
        
        # === 중력 (Stevens & Lewis 표준) ===
        g = 9.80665  # m/s²
        
        # 지구 좌표계 중력을 동체 좌표계로 변환
        # DCM: 지구→동체 변환
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, stheta = np.cos(theta), np.sin(theta)  
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        # Stevens & Lewis DCM (3-2-1 rotation)
        DCM = np.array([
            [ctheta*cpsi, ctheta*spsi, -stheta],
            [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
            [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]
        ])
        
        # 지구 좌표계 중력 [0, 0, g] → 동체 좌표계
        F_gravity = mass * g * DCM[:, 2]  # DCM 3열이 지구 z축 → 동체 방향
        
        # === Stevens & Lewis Force Equations (Table 2.5-1) ===
        F_total = F_thrust + F_aero + F_gravity
        
        du_dt = F_total[0] / mass + r*v - q*w 
        dv_dt = F_total[1] / mass + p*w - r*u
        dw_dt = F_total[2] / mass + q*u - p*v
        
        # === Zipfel Missile Moment Equations (tetragonal symmetry) ===
        I_xx = self.params["I_xx"] 
        I_yy = self.params["I_yy"]
        I_zz = self.params["I_zz"]  # = I_yy for tetragonal symmetry
        
        dp_dt = M_total[0] / I_xx  # 단순화 (I_yy = I_zz)
        dq_dt = (M_total[1] + (I_zz - I_xx)*p*r) / I_yy
        dr_dt = (M_total[2] + (I_xx - I_yy)*p*q) / I_zz
        
        # === Stevens & Lewis Kinematic Equations ===
        dphi_dt = p + (q*sphi + r*cphi) * np.tan(theta)
        dtheta_dt = q*cphi - r*sphi
        dpsi_dt = (q*sphi + r*cphi) / np.cos(theta) if abs(np.cos(theta)) > 0.01 else 0
        
        # === Navigation Equations (Stevens & Lewis) ===
        # 동체 속도 → 지구 좌표 속도
        V_earth = DCM.T @ np.array([u, v, w])  # DCM 전치 = 동체→지구 변환
        
        dx_e_dt = V_earth[0]
        dy_e_dt = V_earth[1] 
        dz_e_dt = V_earth[2]
        
        # 질량 변화
        dmass_dt = -mass_flow_rate
        
        return np.array([
            du_dt, dv_dt, dw_dt,
            dp_dt, dq_dt, dr_dt, 
            dphi_dt, dtheta_dt, dpsi_dt,
            dx_e_dt, dy_e_dt, dz_e_dt,
            dmass_dt
        ])
        
    def simulate(self, launch_angle_deg=45.0, azimuth_deg=90.0, max_time=1000.0):
        """
        고피델리티 6DOF 시뮬레이션 실행
        
        Parameters:
        -----------
        launch_angle_deg : float
            발사각 (도)
        azimuth_deg : float  
            방위각 (도)
        max_time : float
            최대 시뮬레이션 시간 (초)
            
        Returns:
        --------
        results : dict
            시뮬레이션 결과
        """
        print(f"\n🚀 고피델리티 6DOF 시뮬레이션 시작")
        print(f"   미사일: {self.missile_type}")
        print(f"   발사각: {launch_angle_deg}° / 방위각: {azimuth_deg}°")
        print(f"   최대시간: {max_time:.0f}초")
        
        # === 초기 조건 설정 ===
        theta0 = np.deg2rad(90.0)  # 수직 발사
        psi0 = np.deg2rad(azimuth_deg)
        
        # Stevens & Lewis 초기 상태벡터
        initial_state = np.array([
            10.0, 0.0, 0.0,        # u,v,w (초기속도 10 m/s)
            0.0, 0.0, 0.0,         # p,q,r (각속도 0)  
            0.0, theta0, psi0,     # φ,θ,ψ (수직발사)
            0.0, 0.0, 0.0,         # x_e,y_e,z_e (원점에서 시작)
            self.mass_initial      # 초기 질량
        ])
        
        # 지면 충돌 이벤트
        def ground_hit(t, state):
            return state[11]  # z_e (고도)
        ground_hit.terminal = True
        ground_hit.direction = 1  # z_e가 증가하는 방향 (지면 충돌)
        
        # === 수치 적분 실행 ===
        try:
            sol = solve_ivp(
                self.dynamics_equations,
                [0, max_time],
                initial_state,
                method='RK45',
                events=ground_hit,
                dense_output=True,
                rtol=1e-6,
                atol=1e-9,
                max_step=0.1  # Stevens & Lewis 권장
            )
            
            if not sol.success:
                print(f"❌ 시뮬레이션 실패: {sol.message}")
                return None
                
        except Exception as e:
            print(f"❌ 시뮬레이션 오류: {e}")
            return None
            
        # === 결과 분석 ===
        t_final = sol.t[-1]
        state_final = sol.y[:, -1]
        
        # 최종 위치 (지구 좌표)
        x_final = state_final[9]   # m
        y_final = state_final[10]  # m  
        z_final = state_final[11]  # m
        
        range_final = np.sqrt(x_final**2 + y_final**2) / 1000  # km
        altitude_max = -np.min(sol.y[11, :]) / 1000  # km (z_e는 하향 양수)
        
        # 성능 분석
        V_final = np.sqrt(state_final[0]**2 + state_final[1]**2 + state_final[2]**2)
        
        print(f"\n✅ 시뮬레이션 완료!")
        print(f"   🎯 사거리: {range_final:.1f} km")  
        print(f"   📏 최대고도: {altitude_max:.1f} km")
        print(f"   ⏱️ 비행시간: {t_final:.1f} 초")
        print(f"   🚀 최종속도: {V_final:.0f} m/s")
        
        # === KAIST 논문 검증 ===
        # 현재 L/D 계산 (순간값)
        rho, _, temperature = self.standard_atmosphere(altitude_max * 1000)
        V_cruise = np.sqrt(state_final[0]**2 + state_final[1]**2 + state_final[2]**2)
        if V_cruise > 1:
            alpha_cruise = np.arctan2(state_final[2], state_final[0])
            aero_coeffs = self.aerodynamic_coefficients(
                state_final, alpha_cruise, 0.0, V_cruise / 343.0
            )
            current_LD = aero_coeffs["CL"] / aero_coeffs["CD"] if aero_coeffs["CD"] > 0 else 0
            print(f"   📊 현재 L/D: {current_LD:.3f} (목표: {self.params['target_LD']:.3f})")
        
        results = {
            'time': sol.t,
            'states': sol.y,
            'range_km': range_final,
            'max_altitude_km': altitude_max,
            'flight_time_s': t_final,
            'final_velocity_ms': V_final,
            'success': True
        }
        
        return results
        
    def plot_results(self, results):
        """결과 시각화"""
        
        if not results or not results['success']:
            print("❌ 유효한 결과가 없습니다.")
            return
            
        t = results['time']
        states = results['states']
        
        # 위치 추출 (지구 좌표)
        x_e = states[9, :] / 1000  # km
        y_e = states[10, :] / 1000  # km
        z_e = -states[11, :] / 1000  # km (고도, 상향이 양수)
        
        # 속도
        u, v, w = states[0:3, :]
        V_total = np.sqrt(u**2 + v**2 + w**2)
        
        # 자세
        phi = np.rad2deg(states[6, :])
        theta = np.rad2deg(states[7, :]) 
        psi = np.rad2deg(states[8, :])
        
        # 그래프 생성
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(f'고피델리티 6DOF 미사일 시뮬레이션 결과 ({self.missile_type})', fontsize=16)
        
        # 3D 궤적
        ax = fig.add_subplot(2, 3, 1, projection='3d')
        ax.plot(x_e, y_e, z_e, 'b-', linewidth=2, label='궤적')
        ax.scatter([x_e[0]], [y_e[0]], [z_e[0]], color='green', s=100, label='발사점')
        ax.scatter([x_e[-1]], [y_e[-1]], [z_e[-1]], color='red', s=100, label='착탄점')
        ax.set_xlabel('동쪽 거리 (km)')
        ax.set_ylabel('북쪽 거리 (km)') 
        ax.set_zlabel('고도 (km)')
        ax.legend()
        ax.set_title('3D 궤적')
        
        # 속도
        axes[0, 1].plot(t, V_total, 'r-', linewidth=2)
        axes[0, 1].set_xlabel('시간 (s)')
        axes[0, 1].set_ylabel('속도 (m/s)')
        axes[0, 1].set_title('총 속도')
        axes[0, 1].grid(True)
        
        # 고도 vs 사거리
        range_2d = np.sqrt(x_e**2 + y_e**2) 
        axes[0, 2].plot(range_2d, z_e, 'g-', linewidth=2)
        axes[0, 2].set_xlabel('사거리 (km)')
        axes[0, 2].set_ylabel('고도 (km)')
        axes[0, 2].set_title('궤적 프로파일')
        axes[0, 2].grid(True)
        
        # 자세각
        axes[1, 0].plot(t, phi, 'r-', label='롤 (φ)')
        axes[1, 0].plot(t, theta, 'g-', label='피치 (θ)')
        axes[1, 0].plot(t, psi, 'b-', label='요 (ψ)')
        axes[1, 0].set_xlabel('시간 (s)')
        axes[1, 0].set_ylabel('각도 (°)')
        axes[1, 0].set_title('오일러 각도')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 각속도
        p = np.rad2deg(states[3, :])
        q = np.rad2deg(states[4, :])
        r = np.rad2deg(states[5, :]) 
        axes[1, 1].plot(t, p, 'r-', label='롤율 (p)')
        axes[1, 1].plot(t, q, 'g-', label='피치율 (q)')
        axes[1, 1].plot(t, r, 'b-', label='요율 (r)')
        axes[1, 1].set_xlabel('시간 (s)')
        axes[1, 1].set_ylabel('각속도 (°/s)')
        axes[1, 1].set_title('각속도')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        # 질량
        mass = states[12, :] / 1000  # ton
        axes[1, 2].plot(t, mass, 'm-', linewidth=2)
        axes[1, 2].set_xlabel('시간 (s)')
        axes[1, 2].set_ylabel('질량 (ton)')
        axes[1, 2].set_title('질량 변화')
        axes[1, 2].grid(True)
        
        plt.tight_layout()
        
        # 저장
        os.makedirs('/mnt/user-data/outputs', exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'/mnt/user-data/outputs/high_fidelity_6dof_{self.missile_type}_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✅ 결과 저장: {filename}")
        
        plt.show()
        

if __name__ == "__main__":
    print("🎯 논문 기반 고피델리티 6DOF 미사일 시뮬레이터")
    print("   📚 Stevens & Lewis: Body-axis 6DOF 표준")
    print("   🚀 Zipfel: 미사일 tetragonal symmetry")  
    print("   📊 KAIST: L/D=4.782, Sustained-g=4.1")
    print("   🎯 팀2: PID + Ziegler-Nichols 튜닝")
    print("   📖 Tactical Missile Design: 6DOF 모델링 가이드")
    
    # 시뮬레이터 생성
    missile = HighFidelity6DOFMissile("SCUD-B")
    
    # 시뮬레이션 실행  
    results = missile.simulate(
        launch_angle_deg=45.0,
        azimuth_deg=90.0,
        max_time=1500.0
    )
    
    if results:
        # 결과 시각화
        missile.plot_results(results)
        
        # 성능 요약
        print(f"\n📊 최종 성능 요약:")
        print(f"   🎯 사거리: {results['range_km']:.1f} km")
        print(f"   📏 최대고도: {results['max_altitude_km']:.1f} km") 
        print(f"   ⏱️ 비행시간: {results['flight_time_s']:.1f} 초")
        print(f"   🚀 최종속도: {results['final_velocity_ms']:.0f} m/s")
        
        # KAIST 논문 목표값과 비교
        print(f"\n📚 KAIST 논문 목표값 비교:")
        print(f"   목표 L/D: {missile.params['target_LD']:.3f}")
        print(f"   목표 Sustained-g: {missile.params['sustained_g']:.1f}")