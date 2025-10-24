#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6DOF Missile Trajectory Simulation
회전 운동(Roll, Pitch, Yaw) 포함
"""
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import datetime
from scipy.integrate import solve_ivp
import config_6dof as cfg
import math

# matplotlib 설정
plt.rcParams['axes.unicode_minus'] = False

def plot_with_guarantee(fig, save_path, title, show_plot=True):
    """저장 후 새 창에서 그래프를 확실히 표시하는 유틸리티 함수"""
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"그래프가 저장되었습니다: {save_path}")
    
    if show_plot:
        fig_show = plt.figure(figsize=(18, 10))
        plt.imshow(plt.imread(save_path))
        plt.axis('off')
        plt.title(title, fontsize=14)
        plt.tight_layout()
        plt.show(block=True)
        plt.close(fig_show)
    
    plt.close(fig)
    return True

class MissileSimulation6DOF:
    """6DOF 미사일 궤적 시뮬레이션 클래스"""
    
    def __init__(self, missile_type="SCUD-B", apply_errors=False):
        """생성자"""
        self.results = None
        self.states = []
        self.t = []
        self.alpha_list = []
        self.beta_list = []  # 🆕 측면 받음각
        self.CD_list = []
        self.fuel_list = []
        self.mach_list = []
        self.phase_list = []
        
        # 🆕 6DOF: 회전 운동 추적
        self.roll_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        
        self.in_atmosphere = True
        
        self.missile_type = missile_type
        self.update_missile_type(missile_type)
        
        self.apply_errors = apply_errors
        self.error_seed = np.random.randint(1, 10000)
        
        self.error_factors = {
            'thrust': 0.02,
            'cd': 0.05,
            'cl': 0.05,
            'density': 0.03,
            'isp': 0.01,
            'wind': [3.0, 3.0],
            'mass': 0.01,
            'gamma': 0.2,
            'psi': 0.2
        }
    
    def initialize_simulation(self, launch_angle_deg=45, azimuth_deg=90, sim_time=None):
        """시뮬레이션 초기화 (6DOF)"""
        self.alpha_list = []
        self.beta_list = []
        self.CD_list = []
        self.fuel_list = []
        self.mach_list = []
        self.phase_list = []
        self.roll_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        
        self.results = {
            'time': [], 'velocity': [], 'gamma': [], 'psi': [],
            'x': [], 'y': [], 'h': [], 'mass': [],
            'phi': [], 'theta': [], 'psi_euler': [],  # 🆕 오일러각
            'p': [], 'q': [], 'r': [],  # 🆕 각속도
            'alpha': [], 'beta': [], 'CD': [], 'fuel': [], 'mach': [], 'phase': []
        }
        
        self.sim_time = sim_time if sim_time is not None else cfg.SIM_TIME
        
        self.init_speed = 0.0
        self.launch_angle_rad = math.radians(launch_angle_deg)
        self.launch_azimuth_rad = math.radians(azimuth_deg)
        
        missile_info = cfg.MISSILE_TYPES[self.missile_type]
        
        # 🆕 6DOF: 14차원 상태 벡터
        # [V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q, r, M, fuel]
        self.initial_state = cfg.StateVector6DOF.create_initial_state(
            missile_info, launch_angle_deg, azimuth_deg
        )
        
        print(f"6DOF 초기화 완료: {self.missile_type}, 발사각 {launch_angle_deg}°, 방위각 {azimuth_deg}°")
    
    def update_missile_type(self, missile_type):
        """미사일 유형 업데이트 (6DOF)"""
        if missile_type in cfg.MISSILE_TYPES:
            self.missile_type = missile_type
            missile_data = cfg.MISSILE_TYPES[missile_type]
            
            self.diameter = missile_data["diameter"]
            self.length = missile_data["length"]
            self.nozzle_diameter = missile_data["nozzle_diameter"]
            self.propellant_type = missile_data["propellant_type"]
            
            self.missile_mass = missile_data["launch_weight"]
            self.propellant_mass = missile_data["propellant_mass"]
            self.isp_sea = missile_data["isp_sea"]
            self.isp_vacuum = missile_data.get("isp_vacuum", missile_data.get("isp_vac", self.isp_sea * 1.1))
            self.burn_time = missile_data["burn_time"]
            
            self.wing_area = missile_data.get("reference_area", np.pi * (self.diameter/2)**2)
            
            self.vertical_time = missile_data["vertical_time"]
            self.pitch_time = missile_data["pitch_time"]
            self.pitch_angle_deg = missile_data["pitch_angle_deg"]
            
            # 🆕 6DOF: 관성 모멘트
            self.inertia_xx = missile_data["inertia_xx"]
            self.inertia_yy = missile_data["inertia_yy"]
            self.inertia_zz = missile_data["inertia_zz"]
            
            # 🆕 6DOF: 공력 모멘트 계수
            self.cl_alpha = missile_data["cl_alpha"]
            self.cm_alpha = missile_data["cm_alpha"]
            self.cn_beta = missile_data["cn_beta"]
            self.cl_p = missile_data["cl_p"]
            self.cm_q = missile_data["cm_q"]
            self.cn_r = missile_data["cn_r"]
            
            # 무게중심 및 압력중심
            self.cg_location = missile_data["cg_location"]
            self.cp_location = missile_data["cp_location"]
            
            self.CD_TABLE = cfg.get_cd_table_for_missile(missile_type)
            self.thrust_profile = missile_data.get("thrust_profile", None)
            
            print(f"6DOF 미사일 '{missile_type}' 설정 완료")
            return True
        else:
            print(f"경고: 미사일 유형 '{missile_type}' 찾을 수 없음")
            return False
    
    def get_density(self, h):
        """고도에 따른 대기 밀도 계산"""
        return cfg.PhysicsUtils.atmospheric_density(h)
    
    def get_CD_interpolated(self, mach, alpha_deg=0):
        """마하 수 기반 보간된 항력 계수"""
        cd_table = self.CD_TABLE
        mach_values = cd_table['mach']
        cd_values = cd_table['cd']
        
        base_CD = np.interp(mach, mach_values, cd_values)
        
        alpha_factor = 0.05 * (abs(alpha_deg) / 20.0) if alpha_deg != 0 else 0
        
        return base_CD + alpha_factor
    
    def calculate_euler_rates(self, phi, theta, p, q, r):
        """오일러 각도 변화율 계산 (안정화 버전)"""
        cos_theta = np.cos(theta)
        tan_theta = np.tan(theta) if abs(cos_theta) > 0.01 else 0.0
        
        dphi_dt = p + q * np.sin(phi) * tan_theta + r * np.cos(phi) * tan_theta
        dtheta_dt = q * np.cos(phi) - r * np.sin(phi)
        dpsi_euler_dt = (q * np.sin(phi) + r * np.cos(phi)) / (cos_theta + 1e-10)
        
        # 각도 변화율 제한 (급격한 변화 방지)
        max_angle_rate = 5.0  # rad/s
        dphi_dt = np.clip(dphi_dt, -max_angle_rate, max_angle_rate)
        dtheta_dt = np.clip(dtheta_dt, -max_angle_rate, max_angle_rate)
        dpsi_euler_dt = np.clip(dpsi_euler_dt, -max_angle_rate, max_angle_rate)
        
        return dphi_dt, dtheta_dt, dpsi_euler_dt
    
    def calculate_aerodynamic_moments(self, state, q_dynamic):
        """🆕 6DOF: 공력 모멘트 계산 (스무딩 및 제한 적용)"""
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q, r, M_t, fuel = state
        
        # 속도 안전 최소값
        V_safe = max(abs(V), 0.1)
        
        # 받음각 계산 (제한 적용)
        alpha = np.clip(theta - gamma, -np.pi/4, np.pi/4)  # ±45도 제한
        beta = 0.0  # 측면 받음각 (간단화)
        
        # 각속도 정규화 (과도한 회전 방지)
        p_norm = np.clip(p, -10.0, 10.0)  # ±10 rad/s 제한
        q_norm = np.clip(q, -10.0, 10.0)
        r_norm = np.clip(r, -10.0, 10.0)
        
        # 공력 모멘트 계수 (스무딩 적용)
        characteristic_length = self.length / (2 * V_safe)
        
        Cl = self.cl_alpha * alpha + self.cl_p * p_norm * characteristic_length
        Cm = self.cm_alpha * alpha + self.cm_q * q_norm * characteristic_length
        Cn = self.cn_beta * beta + self.cn_r * r_norm * characteristic_length
        
        # 공력 모멘트 (N·m) - 과도한 모멘트 제한
        max_moment = 1e6  # 최대 모멘트 제한
        L_aero = np.clip(q_dynamic * self.wing_area * self.length * Cl, -max_moment, max_moment)
        M_aero = np.clip(q_dynamic * self.wing_area * self.length * Cm, -max_moment, max_moment)
        N_aero = np.clip(q_dynamic * self.wing_area * self.length * Cn, -max_moment, max_moment)
        
        return L_aero, M_aero, N_aero, alpha, beta
    
    def dynamics_vertical_6dof(self, t, state):
        """🆕 6DOF: 수직상승 단계 동역학"""
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state
        
        # 안전 최소값
        V_safe = max(abs(V), 1.0)
        M_safe = max(M_t, 100.0)
        
        # 중력
        g = cfg.G * cfg.R**2 / (cfg.R + h)**2
        
        # 대기 밀도 및 동압
        rho = self.get_density(h)
        q = 0.5 * rho * V_safe**2
        
        # 마하수
        sound_speed = cfg.PhysicsUtils.sound_speed(h)
        mach = V_safe / sound_speed if sound_speed > 0 else 0
        
        # 항력
        CD = self.get_CD_interpolated(mach, 0)
        D = q * CD * self.wing_area
        
        # 추력
        if self.thrust_profile is not None:
            T = self.thrust_profile(t) if t < self.burn_time else 0
        else:
            T = M_safe * g * (self.isp_sea / self.burn_time) if t < self.burn_time else 0
        
        # 연료 소모
        mdot = self.propellant_mass / self.burn_time if t < self.burn_time else 0
        
        # 🆕 6DOF: 공력 모멘트
        L_aero, M_aero, N_aero, alpha, beta = self.calculate_aerodynamic_moments(state, q)
        
        # 병진 운동 방정식
        dV_dt = (T - D) / M_safe - g * np.sin(gamma)
        dgamma_dt = 0.0  # 수직상승 (피치각 일정)
        dpsi_dt = 0.0    # 방위각 일정
        
        dx_dt = V * np.cos(gamma) * np.cos(psi)
        dy_dt = V * np.cos(gamma) * np.sin(psi)
        dh_dt = V * np.sin(gamma)
        
        # 🆕 6DOF: 회전 운동 방정식 (오일러 각도 변화율) - 특이점 방지
        # 🆕 6DOF: 회전 운동 방정식 (헬퍼 함수 사용)
        dphi_dt, dtheta_dt, dpsi_euler_dt = self.calculate_euler_rates(phi, theta, p, q_rate, r)
        
        # 🆕 6DOF: 각속도 변화율 (오일러 방정식)
        dp_dt = (L_aero + (self.inertia_yy - self.inertia_zz) * q_rate * r) / self.inertia_xx
        dq_dt = (M_aero + (self.inertia_zz - self.inertia_xx) * p * r) / self.inertia_yy
        dr_dt = (N_aero + (self.inertia_xx - self.inertia_yy) * p * q_rate) / self.inertia_zz
        
        # 질량 변화
        dM_dt = -mdot
        dfuel_dt = mdot
        
        return [dV_dt, dgamma_dt, dpsi_dt, dx_dt, dy_dt, dh_dt,
                dphi_dt, dtheta_dt, dpsi_euler_dt, dp_dt, dq_dt, dr_dt,
                dM_dt, dfuel_dt]
    
    def dynamics_pitch_6dof(self, t, state):
        """🆕 6DOF: 피치 전환 단계 동역학"""
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state
        
        V_safe = max(abs(V), 1.0)
        M_safe = max(M_t, 100.0)
        
        g = cfg.G * cfg.R**2 / (cfg.R + h)**2
        rho = self.get_density(h)
        q = 0.5 * rho * V_safe**2
        
        sound_speed = cfg.PhysicsUtils.sound_speed(h)
        mach = V_safe / sound_speed if sound_speed > 0 else 0
        
        # 피치 프로그램
        t_in_pitch = t - self.vertical_time
        target_gamma = self.launch_angle_rad - (self.launch_angle_rad - math.radians(self.pitch_angle_deg)) * (t_in_pitch / self.pitch_time)
        
        CD = self.get_CD_interpolated(mach, 0)
        CL = cfg.CL_PITCH
        
        D = q * CD * self.wing_area
        L = q * CL * self.wing_area
        
        if self.thrust_profile is not None:
            T = self.thrust_profile(t) if t < self.burn_time else 0
        else:
            T = M_safe * g * (self.isp_sea / self.burn_time) if t < self.burn_time else 0
        
        mdot = self.propellant_mass / self.burn_time if t < self.burn_time else 0
        
        # 공력 모멘트
        L_aero, M_aero, N_aero, alpha, beta = self.calculate_aerodynamic_moments(state, q)
        
        # 병진 운동
        dV_dt = (T - D) / M_safe - g * np.sin(gamma)
        dgamma_dt = (L + T * 0.1) / (M_safe * V_safe) - g * np.cos(gamma) / V_safe
        dpsi_dt = 0.0
        
        dx_dt = V * np.cos(gamma) * np.cos(psi)
        dy_dt = V * np.cos(gamma) * np.sin(psi)
        dh_dt = V * np.sin(gamma)
        
        # 회전 운동
        dphi_dt = p + q_rate * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        dtheta_dt = q_rate * np.cos(phi) - r * np.sin(phi)
        dpsi_euler_dt = (q_rate * np.sin(phi) + r * np.cos(phi)) / (np.cos(theta) + 1e-10)
        
        dp_dt = (L_aero + (self.inertia_yy - self.inertia_zz) * q_rate * r) / self.inertia_xx
        dq_dt = (M_aero + (self.inertia_zz - self.inertia_xx) * p * r) / self.inertia_yy
        dr_dt = (N_aero + (self.inertia_xx - self.inertia_yy) * p * q_rate) / self.inertia_zz
        
        dM_dt = -mdot
        dfuel_dt = mdot
        
        return [dV_dt, dgamma_dt, dpsi_dt, dx_dt, dy_dt, dh_dt,
                dphi_dt, dtheta_dt, dpsi_euler_dt, dp_dt, dq_dt, dr_dt,
                dM_dt, dfuel_dt]
    
    def dynamics_constant_6dof(self, t, state):
        """🆕 6DOF: 등자세 비행 단계"""
        return self.dynamics_pitch_6dof(t, state)  # 피치와 유사
    
    def dynamics_midcourse_6dof(self, t, state):
        """🆕 6DOF: 중간단계 (관성비행)"""
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state
        
        V_safe = max(abs(V), 0.1)
        M_safe = max(M_t, 100.0)
        
        g = cfg.G * cfg.R**2 / (cfg.R + h)**2
        rho = self.get_density(h)
        q = 0.5 * rho * V_safe**2
        
        sound_speed = cfg.PhysicsUtils.sound_speed(h)
        mach = V_safe / sound_speed if sound_speed > 0 else 0
        
        CD = self.get_CD_interpolated(mach, 0)
        D = q * CD * self.wing_area
        
        # 공력 모멘트
        L_aero, M_aero, N_aero, alpha, beta = self.calculate_aerodynamic_moments(state, q)
        
        # 병진 운동 (추력 없음)
        dV_dt = -D / M_safe - g * np.sin(gamma)
        dgamma_dt = -g * np.cos(gamma) / V_safe
        dpsi_dt = 0.0
        
        dx_dt = V * np.cos(gamma) * np.cos(psi)
        dy_dt = V * np.cos(gamma) * np.sin(psi)
        dh_dt = V * np.sin(gamma)
        
        # 회전 운동
        dphi_dt = p + q_rate * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        dtheta_dt = q_rate * np.cos(phi) - r * np.sin(phi)
        dpsi_euler_dt = (q_rate * np.sin(phi) + r * np.cos(phi)) / (np.cos(theta) + 1e-10)
        
        dp_dt = (L_aero + (self.inertia_yy - self.inertia_zz) * q_rate * r) / self.inertia_xx
        dq_dt = (M_aero + (self.inertia_zz - self.inertia_xx) * p * r) / self.inertia_yy
        dr_dt = (N_aero + (self.inertia_xx - self.inertia_yy) * p * q_rate) / self.inertia_zz
        
        dM_dt = 0.0
        dfuel_dt = 0.0
        
        return [dV_dt, dgamma_dt, dpsi_dt, dx_dt, dy_dt, dh_dt,
                dphi_dt, dtheta_dt, dpsi_euler_dt, dp_dt, dq_dt, dr_dt,
                dM_dt, dfuel_dt]
    
    def event_ground(self, t, state):
        """지면 충돌 이벤트"""
        return state[5]  # h (고도)
    
    event_ground.terminal = True
    event_ground.direction = -1
    
    def run_simulation(self, sim_time=None):
        """6DOF 시뮬레이션 실행"""
        if sim_time is not None:
            self.sim_time = sim_time
        
        print("=" * 60)
        print("6DOF 미사일 시뮬레이션 시작")
        print("=" * 60)
        
        # 1. 수직상승
        print("1단계: 수직상승")
        t_vertical_end = self.vertical_time
        
        sol_vertical = solve_ivp(
            self.dynamics_vertical_6dof,
            [0, t_vertical_end],
            self.initial_state,
            method='RK45',
            dense_output=True
        )
        
        if len(sol_vertical.t) > 0:
            t_dense = np.linspace(0, t_vertical_end, int(t_vertical_end / 0.1) + 1)
            if sol_vertical.sol is not None:
                states_dense = sol_vertical.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())
            
            last_state_after_vertical = sol_vertical.y[:, -1]
        else:
            print("수직상승 단계 실패")
            return None
        
        # 2. 피치 전환
        print("2단계: 피치 전환")
        t_pitch_start = t_vertical_end
        t_pitch_end = t_vertical_end + self.pitch_time
        
        sol_pitch = solve_ivp(
            self.dynamics_pitch_6dof,
            [t_pitch_start, t_pitch_end],
            last_state_after_vertical,
            method='RK45',
            dense_output=True
        )
        
        if len(sol_pitch.t) > 0:
            t_dense = np.linspace(t_pitch_start, t_pitch_end, int(self.pitch_time / 0.1) + 1)
            if sol_pitch.sol is not None:
                states_dense = sol_pitch.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())
            
            last_state_after_pitch = sol_pitch.y[:, -1]
        else:
            last_state_after_pitch = last_state_after_vertical
        
        # 3. 등자세 비행
        print("3단계: 등자세 비행")
        t_constant_start = t_pitch_end
        t_constant_end = self.burn_time
        
        if t_constant_end > t_constant_start:
            sol_constant = solve_ivp(
                self.dynamics_constant_6dof,
                [t_constant_start, t_constant_end],
                last_state_after_pitch,
                method='RK45',
                dense_output=True
            )
            
            if len(sol_constant.t) > 0:
                t_dense = np.linspace(t_constant_start, t_constant_end, int((t_constant_end - t_constant_start) / 0.1) + 1)
                if sol_constant.sol is not None:
                    states_dense = sol_constant.sol(t_dense).T
                    self.t.extend(t_dense.tolist())
                    self.states.extend(states_dense.tolist())
                
                last_state_after_constant = sol_constant.y[:, -1]
            else:
                last_state_after_constant = last_state_after_pitch
        else:
            last_state_after_constant = last_state_after_pitch
        
        # 4. 중간단계 (관성비행)
        print("4단계: 중간단계 비행")
        t_mid_start = t_constant_end
        t_mid_end = self.sim_time
        
        sol_mid = solve_ivp(
            self.dynamics_midcourse_6dof,
            [t_mid_start, t_mid_end],
            last_state_after_constant,
            method='RK45',
            events=[self.event_ground],
            dense_output=True
        )
        
        # 지면 충돌 처리
        collision_times = sol_mid.t_events[0] if sol_mid.t_events else []
        if len(collision_times) > 0:
            t_ground = collision_times[0]
            print(f"지면 충돌 감지: {t_ground:.2f}초")
            t_dense = np.linspace(t_mid_start, t_ground, int((t_ground - t_mid_start) / 0.1) + 1)
            if sol_mid.sol is not None:
                states_dense = sol_mid.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())
        else:
            t_dense = np.linspace(t_mid_start, t_mid_end, int((t_mid_end - t_mid_start) / 0.1) + 1)
            if sol_mid.sol is not None:
                states_dense = sol_mid.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())
        
        print(f"시뮬레이션 계산 완료! 전체 비행 시간: {self.t[-1]:.2f}초")
        
        # 결과 저장
        states_array = np.array(self.states)
        self.results = {
            'time': np.array(self.t),
            'velocity': states_array[:, 0],
            'gamma': states_array[:, 1],
            'psi': states_array[:, 2],
            'x': states_array[:, 3],
            'y': states_array[:, 4],
            'h': states_array[:, 5],
            'phi': states_array[:, 6],
            'theta': states_array[:, 7],
            'psi_euler': states_array[:, 8],
            'p': states_array[:, 9],
            'q': states_array[:, 10],
            'r': states_array[:, 11],
            'mass': states_array[:, 12],
            'fuel': states_array[:, 13]
        }
        
        return self.results
    
    def plot_results_6dof(self):
        """🆕 6DOF 결과 시각화"""
        if self.results is None:
            print("시뮬레이션 결과가 없습니다.")
            return
        
        fig = plt.figure(figsize=(20, 12))
        
        # 1. 3D 궤적
        ax1 = fig.add_subplot(3, 4, 1, projection='3d')
        ax1.plot(self.results['x']/1000, self.results['y']/1000, self.results['h']/1000, 'b-')
        ax1.set_xlabel('X (km)')
        ax1.set_ylabel('Y (km)')
        ax1.set_zlabel('Altitude (km)')
        ax1.set_title('3D Trajectory')
        
        # 2. 속도
        ax2 = fig.add_subplot(3, 4, 2)
        ax2.plot(self.results['time'], self.results['velocity'])
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity')
        ax2.grid(True)
        
        # 3. 고도
        ax3 = fig.add_subplot(3, 4, 3)
        ax3.plot(self.results['time'], self.results['h']/1000)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Altitude (km)')
        ax3.set_title('Altitude')
        ax3.grid(True)
        
        # 4. 비행경로각
        ax4 = fig.add_subplot(3, 4, 4)
        ax4.plot(self.results['time'], np.rad2deg(self.results['gamma']))
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Flight Path Angle (deg)')
        ax4.set_title('Flight Path Angle')
        ax4.grid(True)
        
        # 🆕 5. 롤각
        ax5 = fig.add_subplot(3, 4, 5)
        ax5.plot(self.results['time'], np.rad2deg(self.results['phi']))
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Roll Angle (deg)')
        ax5.set_title('Roll Angle (φ)')
        ax5.grid(True)
        
        # 🆕 6. 피치각
        ax6 = fig.add_subplot(3, 4, 6)
        ax6.plot(self.results['time'], np.rad2deg(self.results['theta']))
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Pitch Angle (deg)')
        ax6.set_title('Pitch Angle (θ)')
        ax6.grid(True)
        
        # 🆕 7. 요각
        ax7 = fig.add_subplot(3, 4, 7)
        ax7.plot(self.results['time'], np.rad2deg(self.results['psi_euler']))
        ax7.set_xlabel('Time (s)')
        ax7.set_ylabel('Yaw Angle (deg)')
        ax7.set_title('Yaw Angle (ψ)')
        ax7.grid(True)
        
        # 🆕 8. 롤 각속도
        ax8 = fig.add_subplot(3, 4, 8)
        ax8.plot(self.results['time'], np.rad2deg(self.results['p']))
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('Roll Rate (deg/s)')
        ax8.set_title('Roll Rate (p)')
        ax8.grid(True)
        
        # 🆕 9. 피치 각속도
        ax9 = fig.add_subplot(3, 4, 9)
        ax9.plot(self.results['time'], np.rad2deg(self.results['q']))
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Pitch Rate (deg/s)')
        ax9.set_title('Pitch Rate (q)')
        ax9.grid(True)
        
        # 🆕 10. 요 각속도
        ax10 = fig.add_subplot(3, 4, 10)
        ax10.plot(self.results['time'], np.rad2deg(self.results['r']))
        ax10.set_xlabel('Time (s)')
        ax10.set_ylabel('Yaw Rate (deg/s)')
        ax10.set_title('Yaw Rate (r)')
        ax10.grid(True)
        
        # 11. 질량
        ax11 = fig.add_subplot(3, 4, 11)
        ax11.plot(self.results['time'], self.results['mass'])
        ax11.set_xlabel('Time (s)')
        ax11.set_ylabel('Mass (kg)')
        ax11.set_title('Mass')
        ax11.grid(True)
        
        # 12. Range vs Altitude
        ax12 = fig.add_subplot(3, 4, 12)
        range_km = np.sqrt(self.results['x']**2 + self.results['y']**2) / 1000
        ax12.plot(range_km, self.results['h']/1000)
        ax12.set_xlabel('Range (km)')
        ax12.set_ylabel('Altitude (km)')
        ax12.set_title('Range vs Altitude')
        ax12.grid(True)
        
        plt.tight_layout()
        
        # 저장
        os.makedirs("results_6dof", exist_ok=True)
        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        save_path = f"results_6dof/6dof_results_{now_str}.png"
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"6DOF 결과 저장: {save_path}")
        plt.show()
        
        return save_path

def main():
    """메인 함수"""
    print("6DOF 미사일 궤적 시뮬레이션을 시작합니다...")
    
    simulation = MissileSimulation6DOF(missile_type="SCUD-B", apply_errors=False)
    
    simulation.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=1500)
    
    results = simulation.run_simulation()
    
    if results is not None:
        simulation.plot_results_6dof()
        
        # 결과 요약
        final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
        max_altitude = np.max(results['h']) / 1000
        final_velocity = results['velocity'][-1]
        
        print("\n" + "=" * 60)
        print("6DOF 시뮬레이션 결과 요약")
        print("=" * 60)
        print(f"최종 사거리: {final_range:.2f} km")
        print(f"최대 고도: {max_altitude:.2f} km")
        print(f"최종 속도: {final_velocity:.2f} m/s")
        print(f"비행 시간: {results['time'][-1]:.2f} s")
        print(f"최종 롤각: {np.rad2deg(results['phi'][-1]):.2f}°")
        print(f"최종 피치각: {np.rad2deg(results['theta'][-1]):.2f}°")
        print(f"최종 요각: {np.rad2deg(results['psi_euler'][-1]):.2f}°")
        print("=" * 60)
    
    print("6DOF 미사일 궤적 시뮬레이션이 완료되었습니다.")

if __name__ == "__main__":
    main()
