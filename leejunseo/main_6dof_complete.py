#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete 6DOF Missile Trajectory Simulation
진정한 6DOF 물리 시뮬레이션 - 모든 좌표계 변환 및 교차 결합 효과 포함

주요 개선 사항:
1. DCM을 사용한 동체/지구 좌표계 변환
2. 적절한 측면 받음각(β) 계산
3. 추력 벡터의 동체 좌표계 정렬
4. 공력 모멘트 팔 (moment arm) 추가
5. 교차 결합 효과 (Coriolis force)
6. 비선형 공기역학 계수
7. 연료 소모에 따른 질량 분포 변화
"""
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import datetime
from scipy.integrate import solve_ivp
import config_6dof_complete as cfg
import math

# matplotlib 설정
plt.rcParams['axes.unicode_minus'] = False

class Complete6DOFSimulation:
    """완전한 6DOF 미사일 궤적 시뮬레이션"""

    def __init__(self, missile_type="SCUD-B", apply_errors=False):
        """생성자"""
        self.results = None
        self.states = []
        self.t = []

        # 추적 데이터
        self.alpha_list = []
        self.beta_list = []
        self.CD_list = []
        self.CL_list = []
        self.fuel_list = []
        self.mach_list = []
        self.phase_list = []

        # 6DOF 추적
        self.roll_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []

        # 무게중심 및 관성 모멘트 추적
        self.cg_list = []
        self.inertia_list = []

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
        """시뮬레이션 초기화"""
        self.alpha_list = []
        self.beta_list = []
        self.CD_list = []
        self.CL_list = []
        self.fuel_list = []
        self.mach_list = []
        self.phase_list = []
        self.roll_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        self.cg_list = []
        self.inertia_list = []

        self.results = {
            'time': [], 'velocity': [], 'gamma': [], 'psi': [],
            'x': [], 'y': [], 'h': [], 'mass': [],
            'phi': [], 'theta': [], 'psi_euler': [],
            'p': [], 'q': [], 'r': [],
            'alpha': [], 'beta': [], 'CD': [], 'CL': [],
            'fuel': [], 'mach': [], 'phase': [],
            'cg': [], 'Ixx': [], 'Iyy': [], 'Izz': []
        }

        self.sim_time = sim_time if sim_time is not None else 1500

        self.init_speed = 0.1
        self.launch_angle_rad = math.radians(launch_angle_deg)
        self.launch_azimuth_rad = math.radians(azimuth_deg)

        missile_info = cfg.MISSILE_TYPES[self.missile_type]

        # 14차원 상태 벡터
        self.initial_state = cfg.StateVector6DOF.create_initial_state(
            missile_info, launch_angle_deg, azimuth_deg
        )

        print(f"Complete 6DOF 초기화: {self.missile_type}, 발사각 {launch_angle_deg}°, 방위각 {azimuth_deg}°")

    def update_missile_type(self, missile_type):
        """미사일 유형 업데이트"""
        if missile_type in cfg.MISSILE_TYPES:
            self.missile_type = missile_type
            missile_data = cfg.MISSILE_TYPES[missile_type]

            self.diameter = missile_data["diameter"]
            self.length = missile_data["length"]
            self.nozzle_diameter = missile_data["nozzle_diameter"]
            self.propellant_type = missile_data["propellant_type"]

            self.missile_mass = missile_data["launch_weight"]
            self.propellant_mass = missile_data["propellant_mass"]
            self.structural_mass = missile_data["structural_mass"]
            self.isp_sea = missile_data["isp_sea"]
            self.isp_vacuum = missile_data.get("isp_vac", missile_data.get("isp_vacuum", self.isp_sea * 1.1))
            self.burn_time = missile_data["burn_time"]

            self.wing_area = missile_data.get("reference_area", np.pi * (self.diameter/2)**2)

            self.vertical_time = missile_data["vertical_time"]
            self.pitch_time = missile_data["pitch_time"]
            self.pitch_angle_deg = missile_data["pitch_angle_deg"]

            # 관성 모멘트 (초기값)
            self.inertia_xx_empty = missile_data["inertia_xx_empty"]
            self.inertia_yy_empty = missile_data["inertia_yy_empty"]
            self.inertia_zz_empty = missile_data["inertia_zz_empty"]

            # 공력 모멘트 계수
            self.cl_alpha = missile_data["cl_alpha"]
            self.cm_alpha = missile_data["cm_alpha"]
            self.cn_beta = missile_data["cn_beta"]
            self.cl_p = missile_data["cl_p"]
            self.cm_q = missile_data["cm_q"]
            self.cn_r = missile_data["cn_r"]

            # 무게중심 및 압력중심
            self.cg_location_full = missile_data["cg_location_full"]
            self.cg_location_empty = missile_data["cg_location_empty"]
            self.cp_location = missile_data["cp_location"]
            self.fuel_tank_location = missile_data["fuel_tank_location"]

            self.CD_TABLE = cfg.get_cd_table_for_missile(missile_type)

            print(f"Complete 6DOF 미사일 '{missile_type}' 설정 완료")
            return True
        else:
            print(f"경고: 미사일 유형 '{missile_type}' 찾을 수 없음")
            return False

    def get_density(self, h):
        """고도에 따른 대기 밀도"""
        return cfg.PhysicsUtils.atmospheric_density(h)

    def get_variable_mass_properties(self, fuel_consumed):
        """
        연료 소모에 따른 질량 분포 변화 계산

        Returns:
            cg_location: 현재 무게중심 위치
            Ixx, Iyy, Izz: 현재 관성 모멘트
        """
        fuel_remaining = max(0, self.propellant_mass - fuel_consumed)
        fuel_fraction = fuel_remaining / self.propellant_mass if self.propellant_mass > 0 else 0

        # 무게중심 선형 보간
        cg_location = self.cg_location_empty + fuel_fraction * (self.cg_location_full - self.cg_location_empty)

        # 관성 모멘트 변화 (연료 탱크의 기여도 감소)
        # 간단한 모델: 연료 질량에 비례
        fuel_contribution_factor = fuel_fraction * 1.2  # 연료가 있을 때 관성 증가

        Ixx = self.inertia_xx_empty * (1 + fuel_contribution_factor)
        Iyy = self.inertia_yy_empty * (1 + fuel_contribution_factor)
        Izz = self.inertia_zz_empty * (1 + fuel_contribution_factor * 0.5)  # 롤 관성은 덜 변함

        return cg_location, Ixx, Iyy, Izz

    def get_CD_interpolated(self, mach, alpha_deg=0):
        """마하 수 기반 보간된 항력 계수 (기본)"""
        cd_table = self.CD_TABLE
        mach_values = cd_table['mach']
        cd_values = cd_table['cd']

        base_CD = np.interp(mach, mach_values, cd_values)

        # 받음각 효과
        alpha_factor = 0.05 * (abs(alpha_deg) / 20.0) if alpha_deg != 0 else 0

        return base_CD + alpha_factor

    def calculate_aerodynamic_forces_and_moments(self, state, t):
        """
        완전한 공기역학적 힘 및 모멘트 계산

        Returns:
            F_aero_body: 동체 좌표계 공력 벡터 [Fx, Fy, Fz]
            M_aero_body: 동체 좌표계 모멘트 벡터 [L, M, N]
            alpha, beta: 받음각, 측면 받음각
            CL, CD: 양력 계수, 항력 계수
        """
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state

        # 속도 벡터 (지구 좌표계)
        v_earth = np.array([
            V * np.cos(gamma) * np.cos(psi),
            V * np.cos(gamma) * np.sin(psi),
            V * np.sin(gamma)
        ])

        # DCM: 지구 → 동체
        dcm_e2b = cfg.CoordinateTransforms.earth_to_body_dcm(phi, theta, psi_euler)

        # 속도를 동체 좌표계로 변환
        v_body = np.dot(dcm_e2b, v_earth)

        # 받음각 및 측면 받음각 계산
        alpha, beta = cfg.CoordinateTransforms.calculate_angles_of_attack(v_body)

        # 대기 밀도 및 동압
        rho = self.get_density(h)
        q_dynamic = 0.5 * rho * V**2

        # 마하수
        sound_speed = cfg.PhysicsUtils.sound_speed(h)
        mach = V / sound_speed if sound_speed > 0 else 0

        # 비선형 항력 및 양력 계수
        CD = cfg.PhysicsUtils.drag_coefficient_nonlinear(mach, alpha, beta)
        CL = cfg.PhysicsUtils.nonlinear_cl_alpha(alpha, mach)

        # 공력 (바람 좌표계)
        D = q_dynamic * self.wing_area * CD  # 항력
        L = q_dynamic * self.wing_area * CL  # 양력
        Y = 0.0  # 측면력 (간단화)

        # 바람 → 동체 좌표계 변환
        dcm_w2b = cfg.CoordinateTransforms.wind_to_body_dcm(alpha, beta)

        # 공력 벡터 (바람 좌표계: [-D, Y, -L])
        F_aero_wind = np.array([-D, Y, -L])

        # 동체 좌표계로 변환
        F_aero_body = np.dot(dcm_w2b, F_aero_wind)

        # 공력 모멘트 계산
        cg_location, Ixx, Iyy, Izz = self.get_variable_mass_properties(fuel)
        moment_arm = self.cp_location - cg_location

        # 무차원 각속도
        p_hat = p * self.length / (2 * V + 0.1)
        q_hat = q_rate * self.length / (2 * V + 0.1)
        r_hat = r * self.length / (2 * V + 0.1)

        # 모멘트 계수
        Cl = self.cl_alpha * beta + self.cl_p * p_hat  # 롤 모멘트
        Cm = self.cm_alpha * alpha + self.cm_q * q_hat  # 피치 모멘트
        Cn = self.cn_beta * beta + self.cn_r * r_hat    # 요 모멘트

        # 모멘트 (N·m)
        L_aero = q_dynamic * self.wing_area * self.diameter * Cl
        M_aero = q_dynamic * self.wing_area * moment_arm * Cm  # 모멘트 팔 적용
        N_aero = q_dynamic * self.wing_area * self.diameter * Cn

        M_aero_body = np.array([L_aero, M_aero, N_aero])

        return F_aero_body, M_aero_body, alpha, beta, CL, CD

    def calculate_thrust_vector(self, t, state):
        """
        추력 벡터 계산 (동체 좌표계)

        Returns:
            T_body: 동체 좌표계 추력 벡터 [Tx, Ty, Tz]
        """
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state

        # 추력 크기
        if t < self.burn_time:
            # ISP는 고도에 따라 변화
            altitude_factor = min(1.0, h / 30000)
            isp = self.isp_sea + (self.isp_vacuum - self.isp_sea) * altitude_factor

            T_magnitude = (self.propellant_mass / self.burn_time) * isp * cfg.G
        else:
            T_magnitude = 0.0

        # 추력은 동체 x축을 따라 작용
        T_body = np.array([T_magnitude, 0.0, 0.0])

        return T_body

    def dynamics_complete_6dof(self, t, state, phase="midcourse"):
        """
        완전한 6DOF 동역학 방정식

        phase: "vertical", "pitch", "constant", "midcourse"
        """
        V, gamma, psi, x, y, h, phi, theta, psi_euler, p, q_rate, r, M_t, fuel = state

        # 안전 체크
        V_safe = max(abs(V), 0.1)
        M_safe = max(M_t, 100.0)

        # 중력
        g = cfg.PhysicsUtils.gravity_at_altitude(h)

        # 질량 분포
        cg_location, Ixx, Iyy, Izz = self.get_variable_mass_properties(fuel)

        # DCM: 동체 → 지구
        dcm_b2e = cfg.CoordinateTransforms.body_to_earth_dcm(phi, theta, psi_euler)

        # 추력 (동체 좌표계)
        T_body = self.calculate_thrust_vector(t, state)

        # 공기역학 (동체 좌표계)
        F_aero_body, M_aero_body, alpha, beta, CL, CD = self.calculate_aerodynamic_forces_and_moments(state, t)

        # 총 힘 (동체 좌표계)
        F_total_body = T_body + F_aero_body

        # 지구 좌표계로 변환
        F_total_earth = np.dot(dcm_b2e, F_total_body)

        # 중력 (지구 좌표계)
        g_earth = np.array([0, 0, -g * M_safe])

        # 총 힘 (지구 좌표계)
        F_total_earth += g_earth

        # 병진 가속도 (지구 좌표계)
        a_earth = F_total_earth / M_safe

        # 속도 미분 (극좌표 형식)
        # v_earth = [vx, vy, vz]
        vx = V * np.cos(gamma) * np.cos(psi)
        vy = V * np.cos(gamma) * np.sin(psi)
        vz = V * np.sin(gamma)

        # 가속도 성분
        ax, ay, az = a_earth

        # 속도 크기 변화율
        dV_dt = (ax * vx + ay * vy + az * vz) / (V_safe + 0.01)

        # 비행경로각 변화율
        v_horizontal = V * np.cos(gamma)
        if v_horizontal > 0.1:
            dgamma_dt = (az * np.cos(gamma) - (ax * np.cos(psi) + ay * np.sin(psi)) * np.sin(gamma)) / (V_safe + 0.01)
        else:
            dgamma_dt = 0.0

        # 방위각 변화율
        if v_horizontal > 0.1:
            dpsi_dt = (ay * np.cos(psi) - ax * np.sin(psi)) / (v_horizontal + 0.01)
        else:
            dpsi_dt = 0.0

        # 위치 변화율
        dx_dt = vx
        dy_dt = vy
        dh_dt = vz

        # 회전 운동 방정식 (오일러 방정식)
        # 각속도 벡터
        omega = np.array([p, q_rate, r])

        # 자이로스코픽 모멘트
        gyro_moment = np.array([
            (Iyy - Izz) * q_rate * r,
            (Izz - Ixx) * p * r,
            (Ixx - Iyy) * p * q_rate
        ])

        # 각가속도
        alpha_angular = (M_aero_body - gyro_moment) / np.array([Ixx, Iyy, Izz])
        dp_dt, dq_dt, dr_dt = alpha_angular

        # 오일러각 변화율
        dphi_dt, dtheta_dt, dpsi_euler_dt = cfg.CoordinateTransforms.velocity_to_euler_rates(
            phi, theta, psi_euler, p, q_rate, r
        )

        # 질량 변화
        mdot = self.propellant_mass / self.burn_time if t < self.burn_time else 0.0
        dM_dt = -mdot
        dfuel_dt = mdot

        # 특수 비행 단계 처리
        if phase == "vertical":
            # 수직상승: 피치 전환 억제
            dgamma_dt *= 0.1
            dpsi_dt *= 0.1

        elif phase == "pitch":
            # 피치 전환
            t_in_pitch = t - self.vertical_time
            target_gamma = self.launch_angle_rad - (
                self.launch_angle_rad - math.radians(self.pitch_angle_deg)
            ) * (t_in_pitch / self.pitch_time)

            # 피치 제어 모멘트 추가 (간단한 제어)
            gamma_error = target_gamma - gamma
            dq_dt += 0.5 * gamma_error  # 비례 제어

        return [dV_dt, dgamma_dt, dpsi_dt, dx_dt, dy_dt, dh_dt,
                dphi_dt, dtheta_dt, dpsi_euler_dt, dp_dt, dq_dt, dr_dt,
                dM_dt, dfuel_dt]

    def dynamics_vertical(self, t, state):
        """수직상승 단계"""
        return self.dynamics_complete_6dof(t, state, phase="vertical")

    def dynamics_pitch(self, t, state):
        """피치 전환 단계"""
        return self.dynamics_complete_6dof(t, state, phase="pitch")

    def dynamics_constant(self, t, state):
        """등자세 비행 단계"""
        return self.dynamics_complete_6dof(t, state, phase="constant")

    def dynamics_midcourse(self, t, state):
        """중간단계 (관성비행)"""
        return self.dynamics_complete_6dof(t, state, phase="midcourse")

    def event_ground(self, t, state):
        """지면 충돌 이벤트"""
        return state[5]  # h (고도)

    event_ground.terminal = True
    event_ground.direction = -1

    def run_simulation(self, sim_time=None):
        """Complete 6DOF 시뮬레이션 실행"""
        if sim_time is not None:
            self.sim_time = sim_time

        print("=" * 70)
        print("Complete 6DOF 미사일 시뮬레이션 시작")
        print("=" * 70)

        # 1. 수직상승
        print("1단계: 수직상승")
        t_vertical_end = self.vertical_time

        sol_vertical = solve_ivp(
            self.dynamics_vertical,
            [0, t_vertical_end],
            self.initial_state,
            method='RK45',
            max_step=0.1,
            dense_output=True
        )

        if len(sol_vertical.t) > 0:
            t_dense = np.linspace(0, t_vertical_end, int(t_vertical_end / 0.1) + 1)
            if sol_vertical.sol is not None:
                states_dense = sol_vertical.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())

            last_state = sol_vertical.y[:, -1]
        else:
            print("수직상승 단계 실패")
            return None

        # 2. 피치 전환
        print("2단계: 피치 전환")
        t_pitch_start = t_vertical_end
        t_pitch_end = t_vertical_end + self.pitch_time

        sol_pitch = solve_ivp(
            self.dynamics_pitch,
            [t_pitch_start, t_pitch_end],
            last_state,
            method='RK45',
            max_step=0.1,
            dense_output=True
        )

        if len(sol_pitch.t) > 0:
            t_dense = np.linspace(t_pitch_start, t_pitch_end, int(self.pitch_time / 0.1) + 1)
            if sol_pitch.sol is not None:
                states_dense = sol_pitch.sol(t_dense).T
                self.t.extend(t_dense.tolist())
                self.states.extend(states_dense.tolist())

            last_state = sol_pitch.y[:, -1]

        # 3. 등자세 비행
        print("3단계: 등자세 비행")
        t_constant_start = t_pitch_end
        t_constant_end = self.burn_time

        if t_constant_end > t_constant_start:
            sol_constant = solve_ivp(
                self.dynamics_constant,
                [t_constant_start, t_constant_end],
                last_state,
                method='RK45',
                max_step=0.1,
                dense_output=True
            )

            if len(sol_constant.t) > 0:
                t_dense = np.linspace(t_constant_start, t_constant_end,
                                    int((t_constant_end - t_constant_start) / 0.1) + 1)
                if sol_constant.sol is not None:
                    states_dense = sol_constant.sol(t_dense).T
                    self.t.extend(t_dense.tolist())
                    self.states.extend(states_dense.tolist())

                last_state = sol_constant.y[:, -1]

        # 4. 중간단계 (관성비행)
        print("4단계: 중간단계 비행")
        t_mid_start = t_constant_end
        t_mid_end = self.sim_time

        sol_mid = solve_ivp(
            self.dynamics_midcourse,
            [t_mid_start, t_mid_end],
            last_state,
            method='RK45',
            max_step=0.5,
            events=[self.event_ground],
            dense_output=True
        )

        # 지면 충돌 처리
        collision_times = sol_mid.t_events[0] if sol_mid.t_events else []
        if len(collision_times) > 0:
            t_ground = collision_times[0]
            print(f"지면 충돌 감지: {t_ground:.2f}초")
            t_dense = np.linspace(t_mid_start, t_ground, int((t_ground - t_mid_start) / 0.5) + 1)
        else:
            t_dense = np.linspace(t_mid_start, t_mid_end, int((t_mid_end - t_mid_start) / 0.5) + 1)

        if sol_mid.sol is not None:
            states_dense = sol_mid.sol(t_dense).T
            self.t.extend(t_dense.tolist())
            self.states.extend(states_dense.tolist())

        print(f"시뮬레이션 계산 완료! 전체 비행 시간: {self.t[-1]:.2f}초")

        # 결과 저장 및 후처리
        states_array = np.array(self.states)

        # 추가 데이터 계산
        for i, (time, state) in enumerate(zip(self.t, states_array)):
            _, _, alpha, beta, CL, CD = self.calculate_aerodynamic_forces_and_moments(state, time)
            cg, Ixx, Iyy, Izz = self.get_variable_mass_properties(state[13])

            self.alpha_list.append(alpha)
            self.beta_list.append(beta)
            self.CL_list.append(CL)
            self.CD_list.append(CD)
            self.cg_list.append(cg)
            self.inertia_list.append([Ixx, Iyy, Izz])

            V = state[0]
            h = state[5]
            mach = V / cfg.PhysicsUtils.sound_speed(h)
            self.mach_list.append(mach)

        inertia_array = np.array(self.inertia_list)

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
            'fuel': states_array[:, 13],
            'alpha': np.array(self.alpha_list),
            'beta': np.array(self.beta_list),
            'CL': np.array(self.CL_list),
            'CD': np.array(self.CD_list),
            'mach': np.array(self.mach_list),
            'cg': np.array(self.cg_list),
            'Ixx': inertia_array[:, 0],
            'Iyy': inertia_array[:, 1],
            'Izz': inertia_array[:, 2],
        }

        return self.results

    def plot_results_complete(self):
        """Complete 6DOF 결과 시각화 (16개 서브플롯)"""
        if self.results is None:
            print("시뮬레이션 결과가 없습니다.")
            return

        fig = plt.figure(figsize=(24, 18))
        plt.rcParams.update({'font.size': 8})

        # 1. 3D 궤적
        ax1 = fig.add_subplot(4, 4, 1, projection='3d')
        ax1.plot(self.results['x']/1000, self.results['y']/1000, self.results['h']/1000, 'b-')
        ax1.set_xlabel('X (km)', fontsize=8)
        ax1.set_ylabel('Y (km)', fontsize=8)
        ax1.set_zlabel('Altitude (km)', fontsize=8)
        ax1.set_title('3D Trajectory', fontsize=9)

        # 2. 속도
        ax2 = fig.add_subplot(4, 4, 2)
        ax2.plot(self.results['time'], self.results['velocity'])
        ax2.set_xlabel('Time (s)', fontsize=8)
        ax2.set_ylabel('Velocity (m/s)', fontsize=8)
        ax2.set_title('Velocity', fontsize=9)
        ax2.grid(True)

        # 3. 고도
        ax3 = fig.add_subplot(4, 4, 3)
        ax3.plot(self.results['time'], self.results['h']/1000)
        ax3.set_xlabel('Time (s)', fontsize=8)
        ax3.set_ylabel('Altitude (km)', fontsize=8)
        ax3.set_title('Altitude', fontsize=9)
        ax3.grid(True)

        # 4. 비행경로각
        ax4 = fig.add_subplot(4, 4, 4)
        ax4.plot(self.results['time'], np.rad2deg(self.results['gamma']))
        ax4.set_xlabel('Time (s)', fontsize=8)
        ax4.set_ylabel('FPA (deg)', fontsize=8)
        ax4.set_title('Flight Path Angle', fontsize=9)
        ax4.grid(True)

        # 5. 롤각
        ax5 = fig.add_subplot(4, 4, 5)
        ax5.plot(self.results['time'], np.rad2deg(self.results['phi']))
        ax5.set_xlabel('Time (s)', fontsize=8)
        ax5.set_ylabel('Roll (deg)', fontsize=8)
        ax5.set_title('Roll Angle (φ)', fontsize=9)
        ax5.grid(True)

        # 6. 피치각
        ax6 = fig.add_subplot(4, 4, 6)
        ax6.plot(self.results['time'], np.rad2deg(self.results['theta']))
        ax6.set_xlabel('Time (s)', fontsize=8)
        ax6.set_ylabel('Pitch (deg)', fontsize=8)
        ax6.set_title('Pitch Angle (θ)', fontsize=9)
        ax6.grid(True)

        # 7. 요각
        ax7 = fig.add_subplot(4, 4, 7)
        ax7.plot(self.results['time'], np.rad2deg(self.results['psi_euler']))
        ax7.set_xlabel('Time (s)', fontsize=8)
        ax7.set_ylabel('Yaw (deg)', fontsize=8)
        ax7.set_title('Yaw Angle (ψ)', fontsize=9)
        ax7.grid(True)

        # 8. 받음각 및 측면 받음각
        ax8 = fig.add_subplot(4, 4, 8)
        ax8.plot(self.results['time'], np.rad2deg(self.results['alpha']), label='α (AoA)')
        ax8.plot(self.results['time'], np.rad2deg(self.results['beta']), label='β (Sideslip)')
        ax8.set_xlabel('Time (s)', fontsize=8)
        ax8.set_ylabel('Angle (deg)', fontsize=8)
        ax8.set_title('Angles of Attack', fontsize=9)
        ax8.legend(fontsize=7)
        ax8.grid(True)

        # 9. 롤 각속도
        ax9 = fig.add_subplot(4, 4, 9)
        ax9.plot(self.results['time'], np.rad2deg(self.results['p']))
        ax9.set_xlabel('Time (s)', fontsize=8)
        ax9.set_ylabel('Roll Rate (deg/s)', fontsize=8)
        ax9.set_title('Roll Rate (p)', fontsize=9)
        ax9.grid(True)

        # 10. 피치 각속도
        ax10 = fig.add_subplot(4, 4, 10)
        ax10.plot(self.results['time'], np.rad2deg(self.results['q']))
        ax10.set_xlabel('Time (s)', fontsize=8)
        ax10.set_ylabel('Pitch Rate (deg/s)', fontsize=8)
        ax10.set_title('Pitch Rate (q)', fontsize=9)
        ax10.grid(True)

        # 11. 요 각속도
        ax11 = fig.add_subplot(4, 4, 11)
        ax11.plot(self.results['time'], np.rad2deg(self.results['r']))
        ax11.set_xlabel('Time (s)', fontsize=8)
        ax11.set_ylabel('Yaw Rate (deg/s)', fontsize=8)
        ax11.set_title('Yaw Rate (r)', fontsize=9)
        ax11.grid(True)

        # 12. 항력 및 양력 계수
        ax12 = fig.add_subplot(4, 4, 12)
        ax12.plot(self.results['time'], self.results['CD'], label='CD')
        ax12.plot(self.results['time'], self.results['CL'], label='CL')
        ax12.set_xlabel('Time (s)', fontsize=8)
        ax12.set_ylabel('Coefficient', fontsize=8)
        ax12.set_title('Aero Coefficients', fontsize=9)
        ax12.legend(fontsize=7)
        ax12.grid(True)

        # 13. 질량
        ax13 = fig.add_subplot(4, 4, 13)
        ax13.plot(self.results['time'], self.results['mass'])
        ax13.set_xlabel('Time (s)', fontsize=8)
        ax13.set_ylabel('Mass (kg)', fontsize=8)
        ax13.set_title('Mass', fontsize=9)
        ax13.grid(True)

        # 14. 무게중심 위치
        ax14 = fig.add_subplot(4, 4, 14)
        ax14.plot(self.results['time'], self.results['cg'])
        ax14.set_xlabel('Time (s)', fontsize=8)
        ax14.set_ylabel('CG Location (m)', fontsize=8)
        ax14.set_title('Center of Gravity', fontsize=9)
        ax14.grid(True)

        # 15. 관성 모멘트
        ax15 = fig.add_subplot(4, 4, 15)
        ax15.plot(self.results['time'], self.results['Ixx'], label='Ixx')
        ax15.plot(self.results['time'], self.results['Iyy'], label='Iyy')
        ax15.plot(self.results['time'], self.results['Izz'], label='Izz')
        ax15.set_xlabel('Time (s)', fontsize=8)
        ax15.set_ylabel('Inertia (kg·m²)', fontsize=8)
        ax15.set_title('Moments of Inertia', fontsize=9)
        ax15.legend(fontsize=7)
        ax15.grid(True)

        # 16. Range vs Altitude
        ax16 = fig.add_subplot(4, 4, 16)
        range_km = np.sqrt(self.results['x']**2 + self.results['y']**2) / 1000
        ax16.plot(range_km, self.results['h']/1000)
        ax16.set_xlabel('Range (km)', fontsize=8)
        ax16.set_ylabel('Altitude (km)', fontsize=8)
        ax16.set_title('Range vs Altitude', fontsize=9)
        ax16.grid(True)

        plt.tight_layout(pad=2.0)

        # 저장
        os.makedirs("results_6dof_complete", exist_ok=True)
        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        save_path = f"results_6dof_complete/complete_6dof_{now_str}.png"
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Complete 6DOF 결과 저장: {save_path}")
        plt.show()

        return save_path

def main():
    """메인 함수"""
    print("Complete 6DOF 미사일 궤적 시뮬레이션")
    print("=" * 70)
    print("개선 사항:")
    print("  ✅ DCM 좌표계 변환")
    print("  ✅ 측면 받음각(β) 계산")
    print("  ✅ 추력 벡터 동체 좌표계 정렬")
    print("  ✅ 공력 모멘트 팔 적용")
    print("  ✅ 비선형 공기역학 계수")
    print("  ✅ 연료 소모에 따른 질량 분포 변화")
    print("=" * 70)

    # 시뮬레이션 객체 생성
    simulation = Complete6DOFSimulation(missile_type="SCUD-B", apply_errors=False)

    # 초기화
    simulation.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=1500)

    # 시뮬레이션 실행
    results = simulation.run_simulation()

    if results is not None:
        # 결과 시각화
        simulation.plot_results_complete()

        # 결과 요약
        final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
        max_altitude = np.max(results['h']) / 1000
        final_velocity = results['velocity'][-1]

        print("\n" + "=" * 70)
        print("Complete 6DOF 시뮬레이션 결과 요약")
        print("=" * 70)
        print(f"최종 사거리: {final_range:.2f} km")
        print(f"최대 고도: {max_altitude:.2f} km")
        print(f"최종 속도: {final_velocity:.2f} m/s")
        print(f"비행 시간: {results['time'][-1]:.2f} s")
        print(f"최종 롤각: {np.rad2deg(results['phi'][-1]):.2f}°")
        print(f"최종 피치각: {np.rad2deg(results['theta'][-1]):.2f}°")
        print(f"최종 요각: {np.rad2deg(results['psi_euler'][-1]):.2f}°")
        print(f"최종 받음각: {np.rad2deg(results['alpha'][-1]):.2f}°")
        print(f"최종 측면 받음각: {np.rad2deg(results['beta'][-1]):.2f}°")
        print(f"최종 무게중심: {results['cg'][-1]:.2f} m")
        print("=" * 70)

    print("\nComplete 6DOF 미사일 궤적 시뮬레이션이 완료되었습니다.")

if __name__ == "__main__":
    main()
