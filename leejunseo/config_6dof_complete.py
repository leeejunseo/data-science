#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete 6DOF Configuration - Full Physics Implementation
진정한 6DOF 물리 시뮬레이션을 위한 설정 파일
"""
import numpy as np

# 기본 단위 환산
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# 물리 상수
G = 9.80665  # 표준 중력가속도 (m/s²)
R_EARTH = 6371000  # 지구 평균 반지름 (m)
R = R_EARTH
GM_EARTH = 3.986004418e14  # 지구 중력 매개변수 (m³/s²)

# 대기 상수
AIR_GAS_CONSTANT = 287.0531  # 건조 공기 기체상수 (J/(kg·K))
STD_TEMPERATURE_SEA_LEVEL = 288.15  # 해면 표준온도 (K)
STD_PRESSURE_SEA_LEVEL = 101325.0  # 해면 표준기압 (Pa)
STD_DENSITY_SEA_LEVEL = 1.225  # 해면 표준밀도 (kg/m³)
EXOSPHERE_HEIGHT = 600000  # 외기권 시작 고도 (m)

class CoordinateTransforms:
    """좌표계 변환 유틸리티"""

    @staticmethod
    def body_to_earth_dcm(phi, theta, psi):
        """
        동체 좌표계 → 지구 좌표계 방향 코사인 행렬 (DCM)

        Args:
            phi: 롤각 (rad)
            theta: 피치각 (rad)
            psi: 요각 (rad)

        Returns:
            3x3 DCM matrix
        """
        cp, sp = np.cos(phi), np.sin(phi)
        ct, st = np.cos(theta), np.sin(theta)
        cs, ss = np.cos(psi), np.sin(psi)

        # ZYX Euler angles (Yaw-Pitch-Roll)
        dcm = np.array([
            [ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss],
            [ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs],
            [-st,   sp*ct,            cp*ct]
        ])
        return dcm

    @staticmethod
    def earth_to_body_dcm(phi, theta, psi):
        """지구 좌표계 → 동체 좌표계 DCM (전치 행렬)"""
        return CoordinateTransforms.body_to_earth_dcm(phi, theta, psi).T

    @staticmethod
    def wind_to_body_dcm(alpha, beta):
        """
        바람 좌표계 → 동체 좌표계 DCM

        Args:
            alpha: 받음각 (rad)
            beta: 측면 받음각 (rad)
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)

        dcm = np.array([
            [ca*cb,  -sb,  sa*cb],
            [ca*sb,   cb,  sa*sb],
            [-sa,     0,   ca]
        ])
        return dcm

    @staticmethod
    def velocity_to_euler_rates(phi, theta, psi, p, q, r):
        """
        각속도 → 오일러각 변화율 (짐벌락 방지)

        Returns:
            dphi_dt, dtheta_dt, dpsi_dt
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        tt = np.tan(theta) if abs(ct) > 0.01 else 0.0

        sp = np.sin(phi)
        cp = np.cos(phi)

        # 특이점 방지
        if abs(ct) < 0.01:
            # theta ≈ ±90° 일 때 대체 공식 사용
            dphi_dt = p + (q*sp + r*cp) * 1000  # 근사값
            dtheta_dt = q*cp - r*sp
            dpsi_dt = 0.0
        else:
            dphi_dt = p + (q*sp + r*cp) * tt
            dtheta_dt = q*cp - r*sp
            dpsi_dt = (q*sp + r*cp) / ct

        # 각도 변화율 제한 (물리적 한계)
        max_rate = 10.0  # rad/s
        dphi_dt = np.clip(dphi_dt, -max_rate, max_rate)
        dtheta_dt = np.clip(dtheta_dt, -max_rate, max_rate)
        dpsi_dt = np.clip(dpsi_dt, -max_rate, max_rate)

        return dphi_dt, dtheta_dt, dpsi_dt

    @staticmethod
    def calculate_angles_of_attack(v_body):
        """
        동체 좌표계 속도로부터 받음각과 측면 받음각 계산

        Args:
            v_body: 동체 좌표계 속도 벡터 [vx, vy, vz]

        Returns:
            alpha (받음각), beta (측면 받음각)
        """
        vx, vy, vz = v_body
        V = np.linalg.norm(v_body)

        if V < 0.1:  # 속도가 너무 작으면
            return 0.0, 0.0

        # 받음각: 속도 벡터와 x축 사이의 각도 (xz 평면)
        alpha = np.arctan2(vz, vx)

        # 측면 받음각: 측면 속도 성분
        beta = np.arcsin(np.clip(vy / V, -1.0, 1.0))

        return alpha, beta

class PhysicsUtils:
    """물리 유틸리티"""

    @staticmethod
    def gravity_at_altitude(h):
        """고도에 따른 중력 계산"""
        return G * (R_EARTH / (R_EARTH + h))**2

    @staticmethod
    def atmospheric_density(h):
        """표준 대기 모델 기반 밀도 계산"""
        if h < 0:
            return STD_DENSITY_SEA_LEVEL

        # 11km 이하 (대류권)
        if h <= 11000:
            T = STD_TEMPERATURE_SEA_LEVEL - 0.0065 * h
            P = STD_PRESSURE_SEA_LEVEL * (T / STD_TEMPERATURE_SEA_LEVEL)**5.2561
        # 11-20km (성층권 하부)
        elif h <= 20000:
            T = 216.65
            P = 22632.1 * np.exp(-0.00015768 * (h - 11000))
        # 20-32km
        elif h <= 32000:
            T = 216.65 + 0.001 * (h - 20000)
            P = 5474.89 * (T / 216.65)**(-34.163)
        # 32km 이상
        else:
            T = 228.65 + 0.0028 * (h - 32000)
            P = 868.02 * (T / 228.65)**(-34.163)

        return P / (AIR_GAS_CONSTANT * T)

    @staticmethod
    def sound_speed(h):
        """고도에 따른 음속 계산"""
        if h <= 11000:
            T = STD_TEMPERATURE_SEA_LEVEL - 0.0065 * h
        elif h <= 20000:
            T = 216.65
        elif h <= 32000:
            T = 216.65 + 0.001 * (h - 20000)
        else:
            T = 228.65 + 0.0028 * (h - 32000)

        gamma = 1.4  # 비열비
        R = AIR_GAS_CONSTANT
        return np.sqrt(gamma * R * T)

    @staticmethod
    def mach_number(velocity, altitude):
        """마하수 계산"""
        sound_speed = PhysicsUtils.sound_speed(altitude)
        return velocity / sound_speed if sound_speed > 0 else 0

    @staticmethod
    def nonlinear_cl_alpha(alpha, mach):
        """
        비선형 양력 기울기

        Args:
            alpha: 받음각 (rad)
            mach: 마하수
        """
        alpha_deg = np.rad2deg(alpha)

        # 천음속/초음속 효과
        if mach < 0.8:
            mach_factor = 1.0
        elif mach < 1.2:
            mach_factor = 0.8 - 0.5 * (mach - 0.8)  # 천음속 감소
        else:
            mach_factor = 0.6 + 0.2 * (mach - 1.2) / 2.0  # 초음속 회복

        # 기본 선형 영역
        cl_linear = 0.05 * alpha_deg  # per degree

        # 실속 효과
        if abs(alpha_deg) < 20:
            cl = cl_linear
        elif abs(alpha_deg) < 30:
            # 실속 시작
            stall_factor = 1.0 - 0.5 * (abs(alpha_deg) - 20) / 10
            cl = cl_linear * stall_factor
        else:
            # 완전 실속
            cl = np.sign(alpha) * 0.5

        return cl * mach_factor

    @staticmethod
    def drag_coefficient_nonlinear(mach, alpha, beta):
        """
        비선형 항력 계수

        Args:
            mach: 마하수
            alpha: 받음각 (rad)
            beta: 측면 받음각 (rad)
        """
        # 기본 항력 (마하수 의존)
        if mach < 0.8:
            cd_base = 0.3
        elif mach < 1.0:
            # 천음속 항력 증가
            cd_base = 0.3 + 2.5 * (mach - 0.8)
        elif mach < 1.2:
            # 음속 돌파 후 감소
            cd_base = 0.8 - 0.3 * (mach - 1.0) / 0.2
        else:
            # 초음속
            cd_base = 0.5 - 0.2 * np.exp(-(mach - 1.5))

        # 받음각에 의한 항력 증가 (유도 항력)
        cd_alpha = 0.5 * (alpha**2 + beta**2)

        return cd_base + cd_alpha

class StateVector6DOF:
    """6DOF 상태 벡터 관리 클래스"""

    # 상태 인덱스 정의 (14차원)
    # 병진 운동 (6개)
    VELOCITY = 0      # 속도 (m/s)
    GAMMA = 1         # 비행경로각 (rad)
    PSI = 2           # 방위각 (rad)
    X = 3             # X 위치 (m)
    Y = 4             # Y 위치 (m)
    H = 5             # 고도 (m)

    # 회전 운동 (6개)
    PHI = 6           # 롤각 (rad)
    THETA = 7         # 피치각 (오일러각) (rad)
    PSI_EULER = 8     # 요각 (오일러각) (rad)
    P = 9             # 롤 각속도 (rad/s)
    Q = 10            # 피치 각속도 (rad/s)
    R_RATE = 11       # 요 각속도 (rad/s)

    # 질량 (2개)
    MASS = 12         # 질량 (kg)
    FUEL_CONSUMED = 13 # 연료소모 (kg)

    STATE_DIM = 14

    STATE_NAMES = [
        'velocity', 'gamma_rad', 'psi_rad', 'x', 'y', 'h',
        'phi_rad', 'theta_rad', 'psi_euler_rad', 'p_rate', 'q_rate', 'r_rate',
        'mass', 'fuel_consumed'
    ]

    STATE_UNITS = [
        'm/s', 'rad', 'rad', 'm', 'm', 'm',
        'rad', 'rad', 'rad', 'rad/s', 'rad/s', 'rad/s',
        'kg', 'kg'
    ]

    @staticmethod
    def create_initial_state(missile_info, launch_angle_deg, launch_azimuth_deg=90):
        """초기 상태 벡터 생성 (6DOF)"""
        initial_state = np.zeros(StateVector6DOF.STATE_DIM)

        # 병진 운동 초기값
        initial_state[StateVector6DOF.VELOCITY] = 0.1  # 작은 초기 속도
        initial_state[StateVector6DOF.GAMMA] = np.deg2rad(launch_angle_deg)
        initial_state[StateVector6DOF.PSI] = np.deg2rad(launch_azimuth_deg)
        initial_state[StateVector6DOF.X] = 0.0
        initial_state[StateVector6DOF.Y] = 0.0
        initial_state[StateVector6DOF.H] = 0.0

        # 회전 운동 초기값 (발사대에서 정렬된 상태)
        initial_state[StateVector6DOF.PHI] = 0.0  # 롤 0
        initial_state[StateVector6DOF.THETA] = np.deg2rad(launch_angle_deg)  # 피치 = 발사각
        initial_state[StateVector6DOF.PSI_EULER] = np.deg2rad(launch_azimuth_deg)  # 요 = 방위각
        initial_state[StateVector6DOF.P] = 0.0  # 롤 각속도 0
        initial_state[StateVector6DOF.Q] = 0.0  # 피치 각속도 0
        initial_state[StateVector6DOF.R_RATE] = 0.0  # 요 각속도 0

        # 질량 초기값
        initial_state[StateVector6DOF.MASS] = missile_info["launch_weight"]
        initial_state[StateVector6DOF.FUEL_CONSUMED] = 0.0

        return initial_state

# 완전한 6DOF 미사일 데이터
COMPLETE_MISSILE_TYPES_6DOF = {
    "SCUD-B": {
        # 기본 정보
        "name": "SCUD-B",
        "diameter": 0.88,
        "length": 10.94,
        "nozzle_diameter": 0.6,
        "launch_weight": 5860,
        "payload": 985,
        "propellant_mass": 4875,
        "structural_mass": 5860 - 4875 - 985,  # 구조 질량
        "range_km": 300,

        # 추진 정보
        "propellant_type": "LIQUID",
        "isp_sea": 230,
        "isp_vac": 258,
        "burn_time": 65,

        # 비행 프로그램
        "vertical_time": 10,
        "pitch_time": 15,
        "pitch_angle_deg": 20,

        # 공기역학
        "reference_area": np.pi * (0.88/2)**2,
        "cd_base": 0.25,

        # 관성 모멘트 (kg·m²) - 초기값
        "inertia_xx_empty": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),
        "inertia_yy_empty": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),
        "inertia_zz_empty": 5860 * ((0.88/2)**2 / 2),

        # 공력 모멘트 계수
        "cl_alpha": 0.05,      # per rad
        "cm_alpha": -0.015,    # per rad (정적 안정)
        "cn_beta": -0.01,      # per rad
        "cl_p": -0.5,          # 롤 댐핑
        "cm_q": -0.8,          # 피치 댐핑
        "cn_r": -0.8,          # 요 댐핑

        # 무게중심 및 압력중심 (미사일 앞쪽에서 거리, m)
        "cg_location_full": 5.0,    # 연료 만땅일 때
        "cg_location_empty": 5.8,   # 연료 소진 시
        "cp_location": 6.5,         # 압력중심 (고정)

        # 연료 탱크 위치
        "fuel_tank_location": 4.5,  # 연료 무게중심
    },

    "NODONG": {
        "name": "노동 1호",
        "diameter": 1.36,
        "length": 16.4,
        "nozzle_diameter": 0.8,
        "launch_weight": 16500,
        "payload": 1200,
        "propellant_mass": 13500,
        "structural_mass": 16500 - 13500 - 1200,
        "range_km": 1500,

        "propellant_type": "UDMH/RFNA",
        "isp_sea": 255,
        "isp_vac": 280,
        "burn_time": 70,

        "vertical_time": 10,
        "pitch_time": 20,
        "pitch_angle_deg": 15,

        "reference_area": np.pi * (1.36/2)**2,
        "cd_base": 0.23,

        "inertia_xx_empty": 16500 * (16.4**2 / 12 + (1.36/2)**2 / 4),
        "inertia_yy_empty": 16500 * (16.4**2 / 12 + (1.36/2)**2 / 4),
        "inertia_zz_empty": 16500 * ((1.36/2)**2 / 2),

        "cl_alpha": 0.055,
        "cm_alpha": -0.018,
        "cn_beta": -0.012,
        "cl_p": -0.6,
        "cm_q": -0.9,
        "cn_r": -0.9,

        "cg_location_full": 7.5,
        "cg_location_empty": 8.5,
        "cp_location": 9.5,
        "fuel_tank_location": 7.0,
    },

    "KN-23": {
        "name": "KN-23",
        "diameter": 0.95,
        "length": 7.5,
        "nozzle_diameter": 0.5,
        "launch_weight": 3415,
        "payload": 500,
        "propellant_mass": 2500,
        "structural_mass": 3415 - 2500 - 500,
        "range_km": 690,

        "propellant_type": "SOLID",
        "isp_sea": 260,
        "isp_vac": 265,
        "burn_time": 40,

        "vertical_time": 6,
        "pitch_time": 10,
        "pitch_angle_deg": 25,

        "reference_area": np.pi * (0.95/2)**2,
        "cd_base": 0.28,

        "inertia_xx_empty": 3415 * (7.5**2 / 12 + (0.95/2)**2 / 4),
        "inertia_yy_empty": 3415 * (7.5**2 / 12 + (0.95/2)**2 / 4),
        "inertia_zz_empty": 3415 * ((0.95/2)**2 / 2),

        "cl_alpha": 0.06,
        "cm_alpha": -0.02,
        "cn_beta": -0.015,
        "cl_p": -0.7,
        "cm_q": -1.0,
        "cn_r": -1.0,

        "cg_location_full": 3.5,
        "cg_location_empty": 4.0,
        "cp_location": 4.8,
        "fuel_tank_location": 3.2,
    }
}

# 역호환성
MISSILE_TYPES = COMPLETE_MISSILE_TYPES_6DOF

# 시뮬레이션 설정
INTERVAL = 0.1
SIM_TIME = 2000
SIMULATION_END_TIME = SIM_TIME

# 공력 계수
CL_VERTICAL = 0.0
CL_PITCH = 0.5
CL_CONSTANT = 0.3
CL_TERMINAL = 0.1
K = 0.05

# 기타 상수
STD_GRAVITY = G
AIR_MOLAR_MASS = 0.0289644
UNIVERSAL_GAS_CONSTANT = 8.314462

def get_cd_table_for_missile(missile_name):
    """미사일별 항력계수 테이블 반환"""
    missile_cd_tables = {
        'SCUD-B': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.32, 0.36, 0.47, 0.82, 0.62, 0.42, 0.37, 0.32, 0.30, 0.27])
        },
        'NODONG': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.28, 0.33, 0.43, 0.78, 0.58, 0.38, 0.33, 0.28, 0.26, 0.23])
        },
        'KN-23': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.35, 0.38, 0.48, 0.85, 0.65, 0.45, 0.40, 0.35, 0.33, 0.30])
        }
    }

    default_cd_table = {
        'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
        'cd': np.array([0.3, 0.35, 0.45, 0.8, 0.6, 0.4, 0.35, 0.3, 0.28, 0.25])
    }

    return missile_cd_tables.get(missile_name, default_cd_table)
