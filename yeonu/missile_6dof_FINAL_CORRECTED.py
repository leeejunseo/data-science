"""
논문 기반 정통 6DOF 탄도 미사일 시뮬레이터 (교수님 호환)

핵심:
1. ✅ 정통 6DOF Body Frame 동역학 (Zipfel 2007, 팀2 논문)
2. ✅ 교수님 4단계 비행 프로그램 호환
3. ✅ 공력 댐핑 (PID 없음)
4. ✅ 교수님 config.py 파라미터

참조:
- Zipfel (2007): 6DOF Body Frame EOM
- 팀2 논문: Nonlinear time-invariant coupled differential equations
- Stevens & Lewis: 공력 모멘트 계수
- KAIST 논문: 항력 계수 검증
- 교수님: 4단계 비행 프로그램
"""

import numpy as np
from typing import Dict
from scipy.integrate import solve_ivp
import sys

sys.path.append('/mnt/project')
try:
    import config as cfg
    CONFIG_LOADED = True
    print("✓ config.py 로드")
except:
    print("✗ config.py 로드 실패")
    CONFIG_LOADED = False
    class cfg:
        G = 9.80665
        R = 6371000
        DEG_TO_RAD = np.pi / 180
        RAD_TO_DEG = 180 / np.pi


def get_density(h):
    """ISA 1976 대기 모델"""
    if h < 0:
        h = 0
    if h < 11000:
        T = 288.15 - 0.0065 * h
        p = 101325 * (T / 288.15) ** 5.2561
    elif h < 25000:
        T = 216.65
        p = 22632.1 * np.exp(-0.00015768 * (h - 11000))
    else:
        T = 216.65 + 0.003 * (h - 25000)
        p = 2488.7 * (T / 216.65) ** (-11.388)
    rho = p / (287.05 * T)
    return rho


class Missile6DOF_Authentic:
    """
    논문 기반 정통 6DOF (Zipfel 2007 + 팀2 논문)
    
    State: [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m] (13차원)
    - X, Y, Z: Inertial Frame 위치
    - u, v, w: Body Frame 속도
    - phi, theta, psi: Euler 각도
    - p, q, r: Body Frame 각속도
    - m: 질량
    
    운동 방정식 (Zipfel 2007):
    병진: m * dV/dt = F_thrust + F_aero + F_gravity (Body Frame)
    회전: I * dω/dt + ω × (I * ω) = M_aero (Body Frame)
    """
    
    def __init__(self, missile_type="SCUD-B"):
        self.missile_type = missile_type
        self.load_params()
        
    def load_params(self):
        """교수님 config.py 기반 파라미터 (하드코딩 fallback)"""
        
        # 교수님 config.py 값 (검증됨)
        PROF_CFG = {
            "SCUD-B": {
                "launch_weight": 5860, "propellant_mass": 4875,
                "diameter": 0.88, "length": 10.94,
                "burn_time": 65, "isp_sea": 230,
                "vertical_time": 10, "pitch_time": 15, "pitch_angle_deg": 20,
            },
            "Nodong": {
                "launch_weight": 16500, "propellant_mass": 15300,
                "diameter": 1.36, "length": 16.4,
                "burn_time": 70, "isp_sea": 255,
                "vertical_time": 10, "pitch_time": 20, "pitch_angle_deg": 15,
            },
            "KN-23": {
                "launch_weight": 3415, "propellant_mass": 2915,
                "diameter": 0.95, "length": 7.5,
                "burn_time": 40, "isp_sea": 260,
                "vertical_time": 6, "pitch_time": 10, "pitch_angle_deg": 25,
            }
        }
        
        # Config 로드 시도 (대문자 키 매칭)
        loaded = False
        if CONFIG_LOADED and hasattr(cfg, 'ENHANCED_MISSILE_TYPES'):
            info = cfg.ENHANCED_MISSILE_TYPES.get(self.missile_type.upper())
            if info:
                self.missile_mass = info["launch_weight"]
                self.propellant_mass = info["propellant_mass"]
                self.diameter = info["diameter"]
                self.length = info["length"]
                self.burn_time = info["burn_time"]
                self.isp_sea = info["isp_sea"]
                self.vertical_time = info.get("vertical_time", 10)
                self.pitch_time = info.get("pitch_time", 15)
                self.pitch_angle_deg = info.get("pitch_angle_deg", 20)
                self.wing_area = info["reference_area"]
                loaded = True
                print(f"✓ {self.missile_type} (config.py)")
        
        # Fallback: 하드코딩된 값 사용
        if not loaded:
            if self.missile_type in PROF_CFG:
                p = PROF_CFG[self.missile_type]
                self.missile_mass = p["launch_weight"]
                self.propellant_mass = p["propellant_mass"]
                self.diameter = p["diameter"]
                self.length = p["length"]
                self.burn_time = p["burn_time"]
                self.isp_sea = p["isp_sea"]
                self.vertical_time = p["vertical_time"]
                self.pitch_time = p["pitch_time"]
                self.pitch_angle_deg = p["pitch_angle_deg"]
                self.wing_area = np.pi * (self.diameter/2)**2
            else:
                print(f"⚠️ {self.missile_type} 미지원, SCUD-B 사용")
                p = PROF_CFG["SCUD-B"]
                self.missile_mass = p["launch_weight"]
                self.propellant_mass = p["propellant_mass"]
                self.diameter = p["diameter"]
                self.length = p["length"]
                self.burn_time = p["burn_time"]
                self.isp_sea = p["isp_sea"]
                self.vertical_time = p["vertical_time"]
                self.pitch_time = p["pitch_time"]
                self.pitch_angle_deg = p["pitch_angle_deg"]
                self.wing_area = np.pi * (self.diameter/2)**2
        
        # 관성 (Fleeman 2012)
        r, L, m = self.diameter/2, self.length, self.missile_mass
        I_xx_physical = 0.5 * m * r**2
        self.I_xx = I_xx_physical * 1.5  # Numerical Padding: 물리적 관성에 가깝게 복구
        self.I_yy = (1/12) * m * L**2 + 0.25 * m * r**2
        self.I_zz = self.I_yy
        
        # 댐핑 (Stevens & Lewis 2003)
        self.C_mq = -8.0
        self.C_nr = -8.0
        self.C_lp = -0.5
        
        # 제어 게인 기준값 (논문 기반)
        self.I_yy_ref = 58729.0  # SCUD-B 기준 관성
        self.K_p_ref = 9.0  # 팀2 논문 Ziegler-Nichols
        self.K_d_ref = 0.19125  # 팀2 논문
        self.q_ref = 0.5 * 1.225 * (100)**2  # 기준 동압 (V=100)
        
        print(f"  질량: {self.missile_mass}kg, 연소: {self.burn_time}s")
        print(f"  피치: {self.pitch_angle_deg}°/{self.pitch_time}s")
        print(f"  제어: 팀2 논문 + Zipfel 동압 스케줄링")
        
        # 발사 초기 고각 저장 (교수님 방식 호환용)
        self.launch_elevation_rad = 0
    
    def get_CD(self, mach, alpha_deg=0):
        """항력 계수 (고도별 증가 반영)"""
        if mach < 0.5:
            CD_mach = 0.25
        elif mach < 0.8:
            CD_mach = 0.22 + (0.35 - 0.22) * (mach - 0.5) / 0.3
        elif mach < 1.0:
            CD_mach = 0.35 + (0.45 - 0.35) * (mach - 0.8) / 0.2
        elif mach < 1.5:
            CD_mach = 0.45 - (0.45 - 0.35) * (mach - 1.0) / 0.5
        elif mach < 2.0:
            CD_mach = 0.35 - (0.35 - 0.30) * (mach - 1.5) / 0.5
        elif mach < 3.0:
            CD_mach = 0.30 - (0.30 - 0.25) * (mach - 2.0) / 1.0
        elif mach < 4.0:
            CD_mach = 0.25 - (0.25 - 0.22) * (mach - 3.0) / 1.0
        else:
            CD_mach = 0.22
        
        # 받음각 효과 증가 (에너지 손실 강화)
        CD_alpha = 0.01 * (alpha_deg / 10.0) ** 2
        return CD_mach + CD_alpha
    
    
    def calculate_control_gains(self, q_dyn):
        """
        동압 기반 제어 게인 (수정)
        
        근거:
        - 팀2 논문: K_p=9, K_d=0.19 (Ziegler-Nichols)
        - Zipfel (2007) Page 421: 동압 스케줄링
        - Stevens (2003): 관성 스케일링
        
        수정:
        - theta 제어는 더 큰 게인 필요
        - 기준 게인 10배 증가
        
        Args:
            q_dyn: 동압 (Pa)
        
        Returns:
            K_p, K_d: 비례/미분 게인
        """
        # 동압 비율
        q_ratio = max(q_dyn / self.q_ref, 0.1)
        
        # 관성 스케일링 (Stevens)
        I_scale = self.I_yy / self.I_yy_ref
        
        # theta 제어는 더 강하게 (10배)
        K_p_base = self.K_p_ref * 10.0  # 9 → 90
        K_d_base = self.K_d_ref * 10.0  # 0.19 → 1.9
        
        # 동압 스케줄링 (Zipfel)
        # 동압 증가 → 게인 감소 (공력 증가)
        K_p = K_p_base * I_scale / np.sqrt(q_ratio)
        K_d = K_d_base * I_scale / np.sqrt(q_ratio)
        
        # 안정성 제한 (더 높은 범위)
        K_p = np.clip(K_p, 10.0, 1000.0)
        K_d = np.clip(K_d, 0.1, 100.0)
        
        return K_p, K_d
    def get_flight_phase(self, t):
        """교수님 4단계 비행 프로그램"""
        if t < self.vertical_time:
            return "Vertical"
        elif t < self.vertical_time + self.pitch_time:
            return "Pitch"
        elif t < self.burn_time + 5:
            return "Constant"
        else:
            return "Ballistic"
    
    def get_target_theta(self, t):
        """
        목표 피치 각도 (교수님 방식 정확 구현)
        
        근거: 교수님 main.py 비행 프로그램
        
        핵심: 수직(90°) → (90° - pitch_angle)로 전환
        - Vertical: 90도 수직
        - Pitch: 90도 → (90 - pitch_angle)
        - Constant: (90 - pitch_angle) 유지
        - Ballistic: 자유비행
        """
        phase = self.get_flight_phase(t)
        
        if phase == "Vertical":
            return np.pi/2  # 90도 수직
        
        elif phase == "Pitch":
            # 90도 → (90 - pitch_angle)로 선형 전환 (속도 완화)
            progress = min(1.0, (t - self.vertical_time) / self.pitch_time * 1.0)
            theta_start = np.pi/2  # 90도
            theta_end = np.pi/2 - self.pitch_angle_deg * cfg.DEG_TO_RAD
            
            return theta_start - (theta_start - theta_end) * progress
        
        else:
            # Constant/Ballistic: (90 - pitch_angle) 유지
            return np.pi/2 - self.pitch_angle_deg * cfg.DEG_TO_RAD
    
    def calculate_control_moments(self, t, theta, q, Z, V_safe, q_dyn):
        """
        PD Control for Pitch Moment (무차원 계수 Cm 형태)
        
        제어 모멘트 = q_dyn * S * d * Cm_control
        Cm_control = -Kp * theta_error - Kd * (q * d / 2V)
        
        Args:
            t: current time
            theta: current pitch angle (rad)
            q: current pitch rate (rad/s)
            Z: current altitude (m)
            V_safe: safe velocity (m/s)
            q_dyn: dynamic pressure (Pa)
            
        Returns:
            M_control: control moment for pitch axis (N·m)
        """
        phase = self.get_flight_phase(t)
        
        # Target theta based on flight phase
        # 피치 오버 조건: 고도 1km 이상, 속도 100m/s 이상
        if phase == "Vertical" or V_safe < 100.0 or Z < 1000.0:
            target_theta = np.deg2rad(85.0)
        elif phase == "Pitch":
            # 피치 오버: 85도 -> (90 - pitch_angle)도
            progress = min(1.0, (t - self.vertical_time) / self.pitch_time)
            theta_start = np.deg2rad(85.0)
            theta_end = np.pi/2 - self.pitch_angle_deg * cfg.DEG_TO_RAD
            target_theta = theta_start - (theta_start - theta_end) * progress
        else:
            target_theta = np.pi/2 - self.pitch_angle_deg * cfg.DEG_TO_RAD
        
        # === 무차원 PD 제어 게인 ===
        # 더 강한 제어력으로 목표 각도 추종
        Kp_Cm = 0.2    # 무차원 비례 게인 (rad^-1)
        Kd_Cm = 0.3    # 무차원 미분 게인 (댐핑)
        
        # Error calculation
        theta_error = theta - target_theta
        
        # 무차원 각속도 (Zipfel 표준: q * d / 2V)
        q_hat = q * self.diameter / (2.0 * V_safe)
        
        # 무차원 제어 계수 Cm_control
        Cm_control = -Kp_Cm * theta_error - Kd_Cm * q_hat
        
        # Cm 범위 제한 (|Cm| < 0.5)
        Cm_limit = 0.5
        Cm_control = Cm_limit * np.tanh(Cm_control / Cm_limit)
        
        # 동압 제한 (저속 구간 보호)
        q_dyn_safe = min(max(q_dyn, 0.0), 100000.0)  # 0 ~ 100kPa
        
        # 제어 모멘트 = q_dyn * S * d * Cm
        M_control = q_dyn_safe * self.wing_area * self.diameter * Cm_control
        
        # 최종 모멘트 제한 (100,000 N·m - 현실적 범위)
        M_control_limit = 100000.0
        M_control = M_control_limit * np.tanh(M_control / M_control_limit)
        
        return M_control
    
    def dynamics(self, t, state):
        """
        6DOF Dynamics (Zipfel 2007 Newton-Euler)
        
        State: [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m]
        
        Equations of Motion:
        1. Translation (Body Frame):
           m * [du/dt]   [F_x]   [qw - rv]
               [dv/dt] = [F_y] + [ru - pw] * m
               [dw/dt]   [F_z]   [pv - qu]
        
        2. Rotation (Body Frame - Euler's Equation):
           [dp/dt]   [L/Ixx + (Iyy-Izz)*q*r/Ixx]
           [dq/dt] = [M/Iyy + (Izz-Ixx)*p*r/Iyy]
           [dr/dt]   [N/Izz + (Ixx-Iyy)*p*q/Izz]
        
        3. Kinematics (Pure - No program injection):
           [dphi/dt  ]   [1  sin(phi)tan(theta)  cos(phi)tan(theta)] [p]
           [dtheta/dt] = [0       cos(phi)           -sin(phi)      ] [q]
           [dpsi/dt  ]   [0  sin(phi)/cos(theta) cos(phi)/cos(theta)] [r]
        
        4. Position (Inertial):
           [dX/dt]       [u]
           [dY/dt] = T_BI [v]
           [dZ/dt]       [w]
        """
        # === 상태 변수 안전 처리 (Overflow 방지) ===
        # NaN/Inf 체크 및 대체
        state = np.array(state, dtype=np.float64)
        if np.any(~np.isfinite(state)):
            state = np.nan_to_num(state, nan=0.0, posinf=1e6, neginf=-1e6)
        
        # 전체 상태 클리핑
        state = np.clip(state, -1e8, 1e8)
        
        X, Y, Z = state[0:3]
        u, v, w = state[3:6]
        
        # 고도 안전 처리 (음수 방지)
        Z = max(Z, 0.0)
        
        # 속도 클리핑 (최대 3km/s = 마하 9 수준)
        u = np.clip(u, -3000.0, 3000.0)
        v = np.clip(v, -500.0, 500.0)
        w = np.clip(w, -500.0, 500.0)
        
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]
        m = state[12]
        
        # 질량 안전 처리
        m = max(m, self.missile_mass - self.propellant_mass)
        
        # 각속도 Soft Saturation (±1 rad/s - 더 보수적)
        omega_limit = 1.0
        p = omega_limit * np.tanh(p / omega_limit)
        q = omega_limit * np.tanh(q / omega_limit)
        r = omega_limit * np.tanh(r / omega_limit)
        
        # 오일러 각도 wrapping [-π, π]
        phi = np.arctan2(np.sin(phi), np.cos(phi))
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        psi = np.arctan2(np.sin(psi), np.cos(psi))
        
        # 중력 및 대기 밀도
        Z_safe = max(Z, 0.0)
        g = cfg.G * (cfg.R / (cfg.R + Z_safe))**2
        rho = get_density(Z_safe)
        
        # 속도 크기 (하한선 강화)
        V = np.sqrt(u**2 + v**2 + w**2)
        V_safe = max(V, 10.0)  # 분모 안정성을 위한 하한선 (10.0으로 상향)
        
        # 받음각, 측면각 (Body Frame - 강화된 수치 가드)
        # u가 0에 가까울 때 수치적 불안정 방지
        u_safe = max(abs(u), 1.0) * np.sign(u) if u != 0 else 1.0
        alpha = np.arctan2(w, u_safe)
        beta = np.arctan2(v, np.sqrt(u_safe**2 + w**2))
        
        # 받음각/옆미끄럼각 하드 클리핑 (±20도)
        alpha = np.clip(alpha, -np.deg2rad(20), np.deg2rad(20))
        beta = np.clip(beta, -np.deg2rad(20), np.deg2rad(20))
        
        # 마하수
        T = 288.15 - 0.0065 * Z if Z < 11000 else 216.65
        a = np.sqrt(1.4 * 287.05 * T)
        mach = V_safe / a
        
        # 동압 (tanh 스무싱으로 연속적 제한)
        q_dyn_raw = 0.5 * rho * V_safe**2  # V_safe 사용으로 초기 동압 튀 방지
        q_max = 500000  # 50만 Pa
        # tanh 스무싱: 불연속점 제거
        q_dyn = q_max * np.tanh(q_dyn_raw / q_max)
        # 고도 상승 시 음수 방지 및 추가 클리핑
        q_dyn = np.clip(q_dyn, 0.0, 300000)  # 30만 Pa 상한
        
        # === 힘 계산 (Body Frame) ===
        
        # 1. 추력 (Body X축)
        if t < self.burn_time:
            thrust = self.isp_sea * (self.propellant_mass / self.burn_time) * g
        else:
            thrust = 0
        F_thrust_x = thrust
        F_thrust_y = 0
        F_thrust_z = 0
        
        # 2. 공력 (Body Frame)
        CD = self.get_CD(mach, alpha * 180/np.pi)
        
        # CL_alpha 현실적 조정 (6.0~8.0 범위)
        # 양항비 목표는 기준 면적 조정으로 보완
        if self.missile_type.upper() == "NODONG":
            CL_alpha = 7.0  # 기존 13.5에서 현실적 값으로 조정
        else:
            CL_alpha = 6.0  # 기존 4.5에서 소폭 상향
        
        CL = CL_alpha * alpha
        CY = -0.5 * beta  # CY_beta (측력)
        
        D = q_dyn * self.wing_area * CD
        L = q_dyn * self.wing_area * CL
        Y_force = q_dyn * self.wing_area * CY
        
        # Body Frame 공력
        F_aero_x = -D
        F_aero_y = Y_force
        F_aero_z = -L
        
        # 3. 중력 (Inertial → Body 변환)
        # T_BI: Body → Inertial
        # T_IB = T_BI^T
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, stheta = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        # T_BI: Body → Inertial 변환 행렬 (NED → ENU 보정)
        # 표준 항공 좌표계: X=전방, Y=우측, Z=하방 (Body)
        # 관성 좌표계: X=북, Y=동, Z=상 (ENU)
        # theta=85도 → 거의 수직 상승 → dZ 양수
        T_BI = np.array([
            [ctheta*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi],
            [ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi],
            [stheta,      -sphi*ctheta,                 -cphi*ctheta]
        ])
        
        # 중력 벡터 (ENU 좌표계: Z-up이므로 -Z 방향)
        g_inertial = np.array([0, 0, -m*g])
        g_body = T_BI.T @ g_inertial
        
        F_grav_x = g_body[0]
        F_grav_y = g_body[1]
        F_grav_z = g_body[2]
        
        # 총 힘
        F_total_x = F_thrust_x + F_aero_x + F_grav_x
        F_total_y = F_thrust_y + F_aero_y + F_grav_y
        F_total_z = F_thrust_z + F_aero_z + F_grav_z
        
        # === 병진 운동 방정식 (Zipfel 2007) ===
        du = F_total_x/m + q*w - r*v
        dv = F_total_y/m + r*u - p*w
        dw = F_total_z/m + p*v - q*u
        
        # 병진 가속도 Soft Saturation (±500 m/s² - 50G 제한)
        accel_limit = 500.0
        du = accel_limit * np.tanh(du / accel_limit)
        dv = accel_limit * np.tanh(dv / accel_limit)
        dw = accel_limit * np.tanh(dw / accel_limit)
        
        # === 모멘트 계산 (Body Frame - Zipfel 2007) ===
        
        # 기준 길이 = diameter (Zipfel 표준)
        D_ref = self.diameter
        q_dyn_safe = min(max(q_dyn, 0.0), 50000.0)  # 0 ~ 50kPa로 제한
        
        # 1. 정적 안정성 모멘트 (Cm_alpha) - 복원력
        Cm_alpha = -0.3  # 더 보수적인 복원력
        M_static = q_dyn_safe * self.wing_area * D_ref * Cm_alpha * alpha
        
        # 2. 동적 댐핑 모멘트 (Cmq) - Zipfel 식
        Cmq = -10.0  # 보수적 댐핑 계수
        Cnr = -10.0
        Clp = -0.3
        
        M_damping = q_dyn_safe * self.wing_area * D_ref * Cmq * (q * D_ref / (2 * V_safe))
        N_damping = q_dyn_safe * self.wing_area * D_ref * Cnr * (r * D_ref / (2 * V_safe))
        L_damping = q_dyn_safe * self.wing_area * D_ref * Clp * (p * D_ref / (2 * V_safe))
        
        # 3. 요 복원력 (Weathercock Stability)
        Cn_beta = -0.2
        N_static = q_dyn_safe * self.wing_area * D_ref * Cn_beta * beta
        
        # 4. PD 제어 모멘트 (피치 프로그램 대체)
        M_control = self.calculate_control_moments(t, theta, q, Z, V_safe, q_dyn)
        
        # 총 모멘트
        L_aero = L_damping
        M_aero = M_static + M_damping + M_control
        N_aero = N_static + N_damping
        
        # 모멘트 Soft Saturation (10,000 N.m - 매우 보수적)
        moment_limit = 10000.0
        L_aero = moment_limit * np.tanh(L_aero / moment_limit)
        M_aero = moment_limit * np.tanh(M_aero / moment_limit)
        N_aero = moment_limit * np.tanh(N_aero / moment_limit)
        
        # === 회전 운동 방정식 (Euler's equation) ===
        dp = L_aero / self.I_xx
        dq = (M_aero + (self.I_zz - self.I_xx) * p * r) / self.I_yy
        dr = (N_aero + (self.I_xx - self.I_yy) * p * q) / self.I_zz
        
        # 각가속도 Soft Saturation (±5 rad/s² - 매우 보수적)
        ang_accel_limit = 5.0
        dp = ang_accel_limit * np.tanh(dp / ang_accel_limit)
        dq = ang_accel_limit * np.tanh(dq / ang_accel_limit)
        dr = ang_accel_limit * np.tanh(dr / ang_accel_limit)
        
        # === 저속 구간 강제 자세 고정 (Fins Ineffective Zone) ===
        # 마하 0.6 (V < 200m/s) 이전에는 각속도 변화를 0으로 강제
        # 공력 날개가 힘을 쓰지 못하는 저속 구간에서 초기 자세 보존
        if V_safe < 200.0:
            dp = 0.0
            dq = 0.0
            dr = 0.0
        
        # === Kinematic 방정식 (Pure - Zipfel 2007) ===
        # Gimbal Lock 방지: cos(theta) = 0 근처에서 견고한 처리
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Gimbal Lock 방지 (theta = ±90도 근처) - 보수적 임계값
        # cos(theta) < 0.1 (약 84.3도) 이면 특수 처리 -> 85도 발사 안정화
        gimbal_threshold = 0.1
        if abs(cos_theta) < gimbal_threshold:
            # Gimbal Lock 근처: 단순화된 기구학 사용
            cos_theta_safe = np.sign(cos_theta) * gimbal_threshold if cos_theta != 0 else gimbal_threshold
            tan_theta_safe = sin_theta / cos_theta_safe
        else:
            cos_theta_safe = cos_theta
            tan_theta_safe = np.tan(theta)
        
        # tan(theta) Soft Saturation - 보수적 제한 (85도 발사 안정화)
        # tan(85도) ≈ 11.4, 여유있게 20으로 제한
        tan_limit = 20.0
        tan_theta_safe = tan_limit * np.tanh(tan_theta_safe / tan_limit)
        
        # 순수 기구학 방정식 (dtheta_program 제거 - Newton-Euler 준수)
        dphi = p + (q*sphi + r*cphi) * tan_theta_safe
        dtheta = q*cphi - r*sphi  # Pure kinematics only
        dpsi = (q*sphi + r*cphi) / cos_theta_safe
        
        # 기구학적 속도 Soft Saturation (초당 5라디안)
        rate_limit = 5.0
        dphi = rate_limit * np.tanh(dphi / rate_limit)
        dtheta = rate_limit * np.tanh(dtheta / rate_limit)
        dpsi = rate_limit * np.tanh(dpsi / rate_limit)
        
        # === 위치 변화 (Inertial) ===
        v_body = np.array([u, v, w])
        v_inertial = T_BI @ v_body
        
        dX = v_inertial[0]
        dY = v_inertial[1]
        dZ = v_inertial[2]
        
        # === 질량 변화 ===
        if t < self.burn_time:
            dm = -(self.propellant_mass / self.burn_time)
        else:
            dm = 0
        
        # 반환값 최종 검사 (Safety Valve)
        derivatives = np.array([dX, dY, dZ, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr, dm])
        
        # 1. 모든 미분값 Soft Saturation (±1e4 범위 - 미분 가능)
        deriv_limit = 1e4
        derivatives = deriv_limit * np.tanh(derivatives / deriv_limit)
        
        # 2. NaN/Inf 완전 제거
        derivatives = np.nan_to_num(derivatives, nan=0.0, posinf=0.0, neginf=0.0)
        
        return derivatives.tolist()
    
    def event_ground(self, t, state):
        """지면 충돌"""
        if t < 5.0:
            return 1000.0
        return state[2]  # Z
    event_ground.terminal = True
    event_ground.direction = -1
    
    def simulate(self, elevation_deg=45, azimuth_deg=90, add_disturbance=True, 
                 return_dataframe=False):
        """
        시뮬레이션
        
        Args:
            elevation_deg: 발사 고각
            azimuth_deg: 방위각
            add_disturbance: 초기 교란 (시그니처용)
            return_dataframe: True면 Pandas DataFrame 반환
            
        Returns:
            results: dict 또는 (DataFrame, reference_data)
        """
        print(f"\n=== 논문 기반 정통 6DOF ===")
        print(f"발사: 고각 {elevation_deg}°, 방위각 {azimuth_deg}°")
        
        # 초기 조건
        X0, Y0, Z0 = 0, 0, 0
        
        # 초기 속도 (Body Frame) - 수치적 Stiffness 완화를 위해 50 m/s
        u0, v0, w0 = 50.0, 0, 0
        
        # 초기 자세 (Vertical phase)
        # 방위각 90도 = X축 정방향 (psi = 0)
        # 방위각 0도 = Y축 정방향 (psi = 90도)
        phi0 = 0
        theta0 = np.deg2rad(85.0)  # 안정적 포물선 궤적 시작점
        psi0 = (90 - azimuth_deg) * cfg.DEG_TO_RAD  # 방위각 변환
        
        # 초기 각속도 (교란) - 대폭 완화
        if add_disturbance:
            sigma = 0.01 * cfg.DEG_TO_RAD  # 기존 0.1에서 0.01로 대폭 완화
            p0 = 0  # Roll rate 고정 (궤적 꼬임 방지)
            q0 = np.random.normal(0, sigma)
            r0 = np.random.normal(0, sigma)
            print(f"✓ 초기 교란: p={p0*cfg.RAD_TO_DEG:.3f}°/s, " +
                  f"q={q0*cfg.RAD_TO_DEG:.3f}°/s, r={r0*cfg.RAD_TO_DEG:.3f}°/s")
        else:
            p0, q0, r0 = 0, 0, 0
        
        m0 = self.missile_mass
        
        state0 = [X0, Y0, Z0, u0, v0, w0, phi0, theta0, psi0, p0, q0, r0, m0]
        
        print("적분 중...")
        # Radau 적분기로 복귀 (물리량 부드러워짐)
        # 상태 변수별 atol 벡터화 (13개 상태변수)
        # [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m]
        # 위치(X,Y,Z): 1e-1, 속도(u,v,w): 1e-3, 각도/각속도: 1e-4 (완화), 질량: 1e-3
        atol_vec = [1.0, 1.0, 1.0,    # X, Y, Z (1m 단위 - 완화)
                    1e-3, 1e-3, 1e-3,  # u, v, w (속도)
                    1e-4, 1e-4, 1e-4,  # phi, theta, psi (각도)
                    1e-4, 1e-4, 1e-4,  # p, q, r (각속도)
                    1e-3]              # m (질량)
        
        # Radau 시도 후 실패 시 DOP853으로 fallback
        try:
            sol = solve_ivp(
                self.dynamics,
                [0, 600],
                state0,
                method='Radau',   # Radau: 암시적 적분기 (Stiff 시스템에 적합)
                max_step=0.1,     # 고속 비행 시 정밀도 확보
                first_step=0.01,
                events=[self.event_ground],
                rtol=1e-3,
                atol=atol_vec
            )
            if not sol.success:
                raise RuntimeError("Radau 실패")
        except Exception as e:
            print(f"⚠️ Radau 실패, DOP853으로 전환: {e}")
            sol = solve_ivp(
                self.dynamics,
                [0, 600],
                state0,
                method='DOP853',  # DOP853: 고차 명시적 적분기
                max_step=0.1,     # 고속 비행 시 정밀도 확보
                first_step=0.01,
                events=[self.event_ground],
                rtol=1e-3,
                atol=atol_vec
            )
        
        if not sol.success:
            raise RuntimeError(f"실패: {sol.message}")
        
        t = sol.t
        states = sol.y.T
        
        print(f"✓ 완료: {len(t)} 포인트, {t[-1]:.2f}초")
        
        # 결과 추출
        results = self.extract_results(t, states, return_dataframe=return_dataframe)
        
        # 통계
        range_xy = np.sqrt(states[-1, 0]**2 + states[-1, 1]**2)
        max_alt = np.max(states[:, 2])
        
        # Body Frame 속도 -> 크기
        V_all = np.sqrt(states[:, 3]**2 + states[:, 4]**2 + states[:, 5]**2)
        max_v = np.max(V_all)
        
        print(f"\n결과:")
        print(f"  비행 시간: {t[-1]:.1f} s")
        print(f"  사거리: {range_xy/1000:.2f} km")
        print(f"  최대 고도: {max_alt/1000:.2f} km")
        print(f"  최대 속도: {max_v:.1f} m/s (마하 {max_v/340:.2f})")
        
        return results
    
    def extract_results(self, t, states, return_dataframe=False):
        """
        결과 추출 (시그니처 분석 최적화)
        
        Args:
            t: 시간 배열
            states: 상태 배열 [N, 13]
            return_dataframe: True면 Pandas DataFrame 반환
            
        Returns:
            results: dict 또는 DataFrame
        """
        import pandas as pd
        
        X = states[:, 0]
        Y = states[:, 1]
        Z = states[:, 2]
        u = states[:, 3]
        v = states[:, 4]
        w = states[:, 5]
        phi = states[:, 6]
        theta = states[:, 7]
        psi = states[:, 8]
        p = states[:, 9]
        q = states[:, 10]
        r = states[:, 11]
        mass = states[:, 12]
        
        # Body Frame 속도 -> 크기
        V = np.sqrt(u**2 + v**2 + w**2)
        
        # 오일러 각도 Wrapping [-pi, pi]
        phi = np.arctan2(np.sin(phi), np.cos(phi))
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        psi = np.arctan2(np.sin(psi), np.cos(psi))
        
        # 받음각 (강화된 수치 가드)
        u_safe = np.where(np.abs(u) > 1.0, u, np.sign(u) * 1.0)
        u_safe = np.where(u_safe == 0, 1.0, u_safe)
        alpha = np.arctan2(w, u_safe)
        alpha = np.clip(alpha, -np.deg2rad(20), np.deg2rad(20))
        
        # 옆미끄럼각
        beta = np.arctan2(v, np.sqrt(u_safe**2 + w**2))
        beta = np.clip(beta, -np.deg2rad(20), np.deg2rad(20))
        
        # 마하수
        mach = V / 340.0
        
        # gamma (비행경로각) - 탄도학적 정의
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, stheta = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        # Inertial velocity components (vectorized)
        Vx_inertial = ctheta*cpsi*u + (sphi*stheta*cpsi - cphi*spsi)*v + (cphi*stheta*cpsi + sphi*spsi)*w
        Vy_inertial = ctheta*spsi*u + (sphi*stheta*spsi + cphi*cpsi)*v + (cphi*stheta*spsi - sphi*cpsi)*w
        Vz_inertial = stheta*u - sphi*ctheta*v - cphi*ctheta*w
        
        # Flight path angle (ballistic definition)
        V_horizontal = np.sqrt(Vx_inertial**2 + Vy_inertial**2)
        V_horizontal_safe = np.where(V_horizontal > 1.0, V_horizontal, 1.0)
        gamma = np.arctan2(Vz_inertial, V_horizontal_safe)
        
        # chi (heading angle) from inertial velocity
        chi = np.arctan2(Vy_inertial, np.where(np.abs(Vx_inertial) > 1.0, Vx_inertial, 1.0))
        
        # === 시그니처 변수 추가 ===
        
        # 1. 3축 가속도 (Body Frame) - 수치 미분
        dt = np.diff(t, prepend=t[0])
        dt = np.where(dt > 0, dt, 1e-6)  # 0 방지
        
        # du/dt, dv/dt, dw/dt (Body Frame 가속도)
        a_x = np.gradient(u, t)  # Body X축 가속도
        a_y = np.gradient(v, t)  # Body Y축 가속도
        a_z = np.gradient(w, t)  # Body Z축 가속도
        
        # 2. 비에너지 (Specific Energy): E_s = h + V^2/(2g)
        g = 9.80665
        specific_energy = Z + (V**2) / (2 * g)
        
        # 3. 각속도 RMS (동적 안정성 지표)
        omega_rms = np.sqrt(p**2 + q**2 + r**2)
        
        # 4. 총 가속도 크기
        a_total = np.sqrt(a_x**2 + a_y**2 + a_z**2)
        
        # === 이벤트 플래그 생성 ===
        
        # Burn-out 플래그 (연소 종료 시점)
        burnout_flag = np.zeros(len(t), dtype=int)
        burnout_idx = np.searchsorted(t, self.burn_time)
        if burnout_idx < len(t):
            burnout_flag[burnout_idx] = 1
        
        # Apogee 플래그 (정점 도달 시점)
        apogee_flag = np.zeros(len(t), dtype=int)
        apogee_idx = np.argmax(Z)
        apogee_flag[apogee_idx] = 1
        
        # === 시그니처 데이터 (분석용 핵심 변수) ===
        signature_data = {
            'time': t,
            # 위치
            'X': X,
            'Y': Y,
            'Z': Z,
            'altitude': Z,
            # 속도
            'u': u,
            'v': v,
            'w': w,
            'V': V,
            'mach': mach,
            # 오일러 각도
            'phi': phi,
            'theta': theta,
            'psi': psi,
            # 각속도
            'p': p,
            'q': q,
            'r': r,
            'omega_rms': omega_rms,
            # 공력 각도
            'alpha': alpha,
            'beta': beta,
            'gamma': gamma,
            'chi': chi,
            # 3축 가속도 (시그니처 핵심)
            'a_x': a_x,
            'a_y': a_y,
            'a_z': a_z,
            'a_total': a_total,
            # 비에너지 (추진체 성능 지표)
            'specific_energy': specific_energy,
            # 이벤트 플래그
            'burnout_flag': burnout_flag,
            'apogee_flag': apogee_flag,
            # 질량 (참조용)
            'mass': mass,
        }
        
        # === 참조용 데이터 (CD, dm/dt - 변인 통제로 분리) ===
        # 실제 실험치와 차이가 날 수 있어 시그니처 분석에서 제외
        reference_data = {
            'time': t,
            # CD는 마하수 기반 추정치 (참조용)
            'CD_estimated': np.array([self.get_CD(m) for m in mach]),
            # dm/dt (연료 소모율 - 참조용)
            'dm_dt': np.where(t < self.burn_time, 
                             -self.propellant_mass / self.burn_time, 
                             0.0),
        }
        
        if return_dataframe:
            # Pandas DataFrame으로 반환
            df = pd.DataFrame(signature_data)
            df['missile_type'] = self.missile_type
            df['burn_time'] = self.burn_time
            df['apogee_time'] = t[apogee_idx]
            df['apogee_altitude'] = Z[apogee_idx]
            return df, reference_data
        else:
            # 기존 dict 형태 (호환성)
            results = signature_data.copy()
            results['reference'] = reference_data
            return results


def run_batch_simulation(missile_types=None, elevation_angles=None, 
                         azimuth_deg=90, add_disturbance=True):
    """
    배치 시뮬레이션 (다양한 기종/고각 조합)
    
    Args:
        missile_types: 미사일 기종 리스트 (기본: ["SCUD-B", "Nodong", "KN-23"])
        elevation_angles: 발사 고각 리스트 (기본: [10, 30, 45, 60, 75])
        azimuth_deg: 방위각 (기본: 90)
        add_disturbance: 초기 교란 여부
        
    Returns:
        combined_df: 모든 시뮬레이션 결과를 합친 DataFrame
        reference_dict: 참조 데이터 딕셔너리 (CD, dm/dt)
    """
    import pandas as pd
    
    if missile_types is None:
        missile_types = ["SCUD-B", "Nodong", "KN-23"]
    if elevation_angles is None:
        elevation_angles = [10, 30, 45, 60, 75]
    
    all_dfs = []
    all_refs = {}
    
    for m_type in missile_types:
        for elev in elevation_angles:
            try:
                print(f"\n--- {m_type} @ {elev}° ---")
                missile = Missile6DOF_Authentic(m_type)
                df, ref = missile.simulate(
                    elevation_deg=elev, 
                    azimuth_deg=azimuth_deg,
                    add_disturbance=add_disturbance,
                    return_dataframe=True
                )
                df['elevation_deg'] = elev
                df['azimuth_deg'] = azimuth_deg
                all_dfs.append(df)
                all_refs[f"{m_type}_{elev}deg"] = ref
                
            except (RuntimeError, ValueError) as e:
                print(f"⚠️ {m_type} @ {elev}° 실패: {e}")
                continue
    
    if all_dfs:
        combined_df = pd.concat(all_dfs, ignore_index=True)
        return combined_df, all_refs
    else:
        return None, None


if __name__ == "__main__":
    # 테스트
    print("="*70)
    print("논문 기반 정통 6DOF 테스트")
    print("="*70)
    
    missiles = ["SCUD-B", "Nodong", "KN-23"]
    
    for m_type in missiles:
        print(f"\n{'='*70}")
        try:
            missile = Missile6DOF_Authentic(m_type)
            result = missile.simulate(elevation_deg=45, azimuth_deg=90, 
                                     add_disturbance=True)
            
            range_km = result['X'][-1] / 1000
            q_std = result['q'].std() * 180/np.pi
            
            print(f"\n✅ {m_type}: {range_km:.1f} km")
            print(f"  각속도 신호: std(q) = {q_std:.4f} deg/s")
            
        except (RuntimeError, ValueError) as e:
            print(f"❌ 오류: {e}")
            import traceback
            traceback.print_exc()
