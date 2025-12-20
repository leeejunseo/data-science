"""
TRUE 6DOF 탄도 미사일 시뮬레이터
=====================================

리서치 기반 구현:
1. Quaternion 기반 자세 표현 (Gimbal lock 해결)
2. Moment-based 비행 프로그램 (TVC/Jet Vane)
3. 정통 6DOF 운동방정식 (Zipfel, Stevens & Lewis, MIL-HDBK-1211)

State Vector (13D):
[x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]
- (x, y, z): Inertial position [m]
- (u, v, w): Body velocity [m/s]
- (q0, q1, q2, q3): Quaternion (scalar first)
- (p, q, r): Body angular rates [rad/s]
+ mass: tracked separately

참조:
- Zipfel "Modeling and Simulation of Aerospace Vehicle Dynamics"
- Stevens & Lewis "Aircraft Control and Simulation"
- MIL-HDBK-1211 "Missile Flight Simulation"
- RocketPy equations of motion
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List
import warnings
import pandas as pd
from pathlib import Path


# ============================================================================
# Physical Constants
# ============================================================================

G0 = 9.80665          # Standard gravity [m/s²]
R_EARTH = 6371000     # Earth radius [m]
RHO_SL = 1.225        # Sea level density [kg/m³]
T_SL = 288.15         # Sea level temperature [K]
P_SL = 101325         # Sea level pressure [Pa]
R_AIR = 287.05        # Air gas constant [J/(kg·K)]
GAMMA_AIR = 1.4       # Ratio of specific heats


# ============================================================================
# Atmospheric Model (ISA 1976)
# ============================================================================

def get_atmosphere(h: float) -> Tuple[float, float, float]:
    """
    ISA 1976 Standard Atmosphere
    
    Returns: (density, pressure, temperature, sound_speed)
    """
    h = max(0, h)
    
    if h <= 11000:
        T = T_SL - 0.0065 * h
        P = P_SL * (T / T_SL) ** 5.2561
    elif h <= 25000:
        T = 216.65
        P = 22632.1 * np.exp(-0.00015768 * (h - 11000))
    elif h <= 47000:
        T = 216.65 + 0.003 * (h - 25000)
        P = 2488.7 * (T / 216.65) ** (-11.388)
    else:
        T = 270.65
        P = 120.45 * np.exp(-0.00012 * (h - 47000))
    
    rho = P / (R_AIR * T)
    a = np.sqrt(GAMMA_AIR * R_AIR * T)
    
    return rho, P, T, a


# ============================================================================
# Quaternion Utilities
# ============================================================================

def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion to unit length"""
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to Direction Cosine Matrix (Body-to-Inertial NED)
    
    Coordinate convention:
    - Body: x-forward, y-right, z-down
    - Inertial NED: x-north, y-east, z-down
    
    Quaternion convention: q = [q0, q1, q2, q3] = [scalar, vector]
    
    Reference: Stevens & Lewis Eq. 1.3-24
    """
    q0, q1, q2, q3 = q
    
    # DCM elements (Body → Inertial NED)
    C = np.array([
        [1 - 2*(q2**2 + q3**2),  2*(q1*q2 - q0*q3),      2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3),      1 - 2*(q1**2 + q3**2),  2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2),      2*(q2*q3 + q0*q1),      1 - 2*(q1**2 + q2**2)]
    ])
    
    return C


def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Convert Euler angles (3-2-1 sequence: yaw-pitch-roll) to quaternion
    
    Args:
        phi: Roll angle [rad]
        theta: Pitch angle [rad]
        psi: Yaw angle [rad]
    
    Returns:
        q: Quaternion [q0, q1, q2, q3]
    """
    cphi, sphi = np.cos(phi/2), np.sin(phi/2)
    ctheta, stheta = np.cos(theta/2), np.sin(theta/2)
    cpsi, spsi = np.cos(psi/2), np.sin(psi/2)
    
    q0 = cphi * ctheta * cpsi + sphi * stheta * spsi
    q1 = sphi * ctheta * cpsi - cphi * stheta * spsi
    q2 = cphi * stheta * cpsi + sphi * ctheta * spsi
    q3 = cphi * ctheta * spsi - sphi * stheta * cpsi
    
    return quat_normalize(np.array([q0, q1, q2, q3]))


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (3-2-1 sequence)
    
    Returns: (phi, theta, psi) in radians
    """
    q0, q1, q2, q3 = q
    
    # Roll (phi)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1**2 + q2**2)
    phi = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (theta) - handle gimbal lock
    sinp = 2 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    theta = np.arcsin(sinp)
    
    # Yaw (psi)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2**2 + q3**2)
    psi = np.arctan2(siny_cosp, cosy_cosp)
    
    return phi, theta, psi


# ============================================================================
# Missile Configuration
# ============================================================================

@dataclass
class MissileConfig:
    """Missile configuration parameters"""
    name: str
    
    # Mass properties
    mass_total: float       # Launch mass [kg]
    mass_propellant: float  # Propellant mass [kg]
    mass_dry: float         # Dry mass [kg]
    
    # Geometry
    diameter: float         # Body diameter [m]
    length: float          # Body length [m]
    reference_area: float  # Reference area (cross-section) [m²]
    
    # Propulsion
    isp_sea: float         # Sea level Isp [s]
    isp_vac: float         # Vacuum Isp [s]
    burn_time: float       # Motor burn time [s]
    
    # Flight program
    vertical_time: float   # Vertical flight duration [s]
    pitch_time: float      # Pitch maneuver duration [s]
    pitch_angle: float     # Target pitch-over angle [rad]
    
    # Moments of inertia (estimated from cylinder)
    I_xx: float = None     # Roll moment of inertia [kg·m²]
    I_yy: float = None     # Pitch moment of inertia [kg·m²]
    I_zz: float = None     # Yaw moment of inertia [kg·m²]
    
    # Aerodynamic derivatives (from research)
    C_m_alpha: float = -3.0    # Pitching moment slope [/rad] (research: -2 to -8)
    C_mq: float = -150.0       # Pitch damping [/rad] (research: -100 to -300)
    C_nr: float = -150.0       # Yaw damping [/rad]
    C_lp: float = -30.0        # Roll damping [/rad] (research: -10 to -50)
    CL_alpha: float = 3.5      # Lift curve slope [/rad]
    
    # TVC parameters (V-2/SCUD use jet vanes with larger deflection)
    tvc_max_angle: float = 15.0 * np.pi/180  # Max gimbal/vane angle [rad] - jet vanes allow larger angles
    tvc_moment_arm: float = None  # Gimbal to CG distance [m]
    
    def __post_init__(self):
        """Calculate derived properties"""
        self.mass_dry = self.mass_total - self.mass_propellant
        
        # Estimate moments of inertia (slender cylinder approximation)
        if self.I_xx is None:
            self.I_xx = 0.5 * self.mass_total * (self.diameter/2)**2
        if self.I_yy is None:
            self.I_yy = self.mass_total * (self.length**2/12 + (self.diameter/2)**2/4)
        if self.I_zz is None:
            self.I_zz = self.I_yy
        
        # TVC moment arm (approximate as 80% of length from nose)
        if self.tvc_moment_arm is None:
            self.tvc_moment_arm = 0.4 * self.length


# Predefined missile configurations
# 수정된 파라미터: 문헌 기반 질량비 조정
MISSILES = {
    "SCUD-B": MissileConfig(
        name="SCUD-B",
        mass_total=5900,          # kg (문헌)
        mass_propellant=3800,     # kg (수정: 현실적 질량비)
        mass_dry=2100,            # kg (수정: 탄두+구조체)
        diameter=0.88,
        length=10.94,
        reference_area=np.pi * (0.88/2)**2,
        isp_sea=226,              # s (문헌 기반)
        isp_vac=254,              # s
        burn_time=65,
        vertical_time=10,
        pitch_time=15,
        pitch_angle=20 * np.pi/180,
        # 안정성 미분계수 (리서치 범위 내)
        C_m_alpha=-2.0,           # /rad (리서치: -2 ~ -8)
        C_mq=-150.0,              # /rad
        C_lp=-25.0,
    ),
    "Nodong": MissileConfig(
        name="Nodong",
        mass_total=16250,         # kg
        mass_propellant=12150,    # kg (질량비 ~2.8 유지)
        mass_dry=4100,            # kg
        diameter=1.36,
        length=16.4,
        reference_area=np.pi * (1.36/2)**2,
        isp_sea=250,              # s (UDMH/RFNA)
        isp_vac=276,              # s
        burn_time=95,             # s (더 긴 연소시간)
        vertical_time=10,
        pitch_time=25,
        pitch_angle=15 * np.pi/180,
        C_m_alpha=-1.5,
        C_mq=-180.0,
        C_lp=-22.0,
    ),
    "KN-23": MissileConfig(
        name="KN-23",
        mass_total=3415,          # kg (문헌 기반)
        mass_propellant=2500,     # kg (고체 추진제)
        mass_dry=915,             # kg
        diameter=0.95,
        length=7.5,
        reference_area=np.pi * (0.95/2)**2,
        isp_sea=260,              # s (고체) - 향상된 Isp
        isp_vac=275,              # s
        burn_time=50,             # s (연장된 연소시간으로 사거리 확보)
        vertical_time=3,          # KN-23: 짧은 수직 비행
        pitch_time=12,            # KN-23: 피치오버 시간
        pitch_angle=15 * np.pi/180,  # KN-23: 낮은 피치각 (편평 탄도)
        C_m_alpha=-3.0,           # KN-23: 정적 안정성
        C_mq=-150.0,              # KN-23: 피치 댐핑
        C_lp=-25.0,
        CL_alpha=4.0,             # KN-23: 높은 양력 기울기 (기동성)
    ),
}


# ============================================================================
# Drag Coefficient Model (KAIST validated)
# ============================================================================

def get_cd(mach: float, alpha_deg: float = 0) -> float:
    """
    Drag coefficient as function of Mach number and angle of attack
    
    Based on KAIST validated data for slender bodies
    """
    # Base CD vs Mach (transonic peak)
    if mach < 0.8:
        cd_base = 0.30
    elif mach < 1.0:
        cd_base = 0.30 + (0.80 - 0.30) * (mach - 0.8) / 0.2
    elif mach < 1.2:
        cd_base = 0.80 - (0.80 - 0.50) * (mach - 1.0) / 0.2
    elif mach < 2.0:
        cd_base = 0.50 - (0.50 - 0.35) * (mach - 1.2) / 0.8
    elif mach < 3.0:
        cd_base = 0.35 - (0.35 - 0.30) * (mach - 2.0) / 1.0
    else:
        cd_base = 0.30
    
    # Angle of attack contribution
    cd_alpha = 0.01 * (alpha_deg / 10)**2
    
    return cd_base + cd_alpha


# ============================================================================
# TRUE 6DOF Simulator
# ============================================================================

class True6DOFSimulator:
    """
    True 6DOF Ballistic Missile Simulator
    
    Implements:
    - Quaternion-based attitude (no gimbal lock)
    - Body-frame force/moment equations
    - TVC moment-based attitude control
    - Variable mass properties
    """
    
    def __init__(self, missile_type: str = "SCUD-B"):
        if missile_type not in MISSILES:
            raise ValueError(f"Unknown missile type: {missile_type}")
        
        self.cfg = MISSILES[missile_type]
        self.missile_type = missile_type
        
        # State indices
        self.IX, self.IY, self.IZ = 0, 1, 2           # Position
        self.IU, self.IV, self.IW = 3, 4, 5           # Body velocity
        self.IQ0, self.IQ1, self.IQ2, self.IQ3 = 6, 7, 8, 9  # Quaternion
        self.IP, self.IQ, self.IR = 10, 11, 12        # Angular rates
        
        # Simulation state
        self.mass = self.cfg.mass_total
        self.t = 0
        
        # Logging
        self.log = {
            'time': [], 'phase': [],
            'x': [], 'y': [], 'z': [],
            'V': [], 'mach': [],
            'phi': [], 'theta': [], 'psi': [],
            'p': [], 'q': [], 'r': [],
            'alpha': [], 'beta': [],
            'thrust': [], 'mass': [],
            'M_pitch_tvc': [], 'M_pitch_aero': [],
        }
        
        print(f"✓ True6DOF initialized: {self.cfg.name}")
        print(f"  Mass: {self.cfg.mass_total} kg, Burn time: {self.cfg.burn_time} s")
        print(f"  C_m_alpha: {self.cfg.C_m_alpha}, C_mq: {self.cfg.C_mq}")
    
    def get_mass_properties(self, t: float) -> Tuple[float, float, float, float]:
        """
        Get current mass and moments of inertia
        
        Mass decreases linearly during burn
        MOI scales with mass (simplified)
        """
        if t < self.cfg.burn_time:
            fuel_remaining = 1.0 - t / self.cfg.burn_time
            m = self.cfg.mass_dry + self.cfg.mass_propellant * fuel_remaining
        else:
            m = self.cfg.mass_dry
        
        # Scale MOI with mass ratio (simplified)
        mass_ratio = m / self.cfg.mass_total
        I_xx = self.cfg.I_xx * mass_ratio
        I_yy = self.cfg.I_yy * mass_ratio
        I_zz = self.cfg.I_zz * mass_ratio
        
        return m, I_xx, I_yy, I_zz
    
    def get_thrust(self, t: float, h: float) -> float:
        """
        Calculate thrust accounting for altitude (pressure) effects
        """
        if t >= self.cfg.burn_time:
            return 0.0
        
        # Isp varies with altitude
        _, P, _, _ = get_atmosphere(h)
        p_ratio = P / P_SL
        isp = self.cfg.isp_sea + (self.cfg.isp_vac - self.cfg.isp_sea) * (1 - p_ratio)
        
        # Mass flow rate
        mdot = self.cfg.mass_propellant / self.cfg.burn_time
        
        return isp * mdot * G0
    
    def get_flight_phase(self, t: float, altitude: float = None) -> str:
        """
        Determine current flight phase
        
        KN-23 특수 단계:
        - Pull-up: 하강 중 25~45km 구간
        - Terminal: 최종 돌입
        """
        # KN-23 Pull-up/Terminal 단계 확인
        if self.missile_type == "KN-23" and altitude is not None:
            is_descending = getattr(self, '_is_descending', False)
            
            if is_descending and 25000 <= altitude <= 45000:
                return "Pull-up"
            if is_descending and altitude < 25000:
                return "Terminal"
        
        # 공통 비행 단계
        if t < self.cfg.vertical_time:
            return "Vertical"
        elif t < self.cfg.vertical_time + self.cfg.pitch_time:
            return "Pitch"
        elif t < self.cfg.burn_time:
            return "Gravity Turn"
        else:
            return "Ballistic"
    
    def get_target_pitch_rate(self, t: float, theta_current: float) -> float:
        """
        Calculate target pitch rate for flight program
        
        Flight profile:
        1. Vertical: Hold theta = 90°
        2. Pitch: Pitch over from 90° to target_elevation
        3. Gravity Turn: Maintain alpha ≈ 0 (thrust along velocity)
        4. Ballistic: Free flight with aerodynamic stabilization
        
        Returns desired dtheta/dt [rad/s]
        """
        phase = self.get_flight_phase(t)
        target_elevation = getattr(self, 'target_elevation', np.pi/4)
        
        if phase == "Vertical":
            # Hold vertical (theta = 90°)
            theta_target = np.pi/2
            theta_error = theta_target - theta_current
            return 1.0 * theta_error
            
        elif phase == "Pitch":
            # Pitch over from 90° to target_elevation
            progress = (t - self.cfg.vertical_time) / self.cfg.pitch_time
            progress = np.clip(progress, 0, 1)
            
            # Smooth S-curve profile
            if progress < 0.5:
                smooth = 2 * progress**2
            else:
                smooth = 1 - 2 * (1 - progress)**2
            
            pitch_range = np.pi/2 - target_elevation
            theta_target = np.pi/2 - pitch_range * smooth
            theta_error = theta_target - theta_current
            
            # Rate-limited pitch command
            dtheta_max = pitch_range / self.cfg.pitch_time * 2.0
            dtheta_cmd = np.clip(2.0 * theta_error, -dtheta_max, dtheta_max)
            
            return dtheta_cmd
            
        else:
            # Gravity Turn / Ballistic: 
            # For gravity turn, we want to follow the velocity vector
            # Return 0 - natural dynamics will handle it
            # But we provide a small stabilizing rate if needed
            return 0.0
    
    def compute_tvc_moment(self, t: float, theta: float, q_rate: float, 
                          thrust: float, alpha: float = 0.0, q_dyn: float = 0.0,
                          altitude: float = 0.0, gamma: float = 0.0) -> float:
        """
        Compute TVC control moment for pitch channel
        
        개선사항:
        1. Gravity Turn에서 속도 벡터 추종 (alpha → 0)
        2. 받음각 10° 제한
        3. 동압 기반 가변 게인 (재진입 안정화)
        4. KN-23 전용: 전 구간 RCS 자세 제어 + Pull-up 기동
        
        Returns: M_pitch [N·m]
        """
        m, I_xx, I_yy, I_zz = self.get_mass_properties(t)
        phase = self.get_flight_phase(t, altitude)
        target_elevation = getattr(self, 'target_elevation', np.pi/4)
        
        # 받음각 제한 (10° = 0.1745 rad)
        ALPHA_MAX = 10.0 * np.pi / 180
        
        # ================================================================
        # KN-23 전용 제어 로직
        # ================================================================
        if self.missile_type == "KN-23":
            return self._compute_kn23_moment(t, theta, q_rate, thrust, alpha, 
                                             q_dyn, altitude, gamma, phase,
                                             m, I_yy, ALPHA_MAX)
        
        # ================================================================
        # 일반 미사일 제어 로직
        # ================================================================
        if thrust < 100:
            return 0.0
        
        # Feedforward pitch rate command
        if phase == "Vertical":
            q_cmd = 0.0
            
        elif phase == "Pitch":
            pitch_range = np.pi/2 - target_elevation
            q_cmd = -pitch_range / self.cfg.pitch_time
            
        elif phase == "Gravity Turn":
            K_alpha = 2.0
            q_cmd = -K_alpha * alpha
            
            if abs(alpha) > ALPHA_MAX:
                q_cmd = -3.0 * np.sign(alpha) * (abs(alpha) - ALPHA_MAX * 0.5)
        else:
            return 0.0
        
        # 동압 기반 가변 게인
        q_dyn_ref = 50000
        gain_scale = 1.0 / (1.0 + q_dyn / q_dyn_ref)
        
        omega_n = 3.0 * gain_scale
        K_rate = I_yy * omega_n
        
        M_tvc = K_rate * (q_cmd - q_rate)
        
        # TVC 권한 제한
        M_max = thrust * self.cfg.tvc_moment_arm * np.sin(self.cfg.tvc_max_angle)
        M_tvc = np.clip(M_tvc, -M_max, M_max)
        
        return M_tvc
    
    def _compute_kn23_moment(self, t: float, theta: float, q_rate: float,
                             thrust: float, alpha: float, q_dyn: float,
                             altitude: float, gamma: float, phase: str,
                             m: float, I_yy: float, ALPHA_MAX: float) -> float:
        """
        KN-23 전용 제어 모멘트 계산 (Quasi-Ballistic Trajectory)
        
        특징:
        1. 편평 탄도: 빠른 피치오버 → 40-60km 고도 유지
        2. 받음각 15° 이하 엄격 제한
        3. Pull-up 기동: 종말 단계에서 양력 활용
        4. 전 구간 RCS 자세 제어
        """
        target_elevation = getattr(self, 'target_elevation', np.pi/4)
        
        # ================================================================
        # 받음각 제한: 15° (0.2618 rad) - 구조적 한계
        # ================================================================
        ALPHA_LIMIT = 15.0 * np.pi / 180  # 15도 제한
        
        # ================================================================
        # 1. 기본 제어: 속도 벡터 추종 (alpha → 0)
        # ================================================================
        K_alpha = 8.0  # KN-23: 강화된 받음각 추종 게인
        q_cmd_alpha = -K_alpha * alpha
        
        # 받음각 15° 제한 - 강력한 복원력
        if abs(alpha) > ALPHA_LIMIT * 0.5:  # 7.5도 이상부터 강한 복원
            overshoot = abs(alpha) - ALPHA_LIMIT * 0.5
            q_cmd_alpha = -15.0 * np.sign(alpha) * overshoot
        
        # ================================================================
        # 2. KN-23 Quasi-Ballistic 비행 단계별 제어
        # ================================================================
        if phase == "Vertical":
            q_cmd = 0.0
            
        elif phase == "Pitch":
            # KN-23: 빠른 피치오버 (15도 목표)
            target_pitch = 15 * np.pi / 180
            pitch_range = np.pi/2 - target_pitch
            
            # 강화된 피치오버 명령
            q_cmd = -pitch_range / self.cfg.pitch_time * 2.0
        
        elif phase == "Gravity Turn":
            # KN-23: 15도 피치각 유지
            target_pitch = 15 * np.pi / 180
            theta_error = target_pitch - theta
            
            K_theta = 3.0
            q_cmd = K_theta * theta_error + q_cmd_alpha * 0.5
        
        elif phase == "Ballistic":
            # KN-23: 속도 벡터 추종 + 고도 유지
            TARGET_ALT = 50000
            
            if altitude > 60000:
                q_cmd = q_cmd_alpha - 0.1
            elif altitude < 40000 and gamma > -5 * np.pi / 180:
                q_cmd = q_cmd_alpha + 0.05
            else:
                q_cmd = q_cmd_alpha
            
        elif phase == "Pull-up":
            # ================================================================
            # Pull-up 기동: 종말 단계 양력 활용
            # 고도 25-45km에서 받음각 증가로 양력 생성
            # ================================================================
            pullup_alpha_target = 10.0 * np.pi / 180  # 목표 받음각 10도
            
            # 고도에 따른 Pull-up 강도
            if altitude > 35000:
                intensity = (45000 - altitude) / 10000
            else:
                intensity = (altitude - 25000) / 10000
            intensity = np.clip(intensity, 0, 1)
            
            # Pull-up 명령: 받음각을 목표값으로 유도
            alpha_error = pullup_alpha_target * intensity - alpha
            q_cmd_pullup = 4.0 * alpha_error
            
            # 받음각 15도 제한 유지
            if abs(alpha) > ALPHA_LIMIT * 0.8:
                q_cmd = q_cmd_alpha
            else:
                q_cmd = q_cmd_pullup
        
        elif phase == "Terminal":
            # ================================================================
            # Terminal 단계: 최종 돌입
            # 받음각 최소화하여 안정적 돌입
            # ================================================================
            q_cmd = q_cmd_alpha  # 속도 벡터 추종
        
        else:
            q_cmd = q_cmd_alpha
        
        # ================================================================
        # 3. 각속도 댐핑 (진동 억제)
        # ================================================================
        K_damp = 2.0  # 댐핑 게인
        q_cmd = q_cmd - K_damp * q_rate
        
        # ================================================================
        # 4. 제어 모멘트 계산
        # ================================================================
        q_dyn_ref = 50000
        gain_scale = 1.0 / (1.0 + q_dyn / q_dyn_ref)
        
        if thrust > 100:
            # TVC 제어
            omega_n = 6.0 * gain_scale
            K_rate = I_yy * omega_n
            M_control = K_rate * q_cmd
            
            M_max = thrust * self.cfg.tvc_moment_arm * np.sin(self.cfg.tvc_max_angle)
            M_control = np.clip(M_control, -M_max, M_max)
        else:
            # RCS 제어 (탄도 비행 중)
            omega_n = 5.0 * gain_scale
            K_rate = I_yy * omega_n
            M_control = K_rate * q_cmd
            
            # RCS 권한 (기동성 탄두)
            M_max_rcs = 0.4 * m * 9.81 * self.cfg.length * 0.4
            M_control = np.clip(M_control, -M_max_rcs, M_max_rcs)
        
        return M_control
    
    def get_theta_target(self, t: float) -> float:
        """Get target pitch angle for current time"""
        phase = self.get_flight_phase(t)
        target_elevation = getattr(self, 'target_elevation', np.pi/4)
        
        if phase == "Vertical":
            return np.pi/2
        elif phase == "Pitch":
            progress = (t - self.cfg.vertical_time) / self.cfg.pitch_time
            progress = np.clip(progress, 0, 1)
            # Smooth S-curve
            if progress < 0.5:
                smooth = 2 * progress**2
            else:
                smooth = 1 - 2 * (1 - progress)**2
            pitch_range = np.pi/2 - target_elevation
            return np.pi/2 - pitch_range * smooth
        else:
            return target_elevation
    
    def dynamics(self, t: float, state: np.ndarray) -> np.ndarray:
        """
        6DOF Equations of Motion
        
        Reference: Zipfel Ch. 5, Stevens & Lewis Ch. 1
        
        State: [x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]
        """
        # Unpack state
        x, y, z = state[0:3]  # NED: z is down, altitude = -z
        u, v, w = state[3:6]
        quat = state[6:10]
        p, q_rate, r = state[10:13]
        
        # Ensure valid state
        altitude = -z  # NED: altitude = -z
        altitude = max(altitude, 0)
        quat = quat_normalize(quat)
        
        # Get DCM (Body → Inertial)
        C_bi = quat_to_dcm(quat)
        C_ib = C_bi.T  # Inertial → Body
        
        # Euler angles (for logging and control)
        phi, theta, psi = quat_to_euler(quat)
        
        # Velocity magnitude
        V = np.sqrt(u**2 + v**2 + w**2)
        V = max(V, 0.1)
        
        # Angle of attack and sideslip
        alpha = np.arctan2(w, max(u, 0.1))
        beta = np.arcsin(np.clip(v / V, -1, 1))
        
        # Atmospheric properties
        rho, P, T, a = get_atmosphere(altitude)
        mach = V / a
        q_dyn = 0.5 * rho * V**2
        
        # Mass properties
        m, I_xx, I_yy, I_zz = self.get_mass_properties(t)
        
        # Reference quantities
        S = self.cfg.reference_area
        d = self.cfg.diameter  # Reference length for missiles
        
        # ================================================================
        # FORCES (Body Frame)
        # ================================================================
        
        # 1. Thrust (along body x-axis)
        thrust = self.get_thrust(t, altitude)
        F_thrust = np.array([thrust, 0, 0])
        
        # 2. Aerodynamic forces
        CD = get_cd(mach, np.abs(alpha) * 180/np.pi)
        CL = self.cfg.CL_alpha * alpha
        CY = -0.5 * beta  # Side force
        
        # Drag opposes velocity
        D = q_dyn * S * CD
        L = q_dyn * S * CL
        Y = q_dyn * S * CY
        
        # Body frame aerodynamic forces
        # (assuming small angles: L acts in -z, D in -x)
        F_aero = np.array([-D, Y, -L])
        
        # 3. Gravity (transform to body frame)
        # NED frame: z-down, so gravity is +z
        g = G0 * (R_EARTH / (R_EARTH + altitude))**2
        g_inertial = np.array([0, 0, m * g])  # NED: gravity is +z (down)
        g_body = C_ib @ g_inertial
        
        # Total force
        F_total = F_thrust + F_aero + g_body
        
        # ================================================================
        # MOMENTS (Body Frame)
        # ================================================================
        
        # 1. Aerodynamic moments
        # Static stability (C_m_alpha)
        M_static = q_dyn * S * d * self.cfg.C_m_alpha * alpha
        N_static = q_dyn * S * d * self.cfg.C_m_alpha * beta  # Yaw (same coeff for axisymmetric)
        
        # Damping moments
        M_damp = q_dyn * S * d * self.cfg.C_mq * (q_rate * d / (2*V)) if V > 1 else 0
        N_damp = q_dyn * S * d * self.cfg.C_nr * (r * d / (2*V)) if V > 1 else 0
        L_damp = q_dyn * S * d * self.cfg.C_lp * (p * d / (2*V)) if V > 1 else 0
        
        M_aero_pitch = M_static + M_damp
        M_aero_yaw = N_static + N_damp
        M_aero_roll = L_damp
        
        # 2. TVC/RCS control moment
        # KN-23: altitude, gamma 추가 전달하여 Pull-up 시그니처 구현
        # 비행 경로각 (gamma) 계산
        v_inertial_temp = C_bi @ np.array([u, v, w])
        V_horizontal = np.sqrt(v_inertial_temp[0]**2 + v_inertial_temp[1]**2)
        V_vertical = -v_inertial_temp[2]  # NED: -dz = 상승 속도
        gamma = np.arctan2(V_vertical, max(V_horizontal, 0.1))
        
        # KN-23 하강 상태 추적 (Pull-up 트리거용)
        if self.missile_type == "KN-23":
            prev_alt = getattr(self, '_prev_altitude', altitude)
            if altitude < prev_alt - 10:  # 10m 이상 하강
                self._is_descending = True
            self._prev_altitude = altitude
        
        M_tvc = self.compute_tvc_moment(t, theta, q_rate, thrust, alpha, q_dyn, altitude, gamma)
        
        # 3. Burnout disturbance
        M_disturbance = 0
        if abs(t - self.cfg.burn_time) < 0.5:
            # Exponential decay disturbance at burnout
            dt = t - self.cfg.burn_time
            M_disturbance = 0.05 * I_yy * np.exp(-10 * dt**2) * np.sin(10 * t)
        
        # Total moments
        L_total = M_aero_roll
        M_total = M_aero_pitch + M_tvc + M_disturbance
        N_total = M_aero_yaw
        
        # ================================================================
        # EQUATIONS OF MOTION
        # ================================================================
        
        # 1. Translational (Body frame) - Zipfel Eq. 5.24
        du = F_total[0]/m + r*v - q_rate*w
        dv = F_total[1]/m + p*w - r*u
        dw = F_total[2]/m + q_rate*u - p*v
        
        # 2. Rotational (Euler's equation) - Zipfel Eq. 5.31
        dp = (L_total + (I_yy - I_zz) * q_rate * r) / I_xx
        dq = (M_total + (I_zz - I_xx) * p * r) / I_yy
        dr = (N_total + (I_xx - I_yy) * p * q_rate) / I_zz
        
        # 3. Quaternion kinematics - Zipfel Eq. 4.80
        q0, q1, q2, q3 = quat
        dq0 = 0.5 * (-q1*p - q2*q_rate - q3*r)
        dq1 = 0.5 * ( q0*p - q3*q_rate + q2*r)
        dq2 = 0.5 * ( q3*p + q0*q_rate - q1*r)
        dq3 = 0.5 * (-q2*p + q1*q_rate + q0*r)
        
        # 4. Position (Inertial frame)
        v_body = np.array([u, v, w])
        v_inertial = C_bi @ v_body
        dx, dy, dz = v_inertial
        
        # Assemble derivative
        dstate = np.array([
            dx, dy, dz,
            du, dv, dw,
            dq0, dq1, dq2, dq3,
            dp, dq, dr
        ])
        
        # Safety clipping
        dstate = np.clip(dstate, -1e6, 1e6)
        
        return dstate
    
    def ground_event(self, t: float, state: np.ndarray) -> float:
        """Event function for ground impact (NED: z > 0 means below ground)"""
        if t < 30:
            return -1000  # Keep negative during initial flight
        return -state[2]  # altitude = -z, trigger when altitude < 0
    ground_event.terminal = True
    ground_event.direction = -1
    
    def simulate(self, elevation_deg: float = 45, azimuth_deg: float = 90,
                max_time: float = 600) -> Dict:
        """
        Run 6DOF simulation
        
        Args:
            elevation_deg: Launch elevation angle [deg]
            azimuth_deg: Launch azimuth angle [deg]
            max_time: Maximum simulation time [s]
            
        Returns:
            Dictionary with trajectory data
        """
        print(f"\n{'='*60}")
        print(f"True 6DOF Simulation: {self.cfg.name}")
        print(f"  Elevation: {elevation_deg}°, Azimuth: {azimuth_deg}°")
        print(f"{'='*60}")
        
        # Store target elevation for flight program
        # KN-23: 편평 탄도를 위해 목표 발사각을 15도로 강제 설정
        if self.missile_type == "KN-23":
            self.target_elevation = 15 * np.pi/180  # 15도 목표 (편평 탄도)
            print(f"  KN-23 Quasi-Ballistic: Target elevation = 15°")
        else:
            self.target_elevation = elevation_deg * np.pi/180
        
        # 초기각 점프 해결: azimuth를 정확히 설정
        # NED 좌표계에서 azimuth 90° = East = y축 방향
        psi0 = (90 - azimuth_deg) * np.pi/180
        
        # ================================================================
        # 초기 조건 설정
        # 모든 미사일: 수직 발사 후 피치오버
        # ================================================================
        theta0 = np.pi/2  # 90도 수직 발사
        u0 = 0.1  # 작은 초기 속도
        
        # 쿼터니언 생성 (Euler to Quaternion)
        phi0 = 0  # Roll = 0
        quat0 = quat_from_euler(phi0, theta0, psi0)
        
        state0 = np.array([
            0, 0, 0,                    # Position (NED: z down)
            u0, 0, 0,                   # Body velocity (작은 전진 속도)
            quat0[0], quat0[1], quat0[2], quat0[3],  # Quaternion (정규화됨)
            0, 0, 0                     # Angular rates (정지 상태에서 시작)
        ])
        
        # Integrate
        sol = solve_ivp(
            self.dynamics,
            [0, max_time],
            state0,
            method='RK45',
            events=[self.ground_event],
            max_step=0.5,
            rtol=1e-6,
            atol=1e-8
        )
        
        # Process results
        t = sol.t
        states = sol.y
        
        # Extract trajectories
        x, y, z = states[0], states[1], states[2]
        altitude = -z  # NED: altitude = -z
        u, v, w = states[3], states[4], states[5]
        
        # Compute derived quantities
        V = np.sqrt(u**2 + v**2 + w**2)
        
        # Euler angles from quaternions
        phi_arr, theta_arr, psi_arr = [], [], []
        for i in range(len(t)):
            quat = states[6:10, i]
            phi, theta, psi = quat_to_euler(quat)
            phi_arr.append(phi)
            theta_arr.append(theta)
            psi_arr.append(psi)
        
        phi_arr = np.array(phi_arr)
        theta_arr = np.array(theta_arr)
        psi_arr = np.array(psi_arr)
        
        # Angular rates
        p_arr = states[10]
        q_arr = states[11]
        r_arr = states[12]
        
        # Calculate range
        range_km = np.sqrt(x[-1]**2 + y[-1]**2) / 1000
        max_alt_km = np.max(altitude) / 1000
        flight_time = t[-1]
        
        print(f"\nResults:")
        print(f"  Range: {range_km:.1f} km")
        print(f"  Max altitude: {max_alt_km:.1f} km")
        print(f"  Flight time: {flight_time:.1f} s")
        print(f"  q (pitch rate) std: {np.std(q_arr)*180/np.pi:.2f} °/s")
        
        # Reference range comparison
        ref_ranges = {"SCUD-B": 300, "Nodong": 1500, "KN-23": 690}
        if self.missile_type in ref_ranges:
            ref = ref_ranges[self.missile_type]
            error = (range_km - ref) / ref * 100
            print(f"  Reference: {ref} km, Error: {error:+.1f}%")
        
        # 받음각 계산
        alpha_arr = np.arctan2(w, np.maximum(u, 0.1))
        beta_arr = np.arcsin(np.clip(v / np.maximum(V, 0.1), -1, 1))
        
        # 최대 받음각 확인
        max_alpha_deg = np.max(np.abs(alpha_arr)) * 180 / np.pi
        print(f"  Max |alpha|: {max_alpha_deg:.1f}°")
        
        return {
            'time': t,
            'x': x, 'y': y, 'z': altitude,  # Return altitude, not NED z
            'u': u, 'v': v, 'w': w,
            'V': V,
            'phi': phi_arr, 'theta': theta_arr, 'psi': psi_arr,
            'p': p_arr, 'q': q_arr, 'r': r_arr,
            'alpha': alpha_arr, 'beta': beta_arr,
            'range_km': range_km,
            'max_alt_km': max_alt_km,
            'flight_time': flight_time,
        }


# ============================================================================
# 데이터셋 추출 및 배치 시뮬레이션
# ============================================================================

def extract_results(sim_result: Dict, missile_type: str, elevation_deg: float,
                    include_reference: bool = False) -> pd.DataFrame:
    """
    시뮬레이션 결과에서 시그니처 분석용 데이터를 추출
    
    포함 항목:
    - 3축 가속도 (Body Frame: a_u, a_v, a_w)
    - 3축 가속도 (관성 좌표계: a_x, a_y, a_z)
    - 비에너지 (E_s = altitude + V^2/(2g))
    - 이벤트 플래그 (Burn-out, Apogee)
    
    Args:
        sim_result: True6DOFSimulator.simulate() 반환값
        missile_type: 미사일 기종명
        elevation_deg: 발사 고각 [deg]
        include_reference: CD, 연료소모량 등 참조용 데이터 포함 여부
        
    Returns:
        시그니처 분석용 DataFrame
    """
    t = sim_result['time']
    n_points = len(t)
    
    # 기본 데이터 추출
    x = sim_result['x']
    y = sim_result['y']
    altitude = sim_result['z']
    
    u = sim_result['u']
    v = sim_result['v']
    w = sim_result['w']
    V = sim_result['V']
    
    phi = sim_result['phi']
    theta = sim_result['theta']
    psi = sim_result['psi']
    
    p = sim_result['p']
    q = sim_result['q']
    r = sim_result['r']
    
    alpha = sim_result.get('alpha', np.arctan2(w, np.maximum(u, 0.1)))
    beta = sim_result.get('beta', np.arcsin(np.clip(v / np.maximum(V, 0.1), -1, 1)))
    
    # ================================================================
    # 1. 가속도 계산 (Body Frame)
    # ================================================================
    a_u = np.gradient(u, t)  # Body x-axis acceleration
    a_v = np.gradient(v, t)  # Body y-axis acceleration
    a_w = np.gradient(w, t)  # Body z-axis acceleration
    
    # ================================================================
    # 2. 관성 좌표계 가속도 계산
    # ================================================================
    V_x = np.gradient(x, t)
    V_y = np.gradient(y, t)
    V_z = np.gradient(altitude, t)
    
    a_x = np.gradient(V_x, t)
    a_y = np.gradient(V_y, t)
    a_z = np.gradient(V_z, t)
    a_total = np.sqrt(a_x**2 + a_y**2 + a_z**2)
    
    # ================================================================
    # 3. 비에너지 (Specific Energy) 계산
    # ================================================================
    E_s = altitude + (V**2) / (2 * G0)
    
    # ================================================================
    # 4. 마하수 및 동압 계산
    # ================================================================
    mach = np.zeros(n_points)
    q_dyn = np.zeros(n_points)
    
    for i in range(n_points):
        rho, P, T, a = get_atmosphere(altitude[i])
        mach[i] = V[i] / a
        q_dyn[i] = 0.5 * rho * V[i]**2
    
    # ================================================================
    # 5. 이벤트 플래그 생성
    # ================================================================
    cfg = MISSILES[missile_type]
    
    # Burn-out 플래그
    flag_burnout = np.zeros(n_points, dtype=int)
    burnout_idx = np.searchsorted(t, cfg.burn_time)
    if burnout_idx < n_points:
        flag_burnout[burnout_idx] = 1
    
    # Apogee 플래그
    flag_apogee = np.zeros(n_points, dtype=int)
    apogee_idx = np.argmax(altitude)
    flag_apogee[apogee_idx] = 1
    
    # 비행 단계 (Phase)
    phase = np.zeros(n_points, dtype=int)
    for i, ti in enumerate(t):
        if ti < cfg.vertical_time:
            phase[i] = 0  # Vertical
        elif ti < cfg.vertical_time + cfg.pitch_time:
            phase[i] = 1  # Pitch
        elif ti < cfg.burn_time:
            phase[i] = 2  # Gravity Turn
        else:
            phase[i] = 3  # Ballistic
    
    # ================================================================
    # 6. DataFrame 구성
    # ================================================================
    data = {
        'time': t,
        'x': x, 'y': y, 'altitude': altitude,
        'u': u, 'v': v, 'w': w, 'V': V,
        'a_u': a_u, 'a_v': a_v, 'a_w': a_w,
        'a_x': a_x, 'a_y': a_y, 'a_z': a_z, 'a_total': a_total,
        'phi': phi, 'theta': theta, 'psi': psi,
        'p': p, 'q': q, 'r': r,
        'alpha': alpha, 'beta': beta,
        'mach': mach, 'q_dyn': q_dyn,
        'E_s': E_s,
        'flag_burnout': flag_burnout,
        'flag_apogee': flag_apogee,
        'phase': phase,
        'missile_type': missile_type,
        'elevation_deg': elevation_deg,
    }
    
    if include_reference:
        mass = np.zeros(n_points)
        thrust = np.zeros(n_points)
        for i, ti in enumerate(t):
            if ti < cfg.burn_time:
                fuel_remaining = 1.0 - ti / cfg.burn_time
                mass[i] = cfg.mass_dry + cfg.mass_propellant * fuel_remaining
                rho, P, T, a = get_atmosphere(altitude[i])
                p_ratio = P / P_SL
                isp = cfg.isp_sea + (cfg.isp_vac - cfg.isp_sea) * (1 - p_ratio)
                mdot = cfg.mass_propellant / cfg.burn_time
                thrust[i] = isp * mdot * G0
            else:
                mass[i] = cfg.mass_dry
        data['_ref_mass'] = mass
        data['_ref_thrust'] = thrust
    
    return pd.DataFrame(data)


def save_signature_data(df: pd.DataFrame, output_dir: str,
                        file_format: str = 'both') -> Tuple[str, str]:
    """
    시그니처 데이터를 파일로 저장
    
    Args:
        df: 시그니처 DataFrame
        output_dir: 출력 디렉토리
        file_format: 'csv', 'npz', 'both'
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    missile_type = df['missile_type'].iloc[0]
    elevation = int(df['elevation_deg'].iloc[0])
    base_name = f"{missile_type}_{elevation}deg_dataset"
    
    csv_path = None
    npz_path = None
    
    if file_format in ['csv', 'both']:
        csv_path = output_path / f"{base_name}.csv"
        df.to_csv(csv_path, index=False)
    
    if file_format in ['npz', 'both']:
        npz_path = output_path / f"{base_name}.npz"
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        arrays = {col: df[col].values for col in numeric_cols}
        arrays['missile_type'] = np.array([missile_type])
        arrays['elevation_deg'] = np.array([elevation])
        np.savez_compressed(npz_path, **arrays)
    
    return str(csv_path) if csv_path else None, str(npz_path) if npz_path else None


def run_batch_simulation(missile_type: str,
                         elevation_start: float = 15,
                         elevation_end: float = 80,
                         elevation_step: float = 5,
                         output_dir: str = None,
                         file_format: str = 'npz',
                         verbose: bool = True) -> List[pd.DataFrame]:
    """
    배치 시뮬레이션 수행
    
    Args:
        missile_type: 미사일 기종명
        elevation_start: 시작 발사각 [deg]
        elevation_end: 종료 발사각 [deg]
        elevation_step: 발사각 간격 [deg]
        output_dir: 출력 디렉토리
        file_format: 저장 형식 ('csv', 'npz', 'both', 'none')
        verbose: 상세 출력 여부
    """
    if missile_type not in MISSILES:
        raise ValueError(f"Unknown missile type: {missile_type}")
    
    if output_dir is None:
        output_dir = Path(__file__).parent / "signature_datasets" / missile_type
    else:
        output_dir = Path(output_dir) / missile_type
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"배치 시뮬레이션: {missile_type}")
        print(f"발사각: {elevation_start}° ~ {elevation_end}° (간격: {elevation_step}°)")
        print(f"{'='*60}")
    
    elevations = np.arange(elevation_start, elevation_end + elevation_step, elevation_step)
    results = []
    
    for i, elev in enumerate(elevations):
        if verbose:
            print(f"\n[{i+1}/{len(elevations)}] 발사각: {elev}°")
        
        try:
            sim = True6DOFSimulator(missile_type)
            sim_result = sim.simulate(elevation_deg=elev)
            
            df = extract_results(sim_result, missile_type, elev)
            
            if file_format != 'none':
                save_signature_data(df, str(output_dir), file_format)
            
            results.append(df)
            
        except Exception as e:
            print(f"  ✗ 오류: {e}")
            continue
    
    if verbose:
        print(f"\n배치 시뮬레이션 완료: {len(results)}/{len(elevations)} 성공")
    
    return results


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Test all missiles
    results = {}
    
    for missile in ["SCUD-B", "Nodong", "KN-23"]:
        sim = True6DOFSimulator(missile)
        results[missile] = sim.simulate(elevation_deg=45)
    
    # Plot results
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    colors = {'SCUD-B': 'b', 'Nodong': 'r', 'KN-23': 'g'}
    
    for missile, r in results.items():
        c = colors[missile]
        
        # Trajectory
        axes[0,0].plot(r['x']/1000, r['z']/1000, c, label=missile)
        
        # Velocity
        axes[0,1].plot(r['time'], r['V'], c, label=missile)
        
        # Pitch angle
        axes[0,2].plot(r['time'], np.array(r['theta'])*180/np.pi, c, label=missile)
        
        # Pitch rate (for FFT)
        axes[1,0].plot(r['time'], np.array(r['q'])*180/np.pi, c, label=missile)
        
        # Roll rate
        axes[1,1].plot(r['time'], np.array(r['p'])*180/np.pi, c, label=missile)
        
        # Yaw rate
        axes[1,2].plot(r['time'], np.array(r['r'])*180/np.pi, c, label=missile)
    
    axes[0,0].set_xlabel('Downrange (km)')
    axes[0,0].set_ylabel('Altitude (km)')
    axes[0,0].legend()
    axes[0,0].grid(True)
    axes[0,0].set_title('Trajectory')
    
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Velocity (m/s)')
    axes[0,1].legend()
    axes[0,1].grid(True)
    axes[0,1].set_title('Velocity')
    
    axes[0,2].set_xlabel('Time (s)')
    axes[0,2].set_ylabel('Theta (deg)')
    axes[0,2].legend()
    axes[0,2].grid(True)
    axes[0,2].set_title('Pitch Angle')
    
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('q (deg/s)')
    axes[1,0].legend()
    axes[1,0].grid(True)
    axes[1,0].set_title('Pitch Rate (FFT signal)')
    
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('p (deg/s)')
    axes[1,1].legend()
    axes[1,1].grid(True)
    axes[1,1].set_title('Roll Rate')
    
    axes[1,2].set_xlabel('Time (s)')
    axes[1,2].set_ylabel('r (deg/s)')
    axes[1,2].legend()
    axes[1,2].grid(True)
    axes[1,2].set_title('Yaw Rate')
    
    plt.tight_layout()
    
    # Windows 호환 경로
    output_dir = Path(__file__).parent / "results_6dof"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "true_6dof_results.png"
    plt.savefig(output_path, dpi=150)
    print(f"\n✓ Plot saved: {output_path}")
    plt.show()
