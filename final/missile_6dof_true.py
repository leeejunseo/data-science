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
from typing import Dict, Tuple, Optional
import warnings


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
        mass_total=3400,          # kg
        mass_propellant=2400,     # kg (고체 추진제)
        mass_dry=1000,            # kg
        diameter=0.95,
        length=7.5,
        reference_area=np.pi * (0.95/2)**2,
        isp_sea=250,              # s (고체)
        isp_vac=265,              # s
        burn_time=35,             # s (고체는 더 짧음)
        vertical_time=5,
        pitch_time=8,
        pitch_angle=25 * np.pi/180,
        C_m_alpha=-2.5,
        C_mq=-100.0,
        C_lp=-28.0,
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
    
    def get_flight_phase(self, t: float) -> str:
        """Determine current flight phase"""
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
                          thrust: float) -> float:
        """
        Compute TVC control moment for pitch channel
        
        Pitch program + rate damping
        Continue pitching during Gravity Turn until target elevation reached
        
        Returns: M_pitch [N·m]
        """
        if thrust < 100:
            return 0.0
        
        phase = self.get_flight_phase(t)
        target_elevation = getattr(self, 'target_elevation', np.pi/4)
        m, I_xx, I_yy, I_zz = self.get_mass_properties(t)
        
        # Feedforward pitch rate command
        if phase == "Vertical":
            q_cmd = 0.0
            
        elif phase == "Pitch":
            # Constant pitch rate to reach target
            pitch_range = np.pi/2 - target_elevation
            q_cmd = -pitch_range / self.cfg.pitch_time
            
        elif phase == "Gravity Turn":
            # Continue pitching if not yet at target elevation
            if theta > target_elevation + 0.02:  # ~1 degree tolerance
                # Slower rate during gravity turn
                q_cmd = -0.03  # ~1.7 deg/s nose-down
            else:
                q_cmd = 0.0
        else:
            return 0.0
        
        # Rate tracking: M = K * (q_cmd - q_rate)
        # K = I * omega_n where omega_n is desired bandwidth
        omega_n = 3.0  # 3 rad/s bandwidth
        K_rate = I_yy * omega_n
        
        M_tvc = K_rate * (q_cmd - q_rate)
        
        # Limit to TVC authority
        M_max = thrust * self.cfg.tvc_moment_arm * np.sin(self.cfg.tvc_max_angle)
        M_tvc = np.clip(M_tvc, -M_max, M_max)
        
        return M_tvc
    
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
        
        # 2. TVC control moment (pitch only for now)
        M_tvc = self.compute_tvc_moment(t, theta, q_rate, thrust)
        
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
        
        # Initial state - START VERTICAL, pitch over to elevation angle
        # theta = 90° = vertical in NED (body x points up = -z_ned)
        theta0 = np.pi/2  # Always start vertical
        psi0 = (90 - azimuth_deg) * np.pi/180
        
        # Store target elevation for flight program
        self.target_elevation = elevation_deg * np.pi/180
        
        # Initial quaternion (vertical)
        quat0 = quat_from_euler(0, theta0, psi0)
        
        # Initial body velocity (small forward velocity for stability)
        u0 = 1.0
        
        state0 = np.array([
            0, 0, 0,                    # Position
            u0, 0, 0,                   # Body velocity
            quat0[0], quat0[1], quat0[2], quat0[3],  # Quaternion
            0, 0, 0                     # Angular rates
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
        
        return {
            'time': t,
            'x': x, 'y': y, 'z': altitude,  # Return altitude, not NED z
            'u': u, 'v': v, 'w': w,
            'V': V,
            'phi': phi_arr, 'theta': theta_arr, 'psi': psi_arr,
            'p': p_arr, 'q': q_arr, 'r': r_arr,
            'range_km': range_km,
            'max_alt_km': max_alt_km,
            'flight_time': flight_time,
        }


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
    plt.savefig('/home/claude/true_6dof_results.png', dpi=150)
    print("\n✓ Plot saved: /home/claude/true_6dof_results.png")
