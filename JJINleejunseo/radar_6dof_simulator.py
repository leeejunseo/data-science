"""
레이더 관측 가능 6DOF 탄도 미사일 시뮬레이터

참고 문헌:
- Fleeman (2012): Tactical Missile Design
- Zipfel (2007): Modeling and Simulation of Aerospace Vehicle Dynamics
- Stevens & Lewis (2015): Aircraft Control and Simulation

상태 벡터: 12차원 [X, Y, Z, Vx, Vy, Vz, phi, theta, psi, p, q, r]
- 연료 질량, 추력 제외 (교수님 요구사항)
- 레이더 관측 가능 변수만 포함

좌표계:
- 관성 좌표계: [X_동쪽, Y_북쪽, Z_상승] (m)
- 동체 좌표계: [u, v, w] (m/s)
- 오일러각: [phi_롤, theta_피치, psi_요] (rad)
- 3-2-1 회전 순서 (요-피치-롤) Zipfel 식 3.14 참조
"""

import numpy as np
from typing import Tuple, Dict
from scipy.integrate import solve_ivp


def direction_cosine_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    3-2-1 오일러 방향 코사인 행렬(DCM) 계산
    
    동체 좌표계에서 관성 좌표계로 변환
    참조: Zipfel (2007) 식 3.14
    
    회전 순서: 요(psi) -> 피치(theta) -> 롤(phi)
    
    Args:
        phi: 롤 각도 (rad)
        theta: 피치 각도 (rad)
        psi: 요 각도 (rad)
    
    Returns:
        DCM: 3x3 회전 행렬 [동체 -> 관성]
    """
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    
    DCM = np.array([
        [cth*cpsi, cth*spsi, -sth],
        [sphi*sth*cpsi - cphi*spsi, sphi*sth*spsi + cphi*cpsi, sphi*cth],
        [cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi - sphi*cpsi, cphi*cth]
    ])
    
    return DCM


def atmospheric_density_isa1976(altitude: float) -> float:
    """
    ISA 1976 표준 대기 모델
    
    Args:
        altitude: 기하학적 고도 (m)
    
    Returns:
        rho: 공기 밀도 (kg/m³)
    """
    if altitude < 0:
        altitude = 0
    
    if altitude < 11000:
        T = 288.15 - 0.0065 * altitude
        p = 101325 * (T / 288.15) ** 5.2561
    elif altitude < 25000:
        T = 216.65
        p = 22632.1 * np.exp(-0.00015768 * (altitude - 11000))
    else:
        T = 216.65 + 0.003 * (altitude - 25000)
        p = 2488.7 * (T / 216.65) ** (-11.388)
    
    rho = p / (287.05 * T)
    
    return rho


def euler_angle_rates(phi: float, theta: float, psi: float, 
                     p: float, q: float, r: float) -> Tuple[float, float, float]:
    """
    동체 각속도를 오일러 각속도로 변환
    
    참조: Zipfel (2007) 식 3.24
    
    Args:
        phi: 롤 각도 (rad)
        theta: 피치 각도 (rad)
        psi: 요 각도 (rad)
        p: 롤 각속도 (rad/s)
        q: 피치 각속도 (rad/s)
        r: 요 각속도 (rad/s)
    
    Returns:
        phi_dot, theta_dot, psi_dot (rad/s)
    """
    sphi = np.sin(phi)
    cphi = np.cos(phi)
    cth = np.cos(theta)
    tth = np.tan(theta)
    
    # 짐벌락(Gimbal lock) 방지
    if abs(cth) < 0.01:
        cth = 0.01 * np.sign(cth)
    
    phi_dot = p + q * sphi * tth + r * cphi * tth
    theta_dot = q * cphi - r * sphi
    psi_dot = (q * sphi + r * cphi) / cth
    
    return phi_dot, theta_dot, psi_dot


class Radar6DOFSimulator:
    """
    레이더 관측 가능 상태 기반 6DOF 탄도 미사일 시뮬레이터
    
    특징:
    - 12차원 상태 벡터 (연료 질량, 추력 제외)
    - 변수 제어 매개변수 (교수님 명세)
    - 상수 평균 질량 및 관성
    - 단순 상수 항력 계수
    """
    
    MISSILE_PARAMS = {
        "SCUD-B": {
            "mass_avg": 3422.5,
            "I_xx": 620,
            "I_yy": 70000,
            "I_zz": 70000,
            "length": 11.25,
            "diameter": 0.88,
            "C_D": 0.30,
            "C_m_alpha": -0.15,
            "C_m_q": -0.80,
            "C_n_r": -0.80,
            "C_l_p": -0.50,
            "propellant_mass": 5000,
            "burn_time": 80,
            "isp_sea": 225,
            "isp_vacuum": 248,
        },
        "KN-23": {
            "mass_avg": 1750,
            "I_xx": 500,
            "I_yy": 12000,
            "I_zz": 12000,
            "length": 7.5,
            "diameter": 0.65,
            "C_D": 0.25,
            "C_m_alpha": -0.20,
            "C_m_q": -0.85,
            "C_n_r": -0.85,
            "C_l_p": -0.55,
            "propellant_mass": 2500,
            "burn_time": 60,
            "isp_sea": 240,
            "isp_vacuum": 264,
        },
        "Nodong": {
            "mass_avg": 6000,
            "I_xx": 1200,
            "I_yy": 170000,
            "I_zz": 170000,
            "length": 16.0,
            "diameter": 1.25,
            "C_D": 0.35,
            "C_m_alpha": -0.13,
            "C_m_q": -0.75,
            "C_n_r": -0.75,
            "C_l_p": -0.45,
            "propellant_mass": 8000,
            "burn_time": 110,
            "isp_sea": 220,
            "isp_vacuum": 242,
        }
    }
    
    def __init__(self, missile_type: str = "SCUD-B"):
        """
        제어된 매개변수로 시뮬레이터 초기화
        
        Args:
            missile_type: ["SCUD-B", "KN-23", "Nodong"] 중 하나
        """
        assert missile_type in self.MISSILE_PARAMS, f"알 수 없는 미사일: {missile_type}"
        
        self.missile_type = missile_type
        self.params = self.MISSILE_PARAMS[missile_type].copy()
        
        self.mass = self.params["mass_avg"]
        self.I_xx = self.params["I_xx"]
        self.I_yy = self.params["I_yy"]
        self.I_zz = self.params["I_zz"]
        self.diameter = self.params["diameter"]
        self.length = self.params["length"]
        self.S_ref = np.pi * (self.diameter / 2) ** 2
        self.c_bar = self.length / 2
        
        self.C_D = self.params["C_D"]
        self.C_m_alpha = self.params["C_m_alpha"]
        self.C_m_q = self.params["C_m_q"]
        self.C_n_r = self.params["C_n_r"]
        self.C_l_p = self.params["C_l_p"]
        
        # 추력 파라미터
        self.propellant_mass = self.params["propellant_mass"]
        self.burn_time = self.params["burn_time"]
        self.isp_sea = self.params["isp_sea"]
        self.isp_vacuum = self.params["isp_vacuum"]
        self.mass_dry = self.mass - self.propellant_mass  # 건조 질량
        
        self.g = 9.80665
    
    def dynamics(self, t: float, state: np.ndarray) -> np.ndarray:
        """
        12차원 동역학 방정식
        
        상태 벡터: [X, Y, Z, Vx, Vy, Vz, phi, theta, psi, p, q, r]
        
        Args:
            t: 시간 (s)
            state: 12차원 상태 벡터
        
        Returns:
            state_dot: 12차원 미분 벡터
        """
        assert len(state) == 12, f"상태 벡터는 12차원이어야 합니다. 현재: {len(state)}차원"
        
        X, Y, Z, Vx, Vy, Vz, phi, theta, psi, p, q, r = state
        
        # 짐벌락 방지: 피치각을 ±85°로 제한 (탄도 미사일은 90° 도달 안 함)
        theta = np.clip(theta, -np.deg2rad(85), np.deg2rad(85))
        
        # 관성 좌표계 속도 벡터
        V_inertial = np.array([Vx, Vy, Vz])
        V_mag = np.linalg.norm(V_inertial)
        
        # 속도가 매우 작으면 계산 중단
        if V_mag < 1e-6:
            return np.zeros(12)
        
        # 방향 코사인 행렬 (관성 -> 동체)
        DCM = direction_cosine_matrix(phi, theta, psi)
        DCM_inv = DCM.T  # 전치 = 역행렬 (직교 행렬)
        
        # 관성 좌표계 속도를 동체 좌표계로 변환
        V_body = DCM_inv @ V_inertial
        u, v, w = V_body
        
        # 받음각(alpha)과 옆미끄럼각(beta) 계산
        alpha = np.arctan2(w, u) if abs(u) > 1e-6 else 0
        beta = np.arcsin(np.clip(v / V_mag, -1, 1)) if V_mag > 1e-6 else 0
        
        # 대기 밀도 및 동압 계산
        rho = atmospheric_density_isa1976(Z)
        q_dyn = 0.5 * rho * V_mag ** 2  # 동압 q = 0.5 * ρ * V²
        
        # 항력 계산 (동체 좌표계)
        D = q_dyn * self.S_ref * self.C_D
        F_aero_body = -D * V_body / V_mag  # 속도 반대 방향
        
        # 공력 모멘트 - 피치 (동체 좌표계)
        M_pitch = q_dyn * self.S_ref * self.c_bar * (
            self.C_m_alpha * alpha +  # 받음각에 의한 모멘트
            self.C_m_q * q * self.c_bar / (2 * V_mag) if V_mag > 1 else 0  # 감쇠 모멘트
        )
        
        # 공력 모멘트 - 요 (동체 좌표계)
        N_yaw = q_dyn * self.S_ref * self.c_bar * (
            self.C_m_alpha * beta +  # 옆미끄럼각에 의한 모멘트
            self.C_n_r * r * self.c_bar / (2 * V_mag) if V_mag > 1 else 0  # 감쇠 모멘트
        )
        
        # 공력 모멘트 - 롤 (동체 좌표계)
        L_roll = q_dyn * self.S_ref * self.diameter * (
            self.C_l_p * p * self.diameter / (2 * V_mag) if V_mag > 1 else 0  # 감쇠 모멘트
        )
        
        # 공력을 관성 좌표계로 변환
        F_aero_inertial = DCM @ F_aero_body
        
        # 현재 질량 계산 (연료 소모 고려)
        if t <= self.burn_time:
            mass_current = self.mass - (self.propellant_mass / self.burn_time) * t
        else:
            mass_current = self.mass_dry
        
        # 질량이 건조질량 이하로 떨어지지 않도록 제한
        mass_current = max(mass_current, self.mass_dry)
        
        # 추력 계산 (동체 좌표계, X축 방향)
        if t <= self.burn_time:
            # 고도에 따른 비추력 보간 (해수면 → 진공)
            altitude_factor = min(1.0, Z / 50000)  # 50km 이상에서 진공 비추력
            isp_current = self.isp_sea + (self.isp_vacuum - self.isp_sea) * altitude_factor
            
            # 추력 = ISP × 연료소모율 × g
            mdot = self.propellant_mass / self.burn_time  # 연료 소모율 (kg/s)
            T_mag = isp_current * mdot * self.g
            
            # 추력 벡터 (동체 좌표계 X축 방향)
            F_thrust_body = np.array([T_mag, 0, 0])
            # 관성 좌표계로 변환
            F_thrust_inertial = DCM @ F_thrust_body
        else:
            F_thrust_inertial = np.array([0, 0, 0])
        
        # 중력 (관성 좌표계, Z축 하향)
        F_grav = np.array([0, 0, -mass_current * self.g])
        
        # 총 힘
        F_total = F_aero_inertial + F_grav + F_thrust_inertial
        
        # 관성 좌표계 가속도 (뉴턴 제2법칙)
        accel_inertial = F_total / mass_current
        
        # 오일러 회전 방정식 (동체 좌표계 각가속도)
        p_dot = (L_roll + (self.I_yy - self.I_zz) * q * r) / self.I_xx
        q_dot = (M_pitch + (self.I_zz - self.I_xx) * p * r) / self.I_yy
        r_dot = (N_yaw + (self.I_xx - self.I_yy) * p * q) / self.I_zz
        
        # 오일러 각속도 (동체 각속도에서 변환)
        phi_dot, theta_dot, psi_dot = euler_angle_rates(phi, theta, psi, p, q, r)
        
        # 12차원 상태 미분 벡터 구성
        state_dot = np.array([
            Vx, Vy, Vz,  # 위치 미분 = 속도
            accel_inertial[0], accel_inertial[1], accel_inertial[2],  # 속도 미분 = 가속도
            phi_dot, theta_dot, psi_dot,  # 오일러각 미분
            p_dot, q_dot, r_dot  # 각속도 미분 = 각가속도
        ])
        
        return state_dot
    
    def create_initial_state(self, elevation_deg: float, azimuth_deg: float) -> np.ndarray:
        """
        주어진 발사 각도에 대한 초기 12차원 상태 생성
        
        Args:
            elevation_deg: 발사 고각 (10-85°)
            azimuth_deg: 방위각 (0-360°)
        
        Returns:
            state0: 12차원 초기 상태 벡터
        """
        assert 10 <= elevation_deg <= 85, "발사 고각은 10-85° 범위여야 합니다"
        assert 0 <= azimuth_deg <= 360, "방위각은 0-360° 범위여야 합니다"
        
        # 각도를 라디안으로 변환
        elevation = np.deg2rad(elevation_deg)
        azimuth = np.deg2rad(azimuth_deg)
        
        # 초기 속도 (m/s) - 발사대에서 낮은 속도로 시작
        V0 = 10.0  # 발사 순간 작은 초기 속도
        
        # 초기 위치 (발사대)
        X0 = 0.0
        Y0 = 0.0
        Z0 = 10.0  # 발사대 높이
        
        # 초기 속도 성분 (발사 방향)
        Vx0 = V0 * np.cos(elevation) * np.sin(azimuth)  # 동쪽
        Vy0 = V0 * np.cos(elevation) * np.cos(azimuth)  # 북쪽
        Vz0 = V0 * np.sin(elevation)  # 상승
        
        # 초기 오일러각 (발사 방향과 일치)
        phi0 = 0.0  # 롤 없음
        theta0 = elevation  # 피치 = 발사 고각
        psi0 = azimuth  # 요 = 방위각
        
        # 초기 각속도 (정지 상태)
        p0 = 0.0  # 롤 레이트
        q0 = 0.0  # 피치 레이트
        r0 = 0.0  # 요 레이트
        
        state0 = np.array([
            X0, Y0, Z0,
            Vx0, Vy0, Vz0,
            phi0, theta0, psi0,
            p0, q0, r0
        ])
        
        return state0
    
    def get_params(self) -> Dict:
        """메타데이터 저장을 위한 미사일 매개변수 반환"""
        return self.params.copy()


if __name__ == "__main__":
    print("✅ 레이더 6DOF 시뮬레이터 모듈")
    print(f"   - 상태 차원: 12D")
    print(f"   - 미사일: {list(Radar6DOFSimulator.MISSILE_PARAMS.keys())}")
    print(f"   - 좌표계: 3-2-1 오일러 (Zipfel 식 3.14)")
    print(f"   - 대기 모델: ISA 1976")
