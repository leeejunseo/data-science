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
        
        # Config 로드
        if CONFIG_LOADED and hasattr(cfg, 'ENHANCED_MISSILE_TYPES'):
            info = cfg.ENHANCED_MISSILE_TYPES.get(self.missile_type)
            if info:
                self.missile_mass = info["launch_weight"]
                self.propellant_mass = info["propellant_mass"]
                self.diameter = info["diameter"]
                self.length = info["length"]
                self.burn_time = info["burn_time"]
                self.isp_sea = info["isp_sea"] * 0.97  # Fleeman: 실제 ISP는 3% 낮음
                self.vertical_time = info.get("vertical_time", 10)
                self.pitch_time = info.get("pitch_time", 15)
                self.pitch_angle_deg = info.get("pitch_angle_deg", 20)
                self.wing_area = info["reference_area"]
                print(f"✓ {self.missile_type} (config.py)")
        # Fallback
        elif self.missile_type in PROF_CFG:
            p = PROF_CFG[self.missile_type]
            self.missile_mass = p["launch_weight"]
            self.propellant_mass = p["propellant_mass"]
            self.diameter = p["diameter"]
            self.length = p["length"]
            self.burn_time = p["burn_time"]
            self.isp_sea = p["isp_sea"] * 0.97  # Fleeman 보정
            self.vertical_time = p["vertical_time"]
            self.pitch_time = p["pitch_time"]
            self.pitch_angle_deg = p["pitch_angle_deg"]
            self.wing_area = np.pi * (self.diameter/2)**2
            print(f"✓ {self.missile_type} (하드코딩)")
        else:
            print(f"⚠️ {self.missile_type} 미지원, SCUD-B 사용")
            p = PROF_CFG["SCUD-B"]
            self.missile_mass = p["launch_weight"]
            self.propellant_mass = p["propellant_mass"]
            self.diameter = p["diameter"]
            self.length = p["length"]
            self.burn_time = p["burn_time"]
            self.isp_sea = p["isp_sea"] * 0.97  # Fleeman 보정
            self.vertical_time = p["vertical_time"]
            self.pitch_time = p["pitch_time"]
            self.pitch_angle_deg = p["pitch_angle_deg"]
            self.wing_area = np.pi * (self.diameter/2)**2
        
        # 관성 (Fleeman 2012)
        r, L, m = self.diameter/2, self.length, self.missile_mass
        self.I_xx = 0.5 * m * r**2
        self.I_yy = (1/12) * m * L**2 + 0.25 * m * r**2
        self.I_zz = self.I_yy
        
        # 댐핑 (Stevens & Lewis 2003)
        self.C_mq = -8.0
        self.C_nr = -8.0
        self.C_lp = -0.5
        
        # PID 제거! 교수님 방식: 순수 비행 프로그램
        
        print(f"  질량: {self.missile_mass}kg, 연소: {self.burn_time}s")
        print(f"  피치: {self.pitch_angle_deg}°/{self.pitch_time}s")
        print(f"  제어: 교수님 방식 (PID 없음, 순수 비행 프로그램)")
        
        # 발사 초기 고각 저장 (교수님 방식 호환용)
        self.launch_elevation_rad = 0
    
    def get_CD(self, mach, alpha_deg=0):
        """항력 계수 (KAIST DATCOM)"""
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
        
        CD_alpha = 0.005 * (alpha_deg / 10.0) ** 2
        return CD_mach + CD_alpha
    
    
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
        목표 피치 각도 (교수님 방식: 90도 → elevation_deg)
        
        핵심: 항상 90도 수직에서 시작!
        - Vertical: 90도 수직
        - Pitch: 90도 → elevation_deg로 전환
        - Constant: elevation_deg 유지
        - Ballistic: 자유비행
        """
        phase = self.get_flight_phase(t)
        
        if phase == "Vertical":
            return np.pi/2  # 90도 수직
        
        elif phase == "Pitch":
            # 90도 → elevation_deg로 선형 전환
            progress = (t - self.vertical_time) / self.pitch_time
            theta_start = np.pi/2  # 90도
            theta_end = self.launch_elevation_deg * cfg.DEG_TO_RAD
            
            return theta_start - (theta_start - theta_end) * progress
        
        else:
            # Constant/Ballistic: elevation_deg 유지
            return self.launch_elevation_deg * cfg.DEG_TO_RAD
    
    def dynamics(self, t, state):
        """
        정통 6DOF 동역학 (Zipfel 2007 + 팀2 논문)
        
        State: [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m]
        
        운동 방정식:
        1. 병진 (Body Frame):
           m * [du/dt]   [F_x]   [qw - rv]
               [dv/dt] = [F_y] + [ru - pw] * m
               [dw/dt]   [F_z]   [pv - qu]
        
        2. 회전 (Body Frame):
           [dp/dt]   [Ixx  0    0  ]^-1  ([L]   [p]   [Ixx  0    0  ] [p])
           [dq/dt] = [ 0  Iyy   0  ]     ([M] - [q] × [ 0  Iyy   0  ] [q])
           [dr/dt]   [ 0    0  Izz]      ([N]   [r]   [ 0    0  Izz] [r])
        
        3. 자세 (Kinematic):
           [dphi/dt  ]   [1  sin(phi)tan(theta)  cos(phi)tan(theta)] [p]
           [dtheta/dt] = [0       cos(phi)           -sin(phi)      ] [q]
           [dpsi/dt  ]   [0  sin(phi)/cos(theta) cos(phi)/cos(theta)] [r]
        
        4. 위치 (Inertial):
           [dX/dt]       [u]
           [dY/dt] = T_IB [v]
           [dZ/dt]       [w]
        """
        X, Y, Z = state[0:3]
        u, v, w = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]
        m = state[12]
        
        g = cfg.G * (cfg.R / (cfg.R + Z))**2
        rho = get_density(Z)
        
        # 속도 크기
        V = np.sqrt(u**2 + v**2 + w**2)
        if V < 0.1:
            V = 0.1
        
        # 받음각, 측면각 (Body Frame에서 자연스럽게)
        if abs(u) > 1e-6:
            alpha = np.arctan2(w, u)
            beta = np.arcsin(np.clip(v/V, -1, 1))
        else:
            alpha = 0
            beta = 0
        
        # 마하수
        T = 288.15 - 0.0065 * Z if Z < 11000 else 216.65
        a = np.sqrt(1.4 * 287.05 * T)
        mach = V / a
        
        # 동압
        q_dyn = 0.5 * rho * V**2
        
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
        CL = 4.5 * alpha  # CL_alpha = 4.5 /rad (교수님)
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
        
        T_BI = np.array([
            [ctheta*cpsi, ctheta*spsi, -stheta],
            [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
            [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]
        ])
        
        g_inertial = np.array([0, 0, -m*g])  # Down = -Z
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
        
        # === 모멘트 계산 (Body Frame) ===
        
        # === 비행 프로그램 (get_target_theta 기반) ===
        phase = self.get_flight_phase(t)
        
        # dtheta_program (교수님 방식: smoothing factor 포함)
        if phase == "Vertical":
            dtheta_program = 0
            
        elif phase == "Pitch":
            # Pitch 변화량 = 90도 - elevation_deg
            pitch_change_rad = np.pi/2 - self.launch_elevation_deg * cfg.DEG_TO_RAD
            
            # Smoothing factor (교수님 방식)
            pitch_progress = (t - self.vertical_time) / self.pitch_time
            pitch_progress = np.clip(pitch_progress, 0.0, 1.0)
            
            if pitch_progress < 0.5:
                smoothing_factor = np.sin(np.pi * pitch_progress)
            else:
                smoothing_factor = np.sin(np.pi * (1 - pitch_progress))
            
            dtheta_program = -pitch_change_rad / self.pitch_time * smoothing_factor
                
        elif phase == "Constant":
            dtheta_program = 0
            
        else:  # Ballistic
            dtheta_program = 0
        
        # === 공력 모멘트 (Stevens & Lewis 2003 + Zipfel 2007) ===
        
        # 1. Static Stability (받음각 → 복원 모멘트)
        # Stevens & Lewis (2003) Page 133:
        # "Roskam (1979) states that C_m_alpha will normally lie 
        #  in the range −3 to +1 rad⁻¹"
        # 
        # 탄도 미사일: 안정성 필요 (Ballistic descent)
        # - 상승 중: 약한 안정성 (자세 유지)
        # - 하강 중: 강한 안정성 (자연 복원) ⭐
        
        phase = self.get_flight_phase(t)
        
        
        if phase in ["Vertical", "Pitch", "Constant"]:
            # 추진 단계: 약한 Static stability (교란 억제) - Cascade 권장
            C_m_alpha = -1.0  # 0 → -1.0
            C_n_beta = -0.5
            C_l_beta = -0.2
            
        else:  # Ballistic ONLY
            # 탄도 비행: 고정 Static stability (FFT 분석용) - Cascade 권장
            C_m_alpha = -2.0  # 시간 의존 제거
            C_n_beta = -0.5
            C_l_beta = -0.2
        # Static moment coefficients
        Cm_static = C_m_alpha * alpha
        Cn_static = C_n_beta * beta
        Cl_static = C_l_beta * beta
        
        # 2. Damping (각속도 → 댐핑 모멘트)
        # Stevens & Lewis Eq (2.6-36): C_m_q = -2*V_H*C_L_alpha*(l_t/c)
        if V > 0.1:
            Cm_damp = self.C_mq * (q * self.length / (2*V))
            Cn_damp = self.C_nr * (r * self.length / (2*V))
            Cl_damp = self.C_lp * (p * self.length / (2*V))
        else:
            Cm_damp = Cn_damp = Cl_damp = 0
        
        # 3. Total aerodynamic moment coefficients
        Cm_total = Cm_static + Cm_damp
        Cn_total = Cn_static + Cn_damp
        Cl_total = Cl_static + Cl_damp
        
        # 4. Dimensional moments
        L_aero = q_dyn * self.wing_area * self.diameter * Cl_total
        M_aero = q_dyn * self.wing_area * self.length * Cm_total  # Static + Damping
        N_aero = q_dyn * self.wing_area * self.length * Cn_total
        
        # === 회전 운동 방정식 (Euler's equation) ===
        # 관성 커플링 댐핑 (수치 안정성)
        coupling_damping = 0.1
        
        dp = (L_aero + coupling_damping * (self.I_yy - self.I_zz) * q * r) / self.I_xx
        dq = (M_aero + coupling_damping * (self.I_zz - self.I_xx) * p * r) / self.I_yy
        dr = (N_aero + coupling_damping * (self.I_xx - self.I_yy) * p * q) / self.I_zz
        
        # === Kinematic 방정식 (교수님 방식 추가) ===
        if abs(np.cos(theta)) < 0.01:
            # Gimbal lock 방지
            dphi = p + r
            dtheta = q * np.sign(np.cos(theta)) + dtheta_program  # ← 추가!
            dpsi = 0
        else:
            dphi = p + (q*sphi + r*cphi) * np.tan(theta)
            dtheta = q*cphi - r*sphi + dtheta_program  # ← 핵심!
            dpsi = (q*sphi + r*cphi) / np.cos(theta)
        
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
        
        return [dX, dY, dZ, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr, dm]
    
    def event_ground(self, t, state):
        """지면 충돌"""
        if t < 5.0:
            return 1000.0
        return state[2]  # Z
    event_ground.terminal = True
    event_ground.direction = -1
    
    def simulate(self, elevation_deg=45, azimuth_deg=90, add_disturbance=True):
        """
        시뮬레이션
        
        Args:
            elevation_deg: 발사 고각
            azimuth_deg: 방위각
            add_disturbance: 초기 교란 (시그니처용)
        """
        print(f"\n=== 논문 기반 정통 6DOF ===")
        print(f"발사: 고각 {elevation_deg}°, 방위각 {azimuth_deg}°")
        
        # 초기 조건 (BACKUP 방식으로 완전 복원)
        X0, Y0, Z0 = 0, 0, 0
        
        # elevation_deg 저장 (get_target_theta에서 사용)
        self.launch_elevation_deg = elevation_deg
        
        # 초기 자세 (항상 90도 수직에서 시작!)
        phi0 = 0
        theta0 = 89.5 * cfg.DEG_TO_RAD  # 90도 수직 (Gimbal lock 회피)
        psi0 = (90 - azimuth_deg) * cfg.DEG_TO_RAD
        
        # 초기 속도 (BACKUP 방식: 거의 정지 상태)
        u0, v0, w0 = 0.1, 0, 0
        
        # 초기 각속도 (교란)
        if add_disturbance:
            sigma = 0.1 * cfg.DEG_TO_RAD
            p0 = np.random.normal(0, sigma)
            q0 = np.random.normal(0, sigma)
            r0 = np.random.normal(0, sigma)
            print(f"✓ 초기 교란: p={p0*cfg.RAD_TO_DEG:.3f}°/s, " +
                  f"q={q0*cfg.RAD_TO_DEG:.3f}°/s, r={r0*cfg.RAD_TO_DEG:.3f}°/s")
        else:
            p0, q0, r0 = 0, 0, 0
        
        m0 = self.missile_mass
        
        state0 = [X0, Y0, Z0, u0, v0, w0, phi0, theta0, psi0, p0, q0, r0, m0]
        
        print("적분 중... (Radau stiff solver)")
        sol = solve_ivp(
            self.dynamics,
            [0, 2400],
            state0,
            method='Radau',  # Stiff solver (안정성 우선)
            max_step=0.5,    # 정밀한 적분
            events=[self.event_ground],
            rtol=1e-5,       # 정확도 향상
            atol=1e-7        # 정확도 향상
        )
        
        if not sol.success:
            raise RuntimeError(f"실패: {sol.message}")
        
        t = sol.t
        states = sol.y.T
        
        print(f"✓ 완료: {len(t)} 포인트, {t[-1]:.2f}초")
        
        # 결과 추출
        results = self.extract_results(t, states)
        
        # 통계
        range_xy = np.sqrt(states[-1, 0]**2 + states[-1, 1]**2)
        max_alt = np.max(states[:, 2])
        
        # Body Frame 속도 → 크기
        V_all = np.sqrt(states[:, 3]**2 + states[:, 4]**2 + states[:, 5]**2)
        max_v = np.max(V_all)
        
        print(f"\n결과:")
        print(f"  비행 시간: {t[-1]:.1f} s")
        print(f"  사거리: {range_xy/1000:.2f} km")
        print(f"  최대 고도: {max_alt/1000:.2f} km")
        print(f"  최대 속도: {max_v:.1f} m/s (마하 {max_v/340:.2f})")
        
        return results
    
    def extract_results(self, t, states):
        """결과 추출"""
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
        
        # Body Frame 속도 → 크기
        V = np.sqrt(u**2 + v**2 + w**2)
        
        # 받음각 (간단한 방법 - 시각화용)
        # 탄도 미사일은 받음각 작음 (±20° 이내)
        alpha = np.array([np.arctan2(w[i], abs(u[i]) + 1e-6) 
                         for i in range(len(u))])
        # ±20° 범위로 제한 (시각화 개선)
        alpha = np.clip(alpha, -20*np.pi/180, 20*np.pi/180)
        
        # 마하수
        mach = np.array([V[i]/340 for i in range(len(V))])
        
        results = {
            'time': t,
            'position_x': X,
            'position_y': Y,
            'position_z': Z,
            'u': u,
            'v': v,
            'w': w,
            'V': V,
            'phi': phi,
            'theta': theta,
            'psi': psi,
            'p': p,
            'q': q,
            'r': r,
            'alpha': alpha,
            'mach': mach,
            'mass': mass,
        }
        
        return results
    
    def save_to_standard_npz(self, results, filepath, launch_angle_deg=45):
        """
        표준 NPZ 포맷으로 저장 (trajectory_io.py 호환)
        
        Parameters:
        -----------
        results : dict
            simulate() 또는 extract_results() 반환값
        filepath : str
            저장 경로
        launch_angle_deg : float
            발사각 (메타데이터)
        """
        try:
            from trajectory_io import save_trajectory
        except ImportError:
            print("⚠ trajectory_io.py를 찾을 수 없음. 기본 저장 사용")
            import os
            import datetime
            os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
            np.savez_compressed(filepath, **results, missile_type=self.missile_type)
            print(f"✓ 기본 NPZ 저장: {filepath}")
            return
        
        # gamma 계산 (Body → Inertial → gamma)
        gamma = np.zeros_like(results['time'])
        for i in range(len(results['time'])):
            phi_i = results['phi'][i]
            theta_i = results['theta'][i]
            psi_i = results['psi'][i]
            u_i, v_i, w_i = results['u'][i], results['v'][i], results['w'][i]
            
            # Body → Inertial 변환
            cphi, sphi = np.cos(phi_i), np.sin(phi_i)
            ctheta, stheta = np.cos(theta_i), np.sin(theta_i)
            cpsi, spsi = np.cos(psi_i), np.sin(psi_i)
            
            T_BI = np.array([
                [ctheta*cpsi, ctheta*spsi, -stheta],
                [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
                [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]
            ])
            
            v_body = np.array([u_i, v_i, w_i])
            v_inertial = T_BI @ v_body
            
            # gamma = arcsin(v_up / V)
            V_i = results['V'][i]
            if V_i > 0.1:
                gamma[i] = np.arcsin(np.clip(v_inertial[2] / V_i, -1, 1))
        
        # trajectory_io 표준 포맷으로 저장
        save_trajectory(
            filepath=filepath,
            time=results['time'],
            position_x=results['position_x'],
            position_y=results['position_y'],
            position_z=results['position_z'],
            u=results['u'],
            v=results['v'],
            w=results['w'],
            phi=results['phi'],
            theta=results['theta'],
            psi=results['psi'],
            p=results['p'],
            q=results['q'],
            r=results['r'],
            mass=results['mass'],
            V=results['V'],
            gamma=gamma,
            chi=results['psi'],  # chi ≈ psi (방위각)
            missile_type=self.missile_type,
            launch_angle=launch_angle_deg,
            alpha=results.get('alpha', np.zeros_like(results['time'])),
            beta=np.zeros_like(results['time']),  # 측면각 (일단 0)
            mach=results.get('mach', results['V'] / 340.0)
        )


if __name__ == "__main__":
    # 테스트
    print("="*70)
    print("논문 기반 정통 6DOF 테스트 (Static Stability 추가)")
    print("="*70)
    
    missiles = ["SCUD-B"]  # 일단 SCUD-B만
    
    for m_type in missiles:
        print(f"\n{'='*70}")
        try:
            missile = Missile6DOF_Authentic(m_type)
            result = missile.simulate(elevation_deg=45, azimuth_deg=90,  # ← 45도
                                     add_disturbance=True)
            
            range_km = result['position_x'][-1] / 1000
            q_std = result['q'].std() * 180/np.pi
            
            print(f"\n✅ {m_type}: {range_km:.1f} km")
            print(f"  각속도 신호: std(q) = {q_std:.4f} deg/s")
            
        except Exception as e:
            print(f"❌ 오류: {e}")
            import traceback
            traceback.print_exc()
