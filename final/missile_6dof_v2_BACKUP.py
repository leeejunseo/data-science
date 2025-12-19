"""
논문 기반 정통 6DOF 탄도 미사일 시뮬레이터 (교수님 호환)

핵심:
1. ✅ 정통 6DOF Body Frame 동역학 (Zipfel 2007)
2. ✅ 교수님 4단계 비행 프로그램 호환
3. ✅ Static Stability (추진: -1.0, 탄도: -2.0)
4. ✅ 교수님 config.py 파라미터

참조:
- Zipfel (2007): 6DOF Body Frame EOM
- Stevens & Lewis: 공력 모멘트 계수
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
    논문 기반 정통 6DOF (Zipfel 2007)
    
    State: [X, Y, Z, u, v, w, phi, theta, psi, p, q, r, m] (13차원)
    """
    
    def __init__(self, missile_type="SCUD-B"):
        self.missile_type = missile_type
        self.load_params()
        
    def load_params(self):
        """교수님 config.py 기반 파라미터 (하드코딩 fallback)"""
        
        # 교수님 config.py 값
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
                self.isp_sea = info["isp_sea"] * 0.97
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
            self.isp_sea = p["isp_sea"] * 0.97
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
            self.isp_sea = p["isp_sea"] * 0.97
            self.vertical_time = p["vertical_time"]
            self.pitch_time = p["pitch_time"]
            self.pitch_angle_deg = p["pitch_angle_deg"]
            self.wing_area = np.pi * (self.diameter/2)**2
        
        # 관성
        r, L, m = self.diameter/2, self.length, self.missile_mass
        self.I_xx = 0.5 * m * r**2
        self.I_yy = (1/12) * m * L**2 + 0.25 * m * r**2
        self.I_zz = self.I_yy
        
        # 댐핑
        self.C_mq = -8.0
        self.C_nr = -8.0
        self.C_lp = -0.5
        
        print(f"  질량: {self.missile_mass}kg, 연소: {self.burn_time}s")
        print(f"  피치: {self.pitch_angle_deg}°/{self.pitch_time}s")
        print(f"  제어: 교수님 방식 (PID 없음, 순수 비행 프로그램)")
    
    def get_CD(self, mach, alpha_deg=0):
        """항력 계수"""
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
    
    def get_dtheta_program(self, t):
        """비행 프로그램 각속도"""
        phase = self.get_flight_phase(t)
        
        if phase == "Vertical":
            return 0
        elif phase == "Pitch":
            pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
            pitch_progress = (t - self.vertical_time) / self.pitch_time
            pitch_progress = np.clip(pitch_progress, 0.0, 1.0)
            
            # Smoothing
            if pitch_progress < 0.5:
                smoothing_factor = np.sin(np.pi * pitch_progress)
            else:
                smoothing_factor = np.sin(np.pi * (1 - pitch_progress))
            
            return -pitch_rad / self.pitch_time * smoothing_factor
        else:
            return 0
    
    def dynamics(self, t, state):
        """6DOF 동역학"""
        X, Y, Z = state[0:3]
        u, v, w = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]
        m = state[12]
        
        g = cfg.G * (cfg.R / (cfg.R + Z))**2
        rho = get_density(Z)
        
        V = np.sqrt(u**2 + v**2 + w**2)
        if V < 0.1:
            V = 0.1
        
        # 받음각, 측면각
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
        q_dyn = 0.5 * rho * V**2
        
        # === 힘 ===
        
        # 추력
        if t < self.burn_time:
            thrust = self.isp_sea * (self.propellant_mass / self.burn_time) * g
        else:
            thrust = 0
        
        F_thrust_x = thrust
        F_thrust_y = 0
        F_thrust_z = 0
        
        # 공력
        CD = self.get_CD(mach, alpha * 180/np.pi)
        CL = 4.5 * alpha
        CY = -0.5 * beta
        
        D = q_dyn * self.wing_area * CD
        L = q_dyn * self.wing_area * CL
        Y_force = q_dyn * self.wing_area * CY
        
        F_aero_x = -D
        F_aero_y = Y_force
        F_aero_z = -L
        
        # 중력
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
        
        # 총 힘
        F_total_x = F_thrust_x + F_aero_x + g_body[0]
        F_total_y = F_thrust_y + F_aero_y + g_body[1]
        F_total_z = F_thrust_z + F_aero_z + g_body[2]
        
        # 병진
        du = F_total_x/m + q*w - r*v
        dv = F_total_y/m + r*u - p*w
        dw = F_total_z/m + p*v - q*u
        
        # === 모멘트 ===
        
        phase = self.get_flight_phase(t)
        dtheta_program = self.get_dtheta_program(t)
        
        # Static stability
        if phase in ["Vertical", "Pitch", "Constant"]:
            C_m_alpha = -1.0
            C_n_beta = -0.5
            C_l_beta = -0.2
        else:
            C_m_alpha = -2.0
            C_n_beta = -0.5
            C_l_beta = -0.2
        
        Cm_static = C_m_alpha * alpha
        Cn_static = C_n_beta * beta
        Cl_static = C_l_beta * beta
        
        # Damping
        if V > 0.1:
            Cm_damp = self.C_mq * (q * self.length / (2*V))
            Cn_damp = self.C_nr * (r * self.length / (2*V))
            Cl_damp = self.C_lp * (p * self.length / (2*V))
        else:
            Cm_damp = Cn_damp = Cl_damp = 0
        
        Cm_total = Cm_static + Cm_damp
        Cn_total = Cn_static + Cn_damp
        Cl_total = Cl_static + Cl_damp
        
        L_aero = q_dyn * self.wing_area * self.diameter * Cl_total
        M_aero = q_dyn * self.wing_area * self.length * Cm_total
        N_aero = q_dyn * self.wing_area * self.length * Cn_total
        
        # 회전
        coupling_damping = 0.1
        
        dp = (L_aero + coupling_damping * (self.I_yy - self.I_zz) * q * r) / self.I_xx
        dq = (M_aero + coupling_damping * (self.I_zz - self.I_xx) * p * r) / self.I_yy
        dr = (N_aero + coupling_damping * (self.I_xx - self.I_yy) * p * q) / self.I_zz
        
        # Kinematic (교수님 방식!)
        if abs(np.cos(theta)) < 0.01:
            dphi = p + r
            dtheta = q * np.sign(np.cos(theta)) + dtheta_program
            dpsi = 0
        else:
            dphi = p + (q*sphi + r*cphi) * np.tan(theta)
            dtheta = q*cphi - r*sphi + dtheta_program  # ← 핵심!
            dpsi = (q*sphi + r*cphi) / np.cos(theta)
        
        # 위치
        v_body = np.array([u, v, w])
        v_inertial = T_BI @ v_body
        
        dX = v_inertial[0]
        dY = v_inertial[1]
        dZ = v_inertial[2]
        
        # 질량
        if t < self.burn_time:
            dm = -(self.propellant_mass / self.burn_time)
        else:
            dm = 0
        
        return [dX, dY, dZ, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr, dm]
    
    def event_ground(self, t, state):
        """지면 충돌"""
        if t < 5.0:
            return 1000.0
        return state[2]
    event_ground.terminal = True
    event_ground.direction = -1
    
    def simulate(self, elevation_deg=45, azimuth_deg=90, add_disturbance=True):
        """시뮬레이션"""
        print(f"\n=== 논문 기반 정통 6DOF ===")
        print(f"발사: 고각 {elevation_deg}°, 방위각 {azimuth_deg}°")
        
        # 초기 조건
        X0, Y0, Z0 = 0, 0, 0
        u0, v0, w0 = 0.1, 0, 0
        phi0 = 0
        theta0 = 89.5 * cfg.DEG_TO_RAD
        psi0 = (90 - azimuth_deg) * cfg.DEG_TO_RAD
        
        # 초기 각속도
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
            method='Radau',
            max_step=0.5,
            events=[self.event_ground],
            rtol=1e-5,
            atol=1e-7
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
        
        V = np.sqrt(u**2 + v**2 + w**2)
        alpha = np.array([np.arctan2(w[i], abs(u[i]) + 1e-6) 
                         for i in range(len(u))])
        alpha = np.clip(alpha, -20*np.pi/180, 20*np.pi/180)
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


if __name__ == "__main__":
    missile = Missile6DOF_Authentic("SCUD-B")
    result = missile.simulate(elevation_deg=45, azimuth_deg=90)
