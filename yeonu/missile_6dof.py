"""
6DOF 탄도 미사일 시뮬레이터 (변인 통제 적용)

변인 통제:
- 모든 미사일에 동일한 Isp, CD, Burn Time 적용
- 미사일별 차이: 질량, 직경, 길이만 다름
"""

import numpy as np
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


class Missile6DOF_Professor:
    """6DOF 탄도 미사일 시뮬레이터 (변인 통제)"""
    
    def __init__(self, missile_type="SCUD-B"):
        self.missile_type = missile_type
        self.load_params()
        
        self.vertical_time = 10.0
        self.pitch_time = 20.0
        self.pitch_angle_deg = 5.0
        self.constant_time = self.vertical_time + self.pitch_time + 35.0
        
    def load_params(self):
        """파라미터 로드 (변인 통제 적용)"""
        if CONFIG_LOADED and hasattr(cfg, 'ENHANCED_MISSILE_TYPES'):
            info = cfg.ENHANCED_MISSILE_TYPES.get(self.missile_type)
            if info:
                # 미사일 고유 파라미터
                self.missile_mass = info["launch_weight"]
                self.propellant_mass = info["propellant_mass"]
                self.diameter = info["diameter"]
                self.length = info["length"]
                self.wing_area = info["reference_area"]
                
                # 변인 통제 적용
                if hasattr(cfg, 'ENABLE_CONTROLLED_PHYSICS') and cfg.ENABLE_CONTROLLED_PHYSICS:
                    self.burn_time = cfg.CONTROLLED_PHYSICS["burn_time"]
                    self.isp_sea = cfg.CONTROLLED_PHYSICS["isp_sea"]
                    self.cd_base = cfg.CONTROLLED_PHYSICS["cd_base"]
                    self.cl_alpha = cfg.CONTROLLED_PHYSICS["cl_alpha"]
                    self.k_induced = cfg.CONTROLLED_PHYSICS["k_induced"]
                    print(f"✓ {self.missile_type} 로드 (변인 통제 활성화)")
                else:
                    self.burn_time = info["burn_time"]
                    self.isp_sea = info["isp_sea"]
                    self.cd_base = info.get("cd_base", 0.25)
                    self.cl_alpha = 4.5
                    self.k_induced = 0.01
                    print(f"✓ {self.missile_type} 로드")
            else:
                self.load_default()
        else:
            self.load_default()
        
        # 관성 모멘트
        r = self.diameter / 2
        L = self.length
        m = self.missile_mass
        
        self.I_xx = 0.5 * m * r**2
        self.I_yy = (1/12) * m * L**2 + 0.25 * m * r**2
        self.I_zz = self.I_yy
        
        self.C_mq = -8.0
        self.C_nr = -8.0
        self.C_lp = -0.5
        
        print(f"  질량: {self.missile_mass} kg")
        print(f"  I_yy/I_zz: {self.I_yy/self.I_zz:.3f}")
        
        if CONFIG_LOADED and hasattr(cfg, 'ENABLE_CONTROLLED_PHYSICS') and cfg.ENABLE_CONTROLLED_PHYSICS:
            print(f"  [변인 통제] Isp={self.isp_sea}s, Burn={self.burn_time}s, CD={self.cd_base}")
        
    def load_default(self):
        """기본값"""
        self.missile_mass = 5860
        self.propellant_mass = 4875
        self.diameter = 0.88
        self.length = 10.94
        self.wing_area = np.pi * (self.diameter/2)**2
        
        if CONFIG_LOADED and hasattr(cfg, 'ENABLE_CONTROLLED_PHYSICS') and cfg.ENABLE_CONTROLLED_PHYSICS:
            self.burn_time = cfg.CONTROLLED_PHYSICS["burn_time"]
            self.isp_sea = cfg.CONTROLLED_PHYSICS["isp_sea"]
            self.cd_base = cfg.CONTROLLED_PHYSICS["cd_base"]
            self.cl_alpha = cfg.CONTROLLED_PHYSICS["cl_alpha"]
            self.k_induced = cfg.CONTROLLED_PHYSICS["k_induced"]
            print("✓ 기본값 로드 (SCUD-B, 변인 통제)")
        else:
            self.burn_time = 65
            self.isp_sea = 230
            self.cd_base = 0.25
            self.cl_alpha = 4.5
            self.k_induced = 0.01
            print("✓ 기본값 로드 (SCUD-B)")
    
    def get_CD(self, mach, alpha_deg=0):
        """항력 계수"""
        CD_base = self.cd_base
        CD_alpha = 0.005 * (alpha_deg / 10.0) ** 2
        return CD_base + CD_alpha
    
    def get_CL(self, alpha_rad):
        """양력 계수"""
        return self.cl_alpha * alpha_rad
    
    def dynamics(self, t, state):
        """6DOF 동역학"""
        V, gamma, psi, x, y, h, phi, theta, p, q_rate, r, m = state
        
        m_safe = max(m, 100.0)
        V_safe = max(V, 1.0)
        
        g = cfg.G * cfg.R**2 / (cfg.R + h)**2
        
        rho = get_density(h)
        temperature = max(216.65, 288.15 - 0.0065 * h)
        if h > 11000:
            temperature = 216.65
        sound_speed = 20.05 * np.sqrt(temperature)
        mach = V_safe / sound_speed
        
        q_dyn = 0.5 * rho * V_safe**2
        
        # 비행 프로그램
        if t < self.vertical_time:
            alpha = 0
        elif t < self.vertical_time + self.pitch_time:
            pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
            pitch_progress = (t - self.vertical_time) / self.pitch_time
            alpha = pitch_rad * pitch_progress
        elif t < self.constant_time:
            alpha = self.pitch_angle_deg * cfg.DEG_TO_RAD
        else:
            alpha = 0
        
        CL = self.get_CL(alpha)
        CD = self.get_CD(mach, alpha * cfg.RAD_TO_DEG)
        CD += self.k_induced * CL**2
        
        D = q_dyn * CD * self.wing_area
        L = q_dyn * CL * self.wing_area
        
        # 추력
        if t <= self.burn_time:
            mdot = self.propellant_mass / self.burn_time
            T = self.isp_sea * mdot * cfg.G
        else:
            T = 0
            mdot = 0
        
        # 운동방정식
        dV = (T - D) / m_safe - g * np.sin(gamma)
        
        if V_safe > 1.0:
            dgamma = (L * np.cos(phi) / m_safe - g * np.cos(gamma) + V_safe**2 * np.cos(gamma) / (cfg.R + h)) / V_safe
        else:
            dgamma = 0
        
        if V_safe > 1.0 and abs(np.cos(gamma)) > 0.01:
            dpsi = (L * np.sin(phi) / (m_safe * V_safe * np.cos(gamma)))
        else:
            dpsi = 0
        
        dx = V_safe * np.cos(gamma) * np.cos(psi)
        dy = V_safe * np.cos(gamma) * np.sin(psi)
        dh = V_safe * np.sin(gamma)
        
        # Euler 각도
        cos_theta = np.cos(theta)
        if abs(cos_theta) > 0.01:
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)
            tan_theta = np.tan(theta)
            
            dphi = p + (q_rate * sin_phi + r * cos_phi) * tan_theta
            dtheta = q_rate * cos_phi - r * sin_phi
            dpsi_euler = (q_rate * sin_phi + r * cos_phi) / cos_theta
        else:
            dphi = p
            dtheta = q_rate
            dpsi_euler = r
        
        # 공력 모멘트
        c_ref = self.diameter
        q_bar = q_dyn
        
        if V_safe > 1.0:
            p_hat = p * c_ref / (2 * V_safe)
            q_hat = q_rate * c_ref / (2 * V_safe)
            r_hat = r * c_ref / (2 * V_safe)
        else:
            p_hat = 0.0
            q_hat = 0.0
            r_hat = 0.0
        
        M_roll = q_bar * self.wing_area * c_ref * self.C_lp * p_hat
        M_pitch = q_bar * self.wing_area * c_ref * self.C_mq * q_hat
        M_yaw = q_bar * self.wing_area * c_ref * self.C_nr * r_hat
        
        k_roll_restore = 0.5
        M_roll_restore = -k_roll_restore * phi * q_bar * self.wing_area * c_ref
        M_roll += M_roll_restore
        
        dp = np.clip(M_roll / self.I_xx, -10.0, 10.0)
        dq = np.clip(M_pitch / self.I_yy, -10.0, 10.0)
        dr = np.clip(M_yaw / self.I_zz, -10.0, 10.0)
        
        dm = -mdot
        
        return [dV, dgamma, dpsi, dx, dy, dh, dphi, dtheta, dp, dq, dr, dm]
    
    def event_ground(self, t, state):
        """지면 충돌"""
        return state[5]
    event_ground.terminal = True
    event_ground.direction = -1
    
    def simulate(self, elevation_deg=45, azimuth_deg=90):
        """시뮬레이션"""
        print(f"\n=== 6DOF 시뮬레이션 ===")
        print(f"발사: 고각 {elevation_deg}°, 방위각 {azimuth_deg}°")
        
        V0 = 50.0  # 초기 속도 (발사대 이탈 속도)
        gamma0 = elevation_deg * cfg.DEG_TO_RAD
        psi0 = (90 - azimuth_deg) * cfg.DEG_TO_RAD
        x0, y0, h0 = 0.0, 0.0, 50.0  # 발사대 높이
        phi0 = 0.0
        theta0 = gamma0
        p0, q0, r0 = 0.0, 0.0, 0.0
        m0 = self.missile_mass
        
        state0 = [V0, gamma0, psi0, x0, y0, h0, phi0, theta0, p0, q0, r0, m0]
        
        print("적분 중...")
        sol = solve_ivp(
            self.dynamics,
            [0, 600],
            state0,
            method='RK45',
            max_step=1.0,
            events=[self.event_ground],
            rtol=1e-6,
            atol=1e-8
        )
        
        if not sol.success:
            raise RuntimeError(f"실패: {sol.message}")
        
        t = sol.t
        states = sol.y.T
        
        print(f"✓ 완료: {len(t)} 포인트, {t[-1]:.2f}초")
        
        results = self.extract_results(t, states)
        
        range_m = np.sqrt(states[-1, 3]**2 + states[-1, 4]**2)
        max_alt = np.max(states[:, 5])
        max_v = np.max(states[:, 0])
        
        print(f"\n결과:")
        print(f"  비행 시간: {t[-1]:.1f} s")
        print(f"  사거리: {range_m/1000:.2f} km")
        print(f"  최대 고도: {max_alt/1000:.2f} km")
        print(f"  최대 속도: {max_v:.1f} m/s (마하 {max_v/340:.2f})")
        
        self.calculate_performance(results)
        
        return results
    
    def extract_results(self, t, states):
        """결과 추출"""
        V = states[:, 0]
        gamma = states[:, 1]
        psi = states[:, 2]
        x = states[:, 3]
        y = states[:, 4]
        h = states[:, 5]
        phi = states[:, 6]
        theta = states[:, 7]
        p = states[:, 8]
        q = states[:, 9]
        r = states[:, 10]
        mass = states[:, 11]
        
        alpha = np.zeros_like(t)
        for i, ti in enumerate(t):
            if ti < self.vertical_time:
                alpha[i] = 0
            elif ti < self.vertical_time + self.pitch_time:
                pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
                pitch_progress = (ti - self.vertical_time) / self.pitch_time
                alpha[i] = pitch_rad * pitch_progress
            elif ti < self.constant_time:
                alpha[i] = self.pitch_angle_deg * cfg.DEG_TO_RAD
            else:
                alpha[i] = 0
        
        return {
            'time': t,
            'V': V,
            'gamma': gamma,
            'psi': psi,
            'position_x': x,
            'position_y': y,
            'altitude': h,
            'phi': phi,
            'theta': theta,
            'p': p,
            'q': q,
            'r': r,
            'mass': mass,
            'mach': V / 340,
            'alpha': alpha,
        }

    def calculate_performance(self, results):
        """성능 계산"""
        t = results['time']
        V = results['V']
        h = results['altitude']
        m = results['mass']
        
        burn_idx = np.argmin(np.abs(t - self.burn_time))
        
        if burn_idx > 0:
            V_burnout = V[burn_idx]
            h_burnout = h[burn_idx]
            t_burnout = t[burn_idx]
            
            rho = get_density(h_burnout)
            q = 0.5 * rho * V_burnout**2
            
            if t_burnout < self.vertical_time + self.pitch_time:
                pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
                pitch_progress = (t_burnout - self.vertical_time) / self.pitch_time
                alpha = pitch_rad * pitch_progress
            else:
                alpha = self.pitch_angle_deg * cfg.DEG_TO_RAD
            
            CL = self.cl_alpha * alpha
            
            mach = V_burnout / 340
            CD = self.get_CD(mach, alpha * cfg.RAD_TO_DEG)
            CD += self.k_induced * CL**2
            
            L = q * CL * self.wing_area
            D = q * CD * self.wing_area
            
            if D > 0:
                LD_ratio = L / D
                print(f"\n성능 (연소 종료 t={t_burnout:.1f}s):")
                print(f"  받음각: {alpha * cfg.RAD_TO_DEG:.2f}°")
                print(f"  CL: {CL:.3f}")
                print(f"  CD: {CD:.3f}")
                print(f"  양항비 (L/D): {LD_ratio:.3f}")
                print(f"  목표 (KAIST): 4.782")
                
                sustained_g = L / (m[burn_idx] * 9.81)
                print(f"  Sustained-g: {sustained_g:.2f}")
                print(f"  목표 (KAIST): 4.1")
    
    def save_trajectory(self, filepath, results):
        """NPZ 저장"""
        np.savez(
            filepath,
            **results,
            missile_type=self.missile_type
        )
        print(f"\n✓ NPZ 저장: {filepath}")


def main():
    """테스트"""
    import os
    import datetime
    
    missile = Missile6DOF_Professor("SCUD-B")
    results = missile.simulate(elevation_deg=45, azimuth_deg=90)
    
    os.makedirs("results_6dof", exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filepath = f"results_6dof/SCUD-B_45deg_professor_{timestamp}.npz"
    
    missile.save_trajectory(filepath, results)


if __name__ == "__main__":
    main()
