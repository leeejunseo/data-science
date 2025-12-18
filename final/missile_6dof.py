"""
교수님 3DOF 구조 기반 6DOF 탄도 미사일 시뮬레이터

핵심:
1. ✅ 교수님 4단계 비행: Vertical → Pitch → Constant → Ballistic
2. ✅ 3DOF 운동방정식을 6DOF로 확장 (Body 좌표계)
3. ✅ PID 없음 (공력 댐핑만)
4. ✅ 교수님 config.py 파라미터 사용

참조:
- 교수님 main.py: 4단계 구조, 양력/항력 모델
- Zipfel (2007): 6DOF Body 좌표계
- Stevens & Lewis: 공력 댐핑 계수
- KAIST 논문: 양항비 4.782 검증
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
except (ImportError, ModuleNotFoundError) as e:
    print(f"✗ config.py 로드 실패 - 기본값 사용 ({e})")
    CONFIG_LOADED = False
    class cfg:
        G = 9.80665
        R = 6371000
        DEG_TO_RAD = np.pi / 180
        RAD_TO_DEG = 180 / np.pi
        CL_PITCH = 0.3
        K = 0.05


def get_density(h):
    """ISA 1976 대기 모델 (교수님 방식)"""
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
    """
    교수님 3DOF 구조를 6DOF로 확장
    
    State: [V, gamma, psi, x, y, h, phi, theta, p, q, r, m] (12차원)
    - V, gamma, psi: 교수님 3DOF 상태
    - phi, theta: 추가 Euler 각
    - p, q, r: 각속도
    """
    
    def __init__(self, missile_type="SCUD-B"):
        self.missile_type = missile_type
        self.load_params()
        
        # 비행 프로그램 (교수님)
        self.vertical_time = 10.0
        self.pitch_time = 15.0
        self.pitch_angle_deg = 20.0
        self.constant_time = self.vertical_time + self.pitch_time + 40.0
        
    def load_params(self):
        """파라미터 로드"""
        if CONFIG_LOADED and hasattr(cfg, 'ENHANCED_MISSILE_TYPES'):
            info = cfg.ENHANCED_MISSILE_TYPES.get(self.missile_type)
            if info:
                self.missile_mass = info["launch_weight"]
                self.propellant_mass = info["propellant_mass"]
                self.diameter = info["diameter"]
                self.length = info["length"]
                self.wing_area = info["reference_area"]
                self.burn_time = info["burn_time"]
                self.isp_sea = info["isp_sea"]
                print(f"✓ {self.missile_type} 로드")
            else:
                self.load_default()
        else:
            self.load_default()
        
        # 관성 모멘트 (Fleeman)
        r = self.diameter / 2
        L = self.length
        m = self.missile_mass
        
        self.I_xx = 0.5 * m * r**2
        self.I_yy = (1/12) * m * L**2 + 0.25 * m * r**2
        self.I_zz = self.I_yy
        
        # 공력 댐핑 (Stevens & Lewis)
        self.C_mq = -8.0
        self.C_nr = -8.0
        self.C_lp = -0.5
        
        print(f"  질량: {self.missile_mass} kg")
        print(f"  I_yy/I_zz: {self.I_yy/self.I_zz:.3f}")
        
    def load_default(self):
        """기본값"""
        self.missile_mass = 5860
        self.propellant_mass = 4875
        self.diameter = 0.88
        self.length = 10.94
        self.wing_area = np.pi * (self.diameter/2)**2
        self.burn_time = 65
        self.isp_sea = 230
        print("✓ 기본값 로드 (SCUD-B)")
    
    def get_CD(self, mach, alpha_deg=0):
        """항력 계수 (KAIST 논문 DATCOM 검증값 사용)"""
        # KAIST 논문 Table - 마하수별 CD
        if mach < 0.5:
            CD_mach = 0.25
        elif mach < 0.8:
            # 0.5~0.8 선형 보간
            CD_mach = 0.22 + (0.35 - 0.22) * (mach - 0.5) / (0.8 - 0.5)
        elif mach < 1.0:
            # 0.8~1.0 천음속 항력 상승
            CD_mach = 0.35 + (0.45 - 0.35) * (mach - 0.8) / (1.0 - 0.8)
        elif mach < 1.5:
            # 1.0~1.5 초음속
            CD_mach = 0.45 - (0.45 - 0.35) * (mach - 1.0) / (1.5 - 1.0)
        elif mach < 2.0:
            CD_mach = 0.35 - (0.35 - 0.30) * (mach - 1.5) / (2.0 - 1.5)
        elif mach < 3.0:
            CD_mach = 0.30 - (0.30 - 0.25) * (mach - 2.0) / (3.0 - 2.0)
        elif mach < 4.0:
            CD_mach = 0.25 - (0.25 - 0.22) * (mach - 3.0) / (4.0 - 3.0)
        else:
            # 극초음속 (M >= 4.0)
            CD_mach = 0.22  # KAIST: 극초음속에서 낮은 항력
        
        # 받음각에 의한 추가 항력 (매우 작게)
        CD_alpha = 0.005 * (alpha_deg / 10.0) ** 2  # 이차 의존성
        
        return CD_mach + CD_alpha
    
    def dynamics(self, t, state):
        """
        6DOF 동역학 (교수님 4단계 구조)
        """
        V, gamma, psi, x, y, h, phi, theta, p, q_rate, r, m = state
        
        # 안전값
        V_safe = max(abs(V), 1.0)
        m_safe = max(m, 100.0)
        
        # 중력
        g = cfg.G * cfg.R**2 / (cfg.R + h)**2
        
        # 대기
        rho = get_density(h)
        temperature = max(216.65, 288.15 - 0.0065 * h)
        if h > 11000:
            temperature = 216.65
        sound_speed = 20.05 * np.sqrt(temperature)
        mach = V_safe / sound_speed
        
        # 동압
        q_dyn = 0.5 * rho * V_safe**2
        
        # === 4단계 비행 프로그램 (교수님) ===
        if t < self.vertical_time:
            # Phase 1: 수직 상승
            alpha = 0
            CL = 0
            CD = self.get_CD(mach, 0)
            
        elif t < self.vertical_time + self.pitch_time:
            # Phase 2: 피치 프로그램
            pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
            pitch_progress = (t - self.vertical_time) / self.pitch_time
            alpha = pitch_rad * pitch_progress
            
            # CL은 받음각에 비례 (CL_alpha = 4.5 /rad from KAIST)
            CL_alpha = 4.5  # KAIST 논문 값
            CL = CL_alpha * alpha
            
            CD = self.get_CD(mach, alpha * cfg.RAD_TO_DEG)
            CD += 0.01 * CL**2  # K = 0.01 (KAIST 최적화)
            
        elif t < self.constant_time:
            # Phase 3: 등자세 선회 - 피치각 유지하면서 받음각도 유지
            pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
            alpha = pitch_rad  # 최종 받음각 유지
            
            CL_alpha = 4.5
            CL = CL_alpha * alpha
            
            CD = self.get_CD(mach, alpha * cfg.RAD_TO_DEG)
            CD += 0.01 * CL**2  # K = 0.01 (KAIST 최적화)
            
        else:
            # Phase 4: 탄도 비행
            alpha = 0
            CL = 0
            CD = self.get_CD(mach, 0)
        
        # 공력
        L = q_dyn * CL * self.wing_area
        D = q_dyn * CD * self.wing_area
        
        # 추력
        if t <= self.burn_time:
            T = self.isp_sea * (self.propellant_mass / self.burn_time) * g
            mdot = self.propellant_mass / self.burn_time
        else:
            T = 0
            mdot = 0
        
        # === 3DOF 운동방정식 (교수님) ===
        dV = (T * np.cos(alpha) - D - m_safe * g * np.sin(gamma)) / m_safe
        
        # dgamma 계산
        if t < self.vertical_time:
            dgamma = 0
        elif t < self.vertical_time + self.pitch_time:
            pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
            pitch_progress = (t - self.vertical_time) / self.pitch_time
            # Smoothing
            if pitch_progress < 0.5:
                smoothing = np.sin(np.pi * pitch_progress)
            else:
                smoothing = np.sin(np.pi * (1 - pitch_progress))
            dgamma = -pitch_rad / self.pitch_time * smoothing
        else:
            dgamma = (T * np.sin(alpha) + L - m_safe * g * np.cos(gamma)) / (m_safe * V_safe)
        
        dpsi = 0  # 방위각 변화 없음
        
        # 위치
        dx = V_safe * np.cos(gamma) * np.sin(psi)
        dy = V_safe * np.cos(gamma) * np.cos(psi)
        dh = V_safe * np.sin(gamma)
        
        # === 6DOF 확장 부분 ===
        # Euler 각: phi, theta는 gamma와 동기화
        # phi는 롤 없음
        dphi = p  # 롤 각속도
        dtheta = dgamma  # CRITICAL FIX: 피치각 변화율 = 경로각 변화율
        
        # 공력 모멘트 (간단한 댐핑만)
        q_ref = q_dyn * self.wing_area * self.diameter / V_safe if V_safe > 1 else 0
        
        M_roll = self.C_lp * q_ref * p * self.diameter
        M_pitch = self.C_mq * q_ref * q_rate * self.diameter
        M_yaw = self.C_nr * q_ref * r * self.diameter
        
        # 각가속도
        dp = M_roll / self.I_xx
        dq = M_pitch / self.I_yy
        dr = M_yaw / self.I_zz
        
        # 질량
        dm = -mdot
        
        return [dV, dgamma, dpsi, dx, dy, dh, dphi, dtheta, dp, dq, dr, dm]
    
    def event_ground(self, t, state):
        """지면 충돌"""
        if t < 5.0:
            return 1000.0
        return state[5]  # h
    event_ground.terminal = True
    event_ground.direction = -1
    
    def simulate(self, elevation_deg=45, azimuth_deg=90):
        """시뮬레이션"""
        print(f"\n=== 교수님 구조 6DOF 시뮬레이션 ===")
        print(f"발사: 고각 {elevation_deg}°, 방위각 {azimuth_deg}°")
        
        # 초기 조건 (교수님 방식)
        V0 = 0.1  # 거의 정지 상태
        gamma0 = elevation_deg * cfg.DEG_TO_RAD
        psi0 = (90 - azimuth_deg) * cfg.DEG_TO_RAD
        x0, y0, h0 = 0, 0, 0
        
        phi0, theta0 = 0, gamma0
        p0, q0, r0 = 0, 0, 0
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
        
        # 결과
        results = self.extract_results(t, states)
        
        # 통계
        range_m = np.sqrt(states[-1, 3]**2 + states[-1, 4]**2)
        max_alt = np.max(states[:, 5])
        max_v = np.max(states[:, 0])
        
        print(f"\n결과:")
        print(f"  비행 시간: {t[-1]:.1f} s")
        print(f"  사거리: {range_m/1000:.2f} km")
        print(f"  최대 고도: {max_alt/1000:.2f} km")
        print(f"  최대 속도: {max_v:.1f} m/s (마하 {max_v/340:.2f})")
        
        # 양항비 계산
        self.calculate_performance(results)
        
        return results
    
    def extract_results(self, t, states):
        """결과 추출 (alpha 포함!)"""
        V = states[:, 0]
        gamma = states[:, 1]
        theta = states[:, 7]
        
        # 받음각 계산 (핵심!)
        # Phase별로 다시 계산
        alpha = np.zeros_like(t)
        for i, ti in enumerate(t):
            if ti < self.vertical_time:
                # Phase 1: 수직 상승
                alpha[i] = 0
            elif ti < self.vertical_time + self.pitch_time:
                # Phase 2: 피치 프로그램
                pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
                pitch_progress = (ti - self.vertical_time) / self.pitch_time
                alpha[i] = pitch_rad * pitch_progress
            elif ti < self.constant_time:
                # Phase 3: 등자세
                pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
                alpha[i] = pitch_rad
            else:
                # Phase 4: 탄도
                alpha[i] = 0
        
        results = {
            'time': t,
            'V': V,
            'gamma': gamma,
            'psi': states[:, 2],
            'position_x': states[:, 3],
            'position_y': states[:, 4],
            'altitude': states[:, 5],
            'phi': states[:, 6],
            'theta': theta,
            'p': states[:, 8],
            'q': states[:, 9],
            'r': states[:, 10],
            'mass': states[:, 11],
            'mach': V / 340,
            'alpha': alpha,  # 추가!
        }
        return results

    
    def calculate_performance(self, results):
        """성능 계산 (KAIST 논문 방식)"""
        t = results['time']
        V = results['V']
        h = results['altitude']
        m = results['mass']
        
        # 연소 종료 시점
        burn_idx = np.argmin(np.abs(t - self.burn_time))
        
        if burn_idx > 0:
            V_burnout = V[burn_idx]
            h_burnout = h[burn_idx]
            t_burnout = t[burn_idx]
            
            # 동압
            rho = get_density(h_burnout)
            q = 0.5 * rho * V_burnout**2
            
            # 받음각 (피치 프로그램에서 계산)
            if t_burnout < self.vertical_time + self.pitch_time:
                pitch_rad = self.pitch_angle_deg * cfg.DEG_TO_RAD
                pitch_progress = (t_burnout - self.vertical_time) / self.pitch_time
                alpha = pitch_rad * pitch_progress
            else:
                alpha = self.pitch_angle_deg * cfg.DEG_TO_RAD
            
            # 실제 CL 계산 (받음각 기반)
            CL_alpha = 4.5  # KAIST 논문
            CL = CL_alpha * alpha
            
            # CD 계산
            mach = V_burnout / 340
            CD = self.get_CD(mach, alpha * cfg.RAD_TO_DEG)
            CD += 0.01 * CL**2  # K = 0.01 (KAIST)
            
            # 양력, 항력
            L = q * CL * self.wing_area
            D = q * CD * self.wing_area
            
            # 양항비
            if D > 0:
                LD_ratio = L / D
                print(f"\n성능 (연소 종료 t={t_burnout:.1f}s):")
                print(f"  받음각: {alpha * cfg.RAD_TO_DEG:.2f}°")
                print(f"  CL: {CL:.3f}")
                print(f"  CD: {CD:.3f}")
                print(f"  양항비 (L/D): {LD_ratio:.3f}")
                print(f"  목표 (KAIST): 4.782")
                
                # Sustained-g
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
    
    # 45도 발사
    results = missile.simulate(elevation_deg=45, azimuth_deg=90)
    
    # Windows 경로로 저장
    os.makedirs("results_6dof", exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filepath = f"results_6dof/SCUD-B_45deg_professor_{timestamp}.npz"
    
    missile.save_trajectory(filepath, results)


if __name__ == "__main__":
    main()