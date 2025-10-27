#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
진짜 6DOF 미사일 시뮬레이션 - 메인 파일
Body Frame 기반, 완전 커플링된 병진-회전 운동
"""

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics_true_6dof import DynamicsTrue6DOF
import datetime


class MissileSimulationTrue6DOF:
    """진짜 6DOF 미사일 시뮬레이션 클래스"""
    
    def __init__(self, missile_config):
        """
        시뮬레이션 초기화
        
        Parameters:
        -----------
        missile_config : dict
            미사일 설정
        """
        self.dynamics = DynamicsTrue6DOF(missile_config)
        self.missile_config = missile_config
        self.results = None
    
    def run_simulation(self, launch_angle_deg=45, azimuth_deg=90, sim_time=600):
        """
        시뮬레이션 실행
        
        Parameters:
        -----------
        launch_angle_deg : float
            발사각 (도)
        azimuth_deg : float
            방위각 (도)
        sim_time : float
            시뮬레이션 시간 (초)
        
        Returns:
        --------
        results : dict
            시뮬레이션 결과
        """
        print("=" * 60)
        print("진짜 6DOF 미사일 시뮬레이션 시작")
        print("=" * 60)
        print(f"발사각: {launch_angle_deg}°, 방위각: {azimuth_deg}°")
        
        # 초기 상태
        state0 = self.dynamics.create_initial_state(launch_angle_deg, azimuth_deg)
        
        # 지면 충돌 이벤트
        def ground_event(t, state):
            return state[11]  # Z (고도)
        
        ground_event.terminal = True
        ground_event.direction = -1
        
        # 적분 실행
        print("적분 중...")
        sol = solve_ivp(
            self.dynamics.dynamics_equations,
            [0, sim_time],
            state0,
            method='RK45',
            events=[ground_event],
            max_step=1.0,
            rtol=1e-6,
            atol=1e-8
        )
        
        if not sol.success:
            print(f"경고: 적분 실패 - {sol.message}")
            return None
        
        # 결과 저장
        print("결과 처리 중...")
        n_points = len(sol.t)
        
        self.results = {
            'time': sol.t,
            'u': sol.y[0, :],
            'v': sol.y[1, :],
            'w': sol.y[2, :],
            'p': sol.y[3, :],
            'q': sol.y[4, :],
            'r': sol.y[5, :],
            'phi': sol.y[6, :],
            'theta': sol.y[7, :],
            'psi': sol.y[8, :],
            'X': sol.y[9, :],
            'Y': sol.y[10, :],
            'Z': sol.y[11, :],
        }
        
        # 추가 계산
        V_mag = np.sqrt(sol.y[0, :]**2 + sol.y[1, :]**2 + sol.y[2, :]**2)
        self.results['V_mag'] = V_mag
        
        # 받음각
        alpha = np.arctan2(sol.y[2, :], sol.y[0, :])
        beta = np.arcsin(np.clip(sol.y[1, :] / (V_mag + 1e-10), -1.0, 1.0))
        self.results['alpha'] = alpha
        self.results['beta'] = beta
        
        # 마하수
        mach = []
        for i in range(n_points):
            alt = sol.y[11, i]
            a = self.dynamics.aero.get_sound_speed(alt)
            mach.append(V_mag[i] / a if a > 0 else 0)
        self.results['mach'] = np.array(mach)
        
        # 거리 (수평)
        range_km = np.sqrt(sol.y[9, :]**2 + sol.y[10, :]**2) / 1000
        self.results['range_km'] = range_km
        
        # 최종 정보
        final_time = sol.t[-1]
        final_range = range_km[-1]
        max_altitude = np.max(sol.y[11, :]) / 1000
        max_velocity = np.max(V_mag)
        
        print(f"\n시뮬레이션 완료!")
        print(f"  비행 시간: {final_time:.2f} s")
        print(f"  최종 거리: {final_range:.2f} km")
        print(f"  최대 고도: {max_altitude:.2f} km")
        print(f"  최대 속도: {max_velocity:.2f} m/s (마하 {max_velocity/340:.2f})")
        print("=" * 60)
        
        return self.results
    
    def plot_results(self, save_dir='results_true_6dof'):
        """
        결과 시각화
        
        Parameters:
        -----------
        save_dir : str
            저장 디렉토리
        """
        if self.results is None:
            print("먼저 시뮬레이션을 실행하세요.")
            return
        
        os.makedirs(save_dir, exist_ok=True)
        
        # 1. 3D 궤적
        fig = plt.figure(figsize=(14, 10))
        
        # 3D 플롯
        ax1 = fig.add_subplot(221, projection='3d')
        X_km = self.results['X'] / 1000
        Y_km = self.results['Y'] / 1000
        Z_km = self.results['Z'] / 1000
        
        ax1.plot(X_km, Y_km, Z_km, 'b-', linewidth=2)
        ax1.scatter([X_km[0]], [Y_km[0]], [Z_km[0]], c='g', s=100, label='Start')
        ax1.scatter([X_km[-1]], [Y_km[-1]], [Z_km[-1]], c='r', s=100, label='Impact')
        ax1.set_xlabel('X (km, North)')
        ax1.set_ylabel('Y (km, East)')
        ax1.set_zlabel('Altitude (km)')
        ax1.set_title('3D Trajectory (True 6DOF)')
        ax1.legend()
        ax1.grid(True)
        
        # 2. 속도
        ax2 = fig.add_subplot(222)
        ax2.plot(self.results['time'], self.results['V_mag'], 'b-', linewidth=2, label='Total')
        ax2.plot(self.results['time'], self.results['u'], 'r--', label='u (body x)')
        ax2.plot(self.results['time'], self.results['v'], 'g--', label='v (body y)')
        ax2.plot(self.results['time'], self.results['w'], 'm--', label='w (body z)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Components (Body Frame)')
        ax2.legend()
        ax2.grid(True)
        
        # 3. 자세 (오일러각)
        ax3 = fig.add_subplot(223)
        ax3.plot(self.results['time'], np.degrees(self.results['phi']), 'r-', label='Roll (φ)')
        ax3.plot(self.results['time'], np.degrees(self.results['theta']), 'g-', label='Pitch (θ)')
        ax3.plot(self.results['time'], np.degrees(self.results['psi']), 'b-', label='Yaw (ψ)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (deg)')
        ax3.set_title('Euler Angles')
        ax3.legend()
        ax3.grid(True)
        
        # 4. 각속도
        ax4 = fig.add_subplot(224)
        ax4.plot(self.results['time'], np.degrees(self.results['p']), 'r-', label='p (roll rate)')
        ax4.plot(self.results['time'], np.degrees(self.results['q']), 'g-', label='q (pitch rate)')
        ax4.plot(self.results['time'], np.degrees(self.results['r']), 'b-', label='r (yaw rate)')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angular Rate (deg/s)')
        ax4.set_title('Angular Rates (Body Frame)')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(save_dir, f'true_6dof_trajectory_{timestamp}.png')
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"그래프 저장: {filepath}")
        plt.show()
        
        # 2. 상세 플롯
        fig2 = plt.figure(figsize=(14, 10))
        
        # 받음각
        ax1 = fig2.add_subplot(231)
        ax1.plot(self.results['time'], np.degrees(self.results['alpha']), 'b-')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle of Attack (deg)')
        ax1.set_title('Angle of Attack')
        ax1.grid(True)
        
        # 측면 받음각
        ax2 = fig2.add_subplot(232)
        ax2.plot(self.results['time'], np.degrees(self.results['beta']), 'r-')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Sideslip Angle (deg)')
        ax2.set_title('Sideslip Angle')
        ax2.grid(True)
        
        # 마하수
        ax3 = fig2.add_subplot(233)
        ax3.plot(self.results['time'], self.results['mach'], 'g-')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Mach Number')
        ax3.set_title('Mach Number')
        ax3.grid(True)
        
        # 고도 vs 거리
        ax4 = fig2.add_subplot(234)
        ax4.plot(self.results['range_km'], self.results['Z']/1000, 'b-', linewidth=2)
        ax4.set_xlabel('Range (km)')
        ax4.set_ylabel('Altitude (km)')
        ax4.set_title('Altitude vs Range')
        ax4.grid(True)
        
        # X-Y 평면도
        ax5 = fig2.add_subplot(235)
        ax5.plot(self.results['X']/1000, self.results['Y']/1000, 'b-', linewidth=2)
        ax5.scatter([self.results['X'][0]/1000], [self.results['Y'][0]/1000], c='g', s=100, label='Start')
        ax5.scatter([self.results['X'][-1]/1000], [self.results['Y'][-1]/1000], c='r', s=100, label='Impact')
        ax5.set_xlabel('X (km, North)')
        ax5.set_ylabel('Y (km, East)')
        ax5.set_title('Ground Track')
        ax5.legend()
        ax5.grid(True)
        ax5.axis('equal')
        
        # 시간-고도
        ax6 = fig2.add_subplot(236)
        ax6.plot(self.results['time'], self.results['Z']/1000, 'b-', linewidth=2)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Altitude (km)')
        ax6.set_title('Altitude vs Time')
        ax6.grid(True)
        
        plt.tight_layout()
        
        filepath2 = os.path.join(save_dir, f'true_6dof_details_{timestamp}.png')
        plt.savefig(filepath2, dpi=300, bbox_inches='tight')
        print(f"상세 그래프 저장: {filepath2}")
        plt.show()


def get_scud_b_config():
    """SCUD-B 미사일 설정 반환"""
    diameter = 0.88
    length = 10.94
    mass = 5860
    
    config = {
        'name': 'SCUD-B',
        'launch_weight': mass,
        'propellant_mass': 4875,
        'diameter': diameter,
        'length': length,
        'reference_area': np.pi * (diameter/2)**2,
        'wingspan': diameter * 2,
        'inertia_xx': mass * (length**2 / 12 + (diameter/2)**2 / 4),
        'inertia_yy': mass * (length**2 / 12 + (diameter/2)**2 / 4),
        'inertia_zz': mass * ((diameter/2)**2 / 2),
        'isp_sea': 230,
        'isp_vac': 258,
        'burn_time': 65,
        'cd_base': 0.25,
        'cl_alpha': 3.5,
        'cy_beta': -0.5,
        'cm_alpha': -0.15,
        'cn_beta': 0.1,
        'cl_p': -0.5,
        'cm_q': -0.8,
        'cn_r': -0.8,
        'thrust_profile': None  # 일정한 추력
    }
    
    return config


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print("진짜 6DOF 미사일 시뮬레이션")
    print("=" * 60 + "\n")
    
    # 미사일 설정
    missile_config = get_scud_b_config()
    print(f"미사일: {missile_config['name']}")
    print(f"  질량: {missile_config['launch_weight']} kg")
    print(f"  연소 시간: {missile_config['burn_time']} s")
    print(f"  비추력: {missile_config['isp_sea']} s (해면)")
    
    # 시뮬레이션 생성
    sim = MissileSimulationTrue6DOF(missile_config)
    
    # 시뮬레이션 실행
    launch_angle = 45  # 도
    azimuth = 90  # 도 (동쪽)
    sim_time = 600  # 초
    
    results = sim.run_simulation(
        launch_angle_deg=launch_angle,
        azimuth_deg=azimuth,
        sim_time=sim_time
    )
    
    if results is not None:
        # 결과 시각화
        sim.plot_results()
    else:
        print("시뮬레이션 실패!")


if __name__ == "__main__":
    main()
