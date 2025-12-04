#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
통합 6DOF 미사일 시뮬레이션
chanjinkim 물리 엔진 + JJINleejunseo 시각화 + NPZ 데이터 파이프라인
"""
import os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import datetime

# 로컬 모듈
from dynamics_true_6dof import DynamicsTrue6DOF
from trajectory_io import save_trajectory, load_trajectory, validate_trajectory
from coordinate_transforms import body_to_trajectory, calculate_alpha_beta


class IntegratedMissileSimulation:
    """통합 6DOF 미사일 시뮬레이션 클래스"""
    
    def __init__(self, missile_config):
        """
        초기화
        
        Parameters:
        -----------
        missile_config : dict
            미사일 설정 (chanjinkim 포맷)
        """
        self.dynamics = DynamicsTrue6DOF(missile_config)
        self.missile_config = missile_config
        self.results = None
        self.npz_path = None
    
    def run_simulation(self, launch_angle_deg=45, azimuth_deg=90, 
                      sim_time=600, disturbance=None):
        """
        물리 시뮬레이션 실행 (chanjinkim 엔진 사용)
        
        Parameters:
        -----------
        launch_angle_deg : float
            발사각 (도)
        azimuth_deg : float
            방위각 (도)
        sim_time : float
            최대 시뮬레이션 시간 (초)
        disturbance : dict, optional
            외란 설정 {'time': float, 'moment': [Mx, My, Mz]}
        
        Returns:
        --------
        success : bool
            시뮬레이션 성공 여부
        """
        print("=" * 60)
        print("통합 6DOF 미사일 시뮬레이션 시작")
        print("=" * 60)
        print(f"미사일: {self.missile_config['name']}")
        print(f"발사각: {launch_angle_deg}°, 방위각: {azimuth_deg}°")
        
        # 초기 상태
        state0 = self.dynamics.create_initial_state(launch_angle_deg, azimuth_deg)
        
        # 외란 주입 래퍼
        if disturbance is not None:
            original_dynamics = self.dynamics.dynamics_equations
            dist_time = disturbance['time']
            dist_moment = np.array(disturbance.get('moment', [0, 0, 0]))
            
            def dynamics_with_disturbance(t, state):
                dstate = original_dynamics(t, state)
                # 특정 시간에 모멘트 충격 추가
                if abs(t - dist_time) < 0.1:  # 0.1초 동안 지속
                    dstate[3] += dist_moment[0] / self.missile_config['inertia_xx']  # p
                    dstate[4] += dist_moment[1] / self.missile_config['inertia_yy']  # q
                    dstate[5] += dist_moment[2] / self.missile_config['inertia_zz']  # r
                return dstate
            
            dynamics_func = dynamics_with_disturbance
            print(f"⚠ 외란 주입: t={dist_time}s, M={dist_moment}")
        else:
            dynamics_func = self.dynamics.dynamics_equations
        
        # 지면 충돌 이벤트
        def ground_event(t, state):
            return state[11]  # Z (고도)
        
        ground_event.terminal = True
        ground_event.direction = -1
        
        # 적분 실행
        print("적분 중...")
        sol = solve_ivp(
            dynamics_func,
            [0, sim_time],
            state0,
            method='RK45',
            events=[ground_event],
            max_step=0.5,  # 더 작은 스텝 (외란 감지용)
            rtol=1e-6,
            atol=1e-8
        )
        
        if not sol.success:
            print(f"⚠ 적분 실패: {sol.message}")
            return False
        
        # 결과 저장
        print("결과 처리 중...")
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
        alpha, beta = calculate_alpha_beta(sol.y[0, :], sol.y[1, :], sol.y[2, :])
        
        self.results['V_mag'] = V_mag
        self.results['alpha'] = alpha
        self.results['beta'] = beta
        
        # 요약 정보
        final_time = sol.t[-1]
        final_range = np.sqrt(sol.y[9, -1]**2 + sol.y[10, -1]**2) / 1000
        max_altitude = np.max(sol.y[11, :]) / 1000
        
        print(f"\n✓ 시뮬레이션 완료!")
        print(f"  비행 시간: {final_time:.2f} s")
        print(f"  최종 거리: {final_range:.2f} km")
        print(f"  최대 고도: {max_altitude:.2f} km")
        print("=" * 60)
        
        return True
    
    def save_to_npz(self, filepath=None, launch_angle_deg=45):
        """
        결과를 NPZ 파일로 저장
        
        Parameters:
        -----------
        filepath : str, optional
            저장 경로 (None이면 자동 생성)
        launch_angle_deg : float
            발사각 (메타데이터)
        
        Returns:
        --------
        filepath : str
            저장된 파일 경로
        """
        if self.results is None:
            print("⚠ 먼저 시뮬레이션을 실행하세요.")
            return None
        
        # 파일 경로 자동 생성
        if filepath is None:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            missile_name = self.missile_config['name']
            filepath = f"results_6dof/{missile_name}_{launch_angle_deg}deg_{timestamp}.npz"
        
        # Trajectory Frame 변환
        V, gamma, chi = body_to_trajectory(
            self.results['u'],
            self.results['v'],
            self.results['w'],
            self.results['phi'],
            self.results['theta'],
            self.results['psi']
        )
        
        # 질량 계산
        time = self.results['time']
        burn_time = self.missile_config.get('burn_time', 0)
        propellant_mass = self.missile_config.get('propellant_mass', 0)
        launch_weight = self.missile_config.get('launch_weight', 0)
        
        mass = np.zeros_like(time)
        for i, t in enumerate(time):
            if t < burn_time:
                mass[i] = launch_weight - (propellant_mass * t / burn_time)
            else:
                mass[i] = launch_weight - propellant_mass
        
        # NPZ 저장
        save_trajectory(
            filepath=filepath,
            time=time,
            position_x=self.results['X'],
            position_y=self.results['Y'],
            position_z=self.results['Z'],
            u=self.results['u'],
            v=self.results['v'],
            w=self.results['w'],
            phi=self.results['phi'],
            theta=self.results['theta'],
            psi=self.results['psi'],
            p=self.results['p'],
            q=self.results['q'],
            r=self.results['r'],
            mass=mass,
            V=V,
            gamma=gamma,
            chi=chi,
            missile_type=self.missile_config['name'],
            launch_angle=launch_angle_deg,
            alpha=self.results['alpha'],
            beta=self.results['beta']
        )
        
        self.npz_path = filepath
        return filepath
    
    def load_from_npz(self, filepath):
        """
        NPZ 파일에서 데이터 로드
        
        Parameters:
        -----------
        filepath : str
            NPZ 파일 경로
        
        Returns:
        --------
        success : bool
            로드 성공 여부
        """
        try:
            data = load_trajectory(filepath)
            
            # results 형식으로 변환
            self.results = {
                'time': data['time'],
                'u': data['u'],
                'v': data['v'],
                'w': data['w'],
                'p': data['p'],
                'q': data['q'],
                'r': data['r'],
                'phi': data['phi'],
                'theta': data['theta'],
                'psi': data['psi'],
                'X': data['position_x'],
                'Y': data['position_y'],
                'Z': data['position_z'],
                'V_mag': data['V'],
                'alpha': data['alpha'],
                'beta': data['beta'],
            }
            
            # 데이터 검증
            validate_trajectory(data)
            
            self.npz_path = filepath
            print(f"✓ NPZ 로드 완료: {filepath}")
            return True
            
        except Exception as e:
            print(f"⚠ NPZ 로드 실패: {e}")
            return False
    
    def plot_comprehensive(self, save_dir='results_6dof'):
        """
        JJINleejunseo 스타일 12-subplot 그래프
        
        Parameters:
        -----------
        save_dir : str
            저장 디렉토리
        """
        if self.results is None:
            print("⚠ 데이터가 없습니다.")
            return
        
        os.makedirs(save_dir, exist_ok=True)
        
        fig = plt.figure(figsize=(18, 10))
        
        # 1. 3D 궤적
        ax1 = fig.add_subplot(3, 4, 1, projection='3d')
        X_km = self.results['X'] / 1000
        Y_km = self.results['Y'] / 1000
        Z_km = self.results['Z'] / 1000
        ax1.plot(X_km, Y_km, Z_km, 'b-', linewidth=2)
        ax1.scatter([X_km[0]], [Y_km[0]], [Z_km[0]], c='g', s=100, label='Start')
        ax1.scatter([X_km[-1]], [Y_km[-1]], [Z_km[-1]], c='r', s=100, label='End')
        ax1.set_xlabel('X (km)', fontsize=9)
        ax1.set_ylabel('Y (km)', fontsize=9)
        ax1.set_zlabel('Alt (km)', fontsize=9)
        ax1.set_title('3D Trajectory', fontsize=10)
        ax1.legend(fontsize=8)
        
        # 2. 속도
        ax2 = fig.add_subplot(3, 4, 2)
        ax2.plot(self.results['time'], self.results['V_mag'], 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)', fontsize=9)
        ax2.set_ylabel('Velocity (m/s)', fontsize=9)
        ax2.set_title('Velocity Magnitude', fontsize=10)
        ax2.grid(True)
        
        # 3. 고도
        ax3 = fig.add_subplot(3, 4, 3)
        ax3.plot(self.results['time'], self.results['Z']/1000, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)', fontsize=9)
        ax3.set_ylabel('Altitude (km)', fontsize=9)
        ax3.set_title('Altitude', fontsize=10)
        ax3.grid(True)
        
        # 4. 받음각 (핵심!)
        ax4 = fig.add_subplot(3, 4, 4)
        ax4.plot(self.results['time'], np.degrees(self.results['alpha']), 'r-', linewidth=2)
        ax4.set_xlabel('Time (s)', fontsize=9)
        ax4.set_ylabel('Alpha (deg)', fontsize=9)
        ax4.set_title('Angle of Attack', fontsize=10)
        ax4.grid(True)
        
        # 5. Euler 각도
        ax5 = fig.add_subplot(3, 4, 5)
        ax5.plot(self.results['time'], np.degrees(self.results['phi']), 'r-', label='Roll')
        ax5.plot(self.results['time'], np.degrees(self.results['theta']), 'g-', label='Pitch')
        ax5.plot(self.results['time'], np.degrees(self.results['psi']), 'b-', label='Yaw')
        ax5.set_xlabel('Time (s)', fontsize=9)
        ax5.set_ylabel('Angle (deg)', fontsize=9)
        ax5.set_title('Euler Angles', fontsize=10)
        ax5.legend(fontsize=7)
        ax5.grid(True)
        
        # 6. 각속도 (핵심!)
        ax6 = fig.add_subplot(3, 4, 6)
        ax6.plot(self.results['time'], np.degrees(self.results['p']), 'r-', label='p')
        ax6.plot(self.results['time'], np.degrees(self.results['q']), 'g-', label='q')
        ax6.plot(self.results['time'], np.degrees(self.results['r']), 'b-', label='r')
        ax6.set_xlabel('Time (s)', fontsize=9)
        ax6.set_ylabel('Rate (deg/s)', fontsize=9)
        ax6.set_title('Angular Rates', fontsize=10)
        ax6.legend(fontsize=7)
        ax6.grid(True)
        
        # 7. Body Frame 속도
        ax7 = fig.add_subplot(3, 4, 7)
        ax7.plot(self.results['time'], self.results['u'], 'r-', label='u')
        ax7.plot(self.results['time'], self.results['v'], 'g-', label='v')
        ax7.plot(self.results['time'], self.results['w'], 'b-', label='w')
        ax7.set_xlabel('Time (s)', fontsize=9)
        ax7.set_ylabel('Velocity (m/s)', fontsize=9)
        ax7.set_title('Body Frame Velocities', fontsize=10)
        ax7.legend(fontsize=7)
        ax7.grid(True)
        
        # 8. 측면 받음각
        ax8 = fig.add_subplot(3, 4, 8)
        ax8.plot(self.results['time'], np.degrees(self.results['beta']), 'm-', linewidth=2)
        ax8.set_xlabel('Time (s)', fontsize=9)
        ax8.set_ylabel('Beta (deg)', fontsize=9)
        ax8.set_title('Sideslip Angle', fontsize=10)
        ax8.grid(True)
        
        # 9. 위상 평면 (α-q) - 시그니처 분석용!
        ax9 = fig.add_subplot(3, 4, 9)
        ax9.plot(np.degrees(self.results['alpha']), np.degrees(self.results['q']), 'b-', linewidth=1.5)
        ax9.scatter([np.degrees(self.results['alpha'][0])], 
                   [np.degrees(self.results['q'][0])], c='g', s=50, label='Start')
        ax9.set_xlabel('Alpha (deg)', fontsize=9)
        ax9.set_ylabel('Pitch Rate q (deg/s)', fontsize=9)
        ax9.set_title('Phase Plane (α-q)', fontsize=10)
        ax9.legend(fontsize=7)
        ax9.grid(True)
        
        # 10. Ground Track
        ax10 = fig.add_subplot(3, 4, 10)
        ax10.plot(X_km, Y_km, 'b-', linewidth=2)
        ax10.scatter([X_km[0]], [Y_km[0]], c='g', s=100, label='Start')
        ax10.scatter([X_km[-1]], [Y_km[-1]], c='r', s=100, label='End')
        ax10.set_xlabel('X (km)', fontsize=9)
        ax10.set_ylabel('Y (km)', fontsize=9)
        ax10.set_title('Ground Track', fontsize=10)
        ax10.legend(fontsize=7)
        ax10.grid(True)
        ax10.axis('equal')
        
        # 11. Range vs Altitude
        ax11 = fig.add_subplot(3, 4, 11)
        range_km = np.sqrt(self.results['X']**2 + self.results['Y']**2) / 1000
        ax11.plot(range_km, Z_km, 'b-', linewidth=2)
        ax11.set_xlabel('Range (km)', fontsize=9)
        ax11.set_ylabel('Altitude (km)', fontsize=9)
        ax11.set_title('Range vs Altitude', fontsize=10)
        ax11.grid(True)
        
        # 12. Roll-Yaw 상호작용 (시그니처 분석용!)
        ax12 = fig.add_subplot(3, 4, 12)
        ax12.plot(np.degrees(self.results['p']), np.degrees(self.results['r']), 'r-', linewidth=1.5)
        ax12.set_xlabel('Roll Rate p (deg/s)', fontsize=9)
        ax12.set_ylabel('Yaw Rate r (deg/s)', fontsize=9)
        ax12.set_title('Roll-Yaw Coupling', fontsize=10)
        ax12.grid(True)
        
        plt.tight_layout(pad=3.0)
        
        # 저장
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(save_dir, f'integrated_6dof_{timestamp}.png')
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"✓ 그래프 저장: {filepath}")
        plt.show()
        
        return filepath


def get_missile_configs():
    """미사일 설정 반환"""
    configs = {}
    
    # SCUD-B
    diameter_scud = 0.88
    length_scud = 10.94
    mass_scud = 5860
    configs['SCUD-B'] = {
        'name': 'SCUD-B',
        'launch_weight': mass_scud,
        'propellant_mass': 4875,
        'diameter': diameter_scud,
        'length': length_scud,
        'reference_area': np.pi * (diameter_scud/2)**2,
        'wingspan': diameter_scud * 2,
        'inertia_xx': mass_scud * (length_scud**2 / 12 + (diameter_scud/2)**2 / 4),
        'inertia_yy': mass_scud * (length_scud**2 / 12 + (diameter_scud/2)**2 / 4),
        'inertia_zz': mass_scud * ((diameter_scud/2)**2 / 2),
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
        'thrust_profile': None
    }
    
    # 노동 (SCUD 기반, 더 큰 크기)
    diameter_nodong = 1.25
    length_nodong = 15.5
    mass_nodong = 16000
    configs['Nodong'] = {
        'name': 'Nodong',
        'launch_weight': mass_nodong,
        'propellant_mass': 13000,
        'diameter': diameter_nodong,
        'length': length_nodong,
        'reference_area': np.pi * (diameter_nodong/2)**2,
        'wingspan': diameter_nodong * 2,
        'inertia_xx': mass_nodong * (length_nodong**2 / 12 + (diameter_nodong/2)**2 / 4),
        'inertia_yy': mass_nodong * (length_nodong**2 / 12 + (diameter_nodong/2)**2 / 4),
        'inertia_zz': mass_nodong * ((diameter_nodong/2)**2 / 2),
        'isp_sea': 240,
        'isp_vac': 270,
        'burn_time': 95,
        'cd_base': 0.23,
        'cl_alpha': 3.2,
        'cy_beta': -0.5,
        'cm_alpha': -0.18,
        'cn_beta': 0.12,
        'cl_p': -0.6,
        'cm_q': -1.0,
        'cn_r': -1.0,
        'thrust_profile': None
    }
    
    # KN-23 (고체 연료, 기동형)
    diameter_kn23 = 0.95
    length_kn23 = 7.5
    mass_kn23 = 3400
    configs['KN-23'] = {
        'name': 'KN-23',
        'launch_weight': mass_kn23,
        'propellant_mass': 2200,
        'diameter': diameter_kn23,
        'length': length_kn23,
        'reference_area': np.pi * (diameter_kn23/2)**2,
        'wingspan': diameter_kn23 * 1.5,  # 작은 날개
        'inertia_xx': mass_kn23 * (length_kn23**2 / 12 + (diameter_kn23/2)**2 / 4),
        'inertia_yy': mass_kn23 * (length_kn23**2 / 12 + (diameter_kn23/2)**2 / 4),
        'inertia_zz': mass_kn23 * ((diameter_kn23/2)**2 / 2),
        'isp_sea': 260,
        'isp_vac': 290,
        'burn_time': 28,  # 고체 연료 - 짧은 연소
        'cd_base': 0.20,
        'cl_alpha': 4.0,  # 높은 양력
        'cy_beta': -0.6,
        'cm_alpha': -0.25,  # 강한 복원 모멘트
        'cn_beta': 0.15,
        'cl_p': -0.4,
        'cm_q': -1.2,  # 강한 댐핑
        'cn_r': -1.2,
        'thrust_profile': None
    }
    
    return configs


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print("통합 6DOF 미사일 시뮬레이션 시스템")
    print("chanjinkim 물리 + JJINleejunseo 시각화")
    print("=" * 60 + "\n")
    
    # 미사일 선택
    configs = get_missile_configs()
    print("미사일 종류:")
    for i, name in enumerate(configs.keys(), 1):
        print(f"  {i}. {name}")
    
    choice = input("\n미사일 선택 (1-3, 기본값: 1): ").strip()
    missile_names = list(configs.keys())
    missile_name = missile_names[int(choice)-1] if choice in ['1','2','3'] else 'SCUD-B'
    
    missile_config = configs[missile_name]
    print(f"\n선택: {missile_name}")
    
    # 시뮬레이션 생성
    sim = IntegratedMissileSimulation(missile_config)
    
    # 실행 모드
    print("\n실행 모드:")
    print("  1. 새 시뮬레이션 실행 (외란 없음)")
    print("  2. 새 시뮬레이션 실행 (외란 주입)")
    print("  3. NPZ 파일 로드")
    
    mode = input("\n모드 선택 (1-3, 기본값: 1): ").strip()
    
    if mode == '3':
        # NPZ 로드
        npz_file = input("NPZ 파일 경로: ").strip()
        if sim.load_from_npz(npz_file):
            sim.plot_comprehensive()
    else:
        # 새 시뮬레이션
        launch_angle = float(input("발사각 (도, 기본값: 45): ").strip() or "45")
        
        disturbance = None
        if mode == '2':
            dist_time = float(input("외란 주입 시간 (초, 기본값: 70): ").strip() or "70")
            dist_magnitude = float(input("외란 크기 (N·m, 기본값: 10000): ").strip() or "10000")
            disturbance = {
                'time': dist_time,
                'moment': [0, dist_magnitude, 0]  # 피치 모멘트
            }
        
        # 시뮬레이션 실행
        if sim.run_simulation(
            launch_angle_deg=launch_angle,
            azimuth_deg=90,
            sim_time=600,
            disturbance=disturbance
        ):
            # NPZ 저장
            npz_path = sim.save_to_npz(launch_angle_deg=launch_angle)
            
            # 시각화
            sim.plot_comprehensive()
            
            print(f"\n✓ 완료! NPZ 파일: {npz_path}")
        else:
            print("\n⚠ 시뮬레이션 실패")


if __name__ == "__main__":
    main()
