#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6DOF 미사일 시각화 시스템
radar_6dof_simulator.py 기반 + NPZ 저장/로드 기능 추가
"""
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import datetime
from scipy.integrate import solve_ivp

# missile_6dof_FINAL_CORRECTED 사용
from missile_6dof_FINAL_CORRECTED import Missile6DOF_Authentic

# NPZ I/O 모듈
try:
    from trajectory_io import save_trajectory, load_trajectory, validate_trajectory
except ImportError:
    print("⚠ trajectory_io 모듈 없음. NPZ 저장/로드 기능 비활성화")
    save_trajectory = None
    load_trajectory = None
    validate_trajectory = None

# matplotlib 설정
plt.rcParams['axes.unicode_minus'] = False


class MissileVisualization6DOF:
    """
    6DOF 미사일 시각화 클래스
    - 시뮬레이션 실행 (radar_6dof_simulator.py의 Radar6DOFSimulator 사용)
    - NPZ 저장/로드
    - 12-subplot 그래프 생성
    """
    
    def __init__(self, missile_type="SCUD-B"):
        """생성자"""
        self.missile_type = missile_type
        self.results = None
        self.npz_path = None
        self.sol = None
        
        # missile_6dof_FINAL_CORRECTED 객체 생성
        self.sim = Missile6DOF_Authentic(missile_type=missile_type)
    
    def run_simulation(self, launch_angle_deg=45, azimuth_deg=90, sim_time=600):
        """
        6DOF 시뮬레이션 실행 (Missile6DOF_Professor 사용)
        
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
        success : bool
            시뮬레이션 성공 여부
        """
        print("=" * 60)
        print(f"6DOF 시뮬레이션 시작: {self.missile_type}")
        print(f"발사각: {launch_angle_deg}°, 방위각: {azimuth_deg}°")
        print("=" * 60)
        
        try:
            # Missile6DOF_Professor의 simulate() 메소드 호출
            sim_results = self.sim.simulate(elevation_deg=launch_angle_deg, azimuth_deg=azimuth_deg)
            
            # 결과를 main_visualization 형식으로 변환
            self._convert_results_from_professor(sim_results)
            
            # 요약 정보
            final_time = self.results['time'][-1]
            final_range = np.sqrt(self.results['x'][-1]**2 + self.results['y'][-1]**2) / 1000
            max_altitude = np.max(self.results['h']) / 1000
            max_velocity = np.max(self.results['velocity'])
            
            print(f"\n✓ 시뮬레이션 완료!")
            print(f"  비행 시간: {final_time:.2f} s")
            print(f"  최종 거리: {final_range:.2f} km")
            print(f"  최대 고도: {max_altitude:.2f} km")
            print(f"  최대 속도: {max_velocity:.2f} m/s (마하 {max_velocity/340:.2f})")
            print("=" * 60)
            
            return True
            
        except Exception as e:
            print(f"⚠ 시뮬레이션 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _convert_results_from_professor(self, sim_results):
        """
        Missile6DOF_Professor 결과를 main_visualization 형식으로 변환
        
        Parameters:
        -----------
        sim_results : dict
            시뮬레이션 결과 (Missile6DOF_Professor.simulate() 반환값)
        """
        t = sim_results.get('time', np.array([]))
        V = sim_results.get('V', np.zeros_like(t))
        psi = sim_results.get('psi', np.zeros_like(t))
        x = sim_results.get('position_x', np.zeros_like(t))
        y = sim_results.get('position_y', np.zeros_like(t))
        h = sim_results.get('position_z', sim_results.get('altitude', np.zeros_like(t)))
        phi = sim_results.get('phi', np.zeros_like(t))
        theta = sim_results.get('theta', np.zeros_like(t))
        p = sim_results.get('p', np.zeros_like(t))
        q = sim_results.get('q', np.zeros_like(t))
        r = sim_results.get('r', np.zeros_like(t))
        mass = sim_results.get('mass', np.zeros_like(t))
        mach = sim_results.get('mach', np.zeros_like(t))
        
        # 받음각 계산
        if 'alpha' in sim_results:
            alpha = sim_results['alpha']
        else:
            alpha = np.zeros_like(t)
        
        # gamma (비행경로각) 직접 계산: gamma = theta - alpha
        if 'gamma' in sim_results:
            gamma = sim_results['gamma']
        else:
            gamma = theta - alpha
        
        beta = np.zeros_like(t)
        
        # CRITICAL: Theta wrapping (-180° ~ 180°)
        # gamma는 계속 감소하므로 theta도 누적됨 (예: 45° → 233°)
        # 시각화를 위해 -π ~ π 범위로 wrap
        def wrap_angle(angle):
            """각도를 -π ~ π 범위로 wrap"""
            while angle > np.pi:
                angle -= 2 * np.pi
            while angle < -np.pi:
                angle += 2 * np.pi
            return angle
        
        theta_wrapped = np.array([wrap_angle(th) for th in theta])
        
        # 연료 (초기 질량 - 현재 질량)
        fuel = mass[0] - mass
        
        # results 딕셔너리 생성
        self.results = {
            'time': t,
            'x': x,
            'y': y,
            'h': h,
            'velocity': V,
            'gamma': gamma,
            'psi': psi,
            'phi': phi,
            'theta': theta_wrapped,  # ✅ Wrapped!
            'psi_euler': psi,  # 오일러 요각 (같은 값 사용)
            'p': p,
            'q': q,
            'r': r,
            'alpha': alpha,
            'beta': beta,
            'mach': mach,
            'mass': mass,
            'fuel': fuel
        }
    
    def save_to_npz(self, filepath=None, launch_angle_deg=45):
        """
        결과를 NPZ 파일로 저장
        
        Parameters:
        -----------
        filepath : str, optional
            저장 경로
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
        
        if save_trajectory is None:
            print("⚠ trajectory_io 모듈이 없어 NPZ 저장 불가")
            return None
        
        # 파일 경로 자동 생성
        if filepath is None:
            os.makedirs('results_6dof', exist_ok=True)
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filepath = f"results_6dof/{self.missile_type}_{launch_angle_deg}deg_{timestamp}.npz"
        
        # Trajectory Frame에서 Body Frame으로 역변환 (근사)
        # V, gamma, psi -> u, v, w 변환
        V = self.results['velocity']
        gamma = self.results['gamma']
        psi = self.results['psi']
        phi = self.results['phi']
        theta = self.results['theta']
        
        # 간단한 변환 (정확하지 않지만 시각화용으로 충분)
        u = V * np.cos(gamma) * np.cos(psi)
        v = V * np.cos(gamma) * np.sin(psi)
        w = V * np.sin(gamma)
        
        # NPZ 저장
        save_trajectory(
            filepath=filepath,
            time=self.results['time'],
            position_x=self.results['x'],
            position_y=self.results['y'],
            position_z=self.results['h'],
            u=u,
            v=v,
            w=w,
            phi=self.results['phi'],
            theta=self.results['theta'],
            psi=self.results['psi_euler'],
            p=self.results['p'],
            q=self.results['q'],
            r=self.results['r'],
            mass=self.results['mass'],
            V=V,
            gamma=gamma,
            chi=psi,
            missile_type=self.missile_type,
            launch_angle=launch_angle_deg,
            alpha=self.results.get('alpha', np.zeros_like(self.results['time'])),
            beta=self.results.get('beta', np.zeros_like(self.results['time'])),
            mach=self.results.get('mach', np.zeros_like(self.results['time']))
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
                'velocity': data['V'],
                'gamma': data['gamma'],
                'psi': data['chi'],
                'x': data['position_x'],
                'y': data['position_y'],
                'h': data['position_z'],
                'phi': data['phi'],
                'theta': data['theta'],
                'psi_euler': data['psi'],
                'p': data['p'],
                'q': data['q'],
                'r': data['r'],
                'mass': data['mass'],
                'alpha': data.get('alpha', np.zeros_like(data['time'])),
                'beta': data.get('beta', np.zeros_like(data['time'])),
                'mach': data.get('mach', np.zeros_like(data['time'])),
                'fuel': data.get('propellant_mass', 0) - (data['mass'] - (data['mass'][0] - data.get('propellant_mass', 0)))
            }
            
            # 데이터 검증
            validate_trajectory(data)
            
            self.npz_path = filepath
            self.missile_type = str(data.get('missile_type', 'Unknown'))
            print(f"✓ NPZ 로드 완료: {filepath}")
            return True
            
        except Exception as e:
            print(f"⚠ NPZ 로드 실패: {e}")
            return False
    
    def plot_comprehensive(self, save_dir='results_6dof'):
        """
        12-subplot 종합 그래프 생성 (main_fixed.py 스타일)
        
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
        X_km = self.results['x'] / 1000
        Y_km = self.results['y'] / 1000
        H_km = self.results['h'] / 1000
        ax1.plot(X_km, Y_km, H_km, 'b-', linewidth=2)
        ax1.scatter([X_km[0]], [Y_km[0]], [H_km[0]], c='g', s=100, label='Start')
        ax1.scatter([X_km[-1]], [Y_km[-1]], [H_km[-1]], c='r', s=100, label='End')
        ax1.set_xlabel('X (km)', fontsize=9)
        ax1.set_ylabel('Y (km)', fontsize=9)
        ax1.set_zlabel('Altitude (km)', fontsize=9)
        ax1.set_title('3D Trajectory', fontsize=10, pad=10)
        ax1.legend(fontsize=7)
        ax1.tick_params(labelsize=8)
        
        # 2. 속도
        ax2 = fig.add_subplot(3, 4, 2)
        ax2.plot(self.results['time'], self.results['velocity'], 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)', fontsize=9)
        ax2.set_ylabel('Velocity (m/s)', fontsize=9)
        ax2.set_title('Velocity', fontsize=10, pad=10)
        ax2.grid(True)
        ax2.tick_params(labelsize=8)
        
        # 3. 고도
        ax3 = fig.add_subplot(3, 4, 3)
        ax3.plot(self.results['time'], H_km, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)', fontsize=9)
        ax3.set_ylabel('Altitude (km)', fontsize=9)
        ax3.set_title('Altitude', fontsize=10, pad=10)
        ax3.grid(True)
        ax3.tick_params(labelsize=8)
        
        # 4. 받음각 (핵심!)
        ax4 = fig.add_subplot(3, 4, 4)
        if 'alpha' in self.results:
            ax4.plot(self.results['time'], np.degrees(self.results['alpha']), 'r-', linewidth=2)
        else:
            # alpha가 없으면 theta - gamma로 근사
            alpha_approx = self.results['theta'] - self.results['gamma']
            ax4.plot(self.results['time'], np.degrees(alpha_approx), 'r-', linewidth=2)
        ax4.set_xlabel('Time (s)', fontsize=9)
        ax4.set_ylabel('Alpha (deg)', fontsize=9)
        ax4.set_title('Angle of Attack', fontsize=10, pad=10)
        ax4.grid(True)
        ax4.tick_params(labelsize=8)
        
        # 5. Euler 각도
        ax5 = fig.add_subplot(3, 4, 5)
        ax5.plot(self.results['time'], np.degrees(self.results['phi']), 'r-', label='Roll', linewidth=1.5)
        ax5.plot(self.results['time'], np.degrees(self.results['theta']), 'g-', label='Pitch', linewidth=1.5)
        ax5.plot(self.results['time'], np.degrees(self.results['psi_euler']), 'b-', label='Yaw', linewidth=1.5)
        ax5.set_xlabel('Time (s)', fontsize=9)
        ax5.set_ylabel('Angle (deg)', fontsize=9)
        ax5.set_title('Euler Angles', fontsize=10, pad=10)
        ax5.legend(fontsize=7)
        ax5.grid(True)
        ax5.tick_params(labelsize=8)
        
        # 6. 각속도 (핵심!)
        ax6 = fig.add_subplot(3, 4, 6)
        ax6.plot(self.results['time'], np.degrees(self.results['p']), 'r-', label='p (roll)', linewidth=1.5)
        ax6.plot(self.results['time'], np.degrees(self.results['q']), 'g-', label='q (pitch)', linewidth=1.5)
        ax6.plot(self.results['time'], np.degrees(self.results['r']), 'b-', label='r (yaw)', linewidth=1.5)
        ax6.set_xlabel('Time (s)', fontsize=9)
        ax6.set_ylabel('Rate (deg/s)', fontsize=9)
        ax6.set_title('Angular Rates', fontsize=10, pad=10)
        ax6.legend(fontsize=7)
        ax6.grid(True)
        ax6.tick_params(labelsize=8)
        
        # 7. 비행경로각 & 방위각
        ax7 = fig.add_subplot(3, 4, 7)
        ax7.plot(self.results['time'], np.degrees(self.results['gamma']), 'b-', label='Gamma', linewidth=1.5)
        ax7.plot(self.results['time'], np.degrees(self.results['psi']), 'r-', label='Psi', linewidth=1.5)
        ax7.set_xlabel('Time (s)', fontsize=9)
        ax7.set_ylabel('Angle (deg)', fontsize=9)
        ax7.set_title('Flight Path & Heading', fontsize=10, pad=10)
        ax7.legend(fontsize=7)
        ax7.grid(True)
        ax7.tick_params(labelsize=8)
        
        # 8. 마하수
        ax8 = fig.add_subplot(3, 4, 8)
        if 'mach' in self.results:
            ax8.plot(self.results['time'], self.results['mach'], 'm-', linewidth=2)
        else:
            # mach 계산
            mach_calc = self.results['velocity'] / 340.0  # 간단한 근사
            ax8.plot(self.results['time'], mach_calc, 'm-', linewidth=2)
        ax8.set_xlabel('Time (s)', fontsize=9)
        ax8.set_ylabel('Mach Number', fontsize=9)
        ax8.set_title('Mach Number', fontsize=10, pad=10)
        ax8.grid(True)
        ax8.tick_params(labelsize=8)
        
        # 9. 위상 평면 (α-q) - 시그니처 분석용!
        ax9 = fig.add_subplot(3, 4, 9)
        if 'alpha' in self.results:
            alpha_plot = self.results['alpha']
        else:
            alpha_plot = self.results['theta'] - self.results['gamma']
        ax9.plot(np.degrees(alpha_plot), np.degrees(self.results['q']), 'b-', linewidth=1.5)
        ax9.scatter([np.degrees(alpha_plot[0])], 
                   [np.degrees(self.results['q'][0])], c='g', s=50, label='Start', zorder=5)
        ax9.set_xlabel('Alpha (deg)', fontsize=9)
        ax9.set_ylabel('Pitch Rate q (deg/s)', fontsize=9)
        ax9.set_title('Phase Plane (α-q)', fontsize=10, pad=10)
        ax9.legend(fontsize=7)
        ax9.grid(True)
        ax9.tick_params(labelsize=8)
        
        # 10. Ground Track
        ax10 = fig.add_subplot(3, 4, 10)
        ax10.plot(X_km, Y_km, 'b-', linewidth=2)
        ax10.scatter([X_km[0]], [Y_km[0]], c='g', s=100, label='Start', zorder=5)
        ax10.scatter([X_km[-1]], [Y_km[-1]], c='r', s=100, label='End', zorder=5)
        ax10.set_xlabel('X (km)', fontsize=9)
        ax10.set_ylabel('Y (km)', fontsize=9)
        ax10.set_title('Ground Track', fontsize=10, pad=10)
        ax10.legend(fontsize=7)
        ax10.grid(True)
        ax10.axis('equal')
        ax10.tick_params(labelsize=8)
        
        # 11. Range vs Altitude
        ax11 = fig.add_subplot(3, 4, 11)
        range_km = np.sqrt(self.results['x']**2 + self.results['y']**2) / 1000
        ax11.plot(range_km, H_km, 'b-', linewidth=2)
        ax11.set_xlabel('Range (km)', fontsize=9)
        ax11.set_ylabel('Altitude (km)', fontsize=9)
        ax11.set_title('Range vs Altitude', fontsize=10, pad=10)
        ax11.grid(True)
        ax11.tick_params(labelsize=8)
        
        # 12. Roll-Yaw 상호작용 (시그니처 분석용!)
        ax12 = fig.add_subplot(3, 4, 12)
        ax12.plot(np.degrees(self.results['p']), np.degrees(self.results['r']), 'r-', linewidth=1.5)
        ax12.set_xlabel('Roll Rate p (deg/s)', fontsize=9)
        ax12.set_ylabel('Yaw Rate r (deg/s)', fontsize=9)
        ax12.set_title('Roll-Yaw Coupling', fontsize=10, pad=10)
        ax12.grid(True)
        ax12.tick_params(labelsize=8)
        
        plt.tight_layout(pad=3.0, h_pad=3.5, w_pad=3.5)
        
        # 저장
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(save_dir, f'visualization_6dof_{self.missile_type}_{timestamp}.png')
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"✓ 그래프 저장: {filepath}")
        plt.show()
        
        return filepath


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print("6DOF 미사일 시각화 시스템 (교수님 구조 기반)")
    print("=" * 60 + "\n")
    
    # 미사일 선택
    print("미사일 종류:")
    missile_list = ["SCUD-B", "KN-23", "Nodong"]
    for i, name in enumerate(missile_list, 1):
        print(f"  {i}. {name}")
    
    choice = input("\n미사일 선택 (1-3, 기본값: 1): ").strip()
    missile_idx = int(choice) - 1 if choice.isdigit() and 1 <= int(choice) <= len(missile_list) else 0
    missile_name = missile_list[missile_idx]
    
    print(f"\n선택: {missile_name}")
    
    # 시각화 객체 생성
    viz = MissileVisualization6DOF(missile_type=missile_name)
    
    # 실행 모드
    print("\n실행 모드:")
    print("  1. 새 시뮬레이션 실행")
    print("  2. NPZ 파일 로드")
    
    mode = input("\n모드 선택 (1-2, 기본값: 1): ").strip()
    
    if mode == '2':
        # NPZ 로드
        npz_file = input("NPZ 파일 경로: ").strip()
        if viz.load_from_npz(npz_file):
            viz.plot_comprehensive()
    else:
        # 새 시뮬레이션
        launch_angle = float(input("발사각 (도, 기본값: 45): ").strip() or "45")
        azimuth = float(input("방위각 (도, 기본값: 90): ").strip() or "90")
        
        # 시뮬레이션 실행
        if viz.run_simulation(
            launch_angle_deg=launch_angle,
            azimuth_deg=azimuth,
            sim_time=600
        ):
            # NPZ 저장
            npz_path = viz.save_to_npz(launch_angle_deg=launch_angle)
            
            # 시각화
            viz.plot_comprehensive()
            
            print(f"\n✓ 완료! NPZ 파일: {npz_path}")
        else:
            print("\n⚠ 시뮬레이션 실패")


if __name__ == "__main__":
    main()
