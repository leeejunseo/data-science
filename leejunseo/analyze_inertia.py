#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
관성 모멘트 상세 분석 도구
Detailed Inertia Analysis for Complete 6DOF
"""
import numpy as np
import matplotlib.pyplot as plt
from main_6dof_complete import Complete6DOFSimulation
import config_6dof_complete as cfg

def analyze_inertia_effects():
    """관성 모멘트 변화가 비행에 미치는 영향 분석"""

    print("=" * 70)
    print("관성 모멘트 상세 분석")
    print("=" * 70)

    # 시뮬레이션 실행
    sim = Complete6DOFSimulation(missile_type="SCUD-B", apply_errors=False)
    sim.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=300)

    print("\n시뮬레이션 실행 중...")
    results = sim.run_simulation()

    if results is None:
        print("시뮬레이션 실패")
        return

    # 그래프 생성
    fig = plt.figure(figsize=(16, 12))

    # 1. 관성 모멘트 변화 (개별)
    ax1 = fig.add_subplot(3, 3, 1)
    ax1.plot(results['time'], results['Ixx'], 'r-', linewidth=2, label='Ixx (Pitch/Yaw)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Ixx (kg·m²)')
    ax1.set_title('Pitch/Yaw Inertia (Ixx)')
    ax1.grid(True)
    ax1.legend()

    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(results['time'], results['Iyy'], 'g-', linewidth=2, label='Iyy (Pitch/Yaw)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Iyy (kg·m²)')
    ax2.set_title('Pitch/Yaw Inertia (Iyy)')
    ax2.grid(True)
    ax2.legend()

    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(results['time'], results['Izz'], 'b-', linewidth=2, label='Izz (Roll)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Izz (kg·m²)')
    ax3.set_title('Roll Inertia (Izz)')
    ax3.grid(True)
    ax3.legend()

    # 2. 관성 모멘트 비율 변화
    ax4 = fig.add_subplot(3, 3, 4)
    ratio_xy = results['Ixx'] / results['Iyy']
    ratio_xz = results['Ixx'] / results['Izz']
    ratio_yz = results['Iyy'] / results['Izz']
    ax4.plot(results['time'], ratio_xy, label='Ixx/Iyy')
    ax4.plot(results['time'], ratio_xz, label='Ixx/Izz')
    ax4.plot(results['time'], ratio_yz, label='Iyy/Izz')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Inertia Ratio')
    ax4.set_title('Inertia Ratios')
    ax4.grid(True)
    ax4.legend()

    # 3. 관성 모멘트 vs 각속도
    ax5 = fig.add_subplot(3, 3, 5)
    ax5_twin = ax5.twinx()
    ax5.plot(results['time'], results['Ixx'], 'r-', label='Ixx')
    ax5_twin.plot(results['time'], np.rad2deg(results['q']), 'b--', label='Pitch Rate (q)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Ixx (kg·m²)', color='r')
    ax5_twin.set_ylabel('Pitch Rate (deg/s)', color='b')
    ax5.set_title('Inertia vs Pitch Rate')
    ax5.grid(True)
    ax5.tick_params(axis='y', labelcolor='r')
    ax5_twin.tick_params(axis='y', labelcolor='b')

    # 4. 관성 모멘트 vs 질량
    ax6 = fig.add_subplot(3, 3, 6)
    ax6.scatter(results['mass'], results['Ixx'], c=results['time'], cmap='viridis', s=10)
    ax6.set_xlabel('Mass (kg)')
    ax6.set_ylabel('Ixx (kg·m²)')
    ax6.set_title('Inertia vs Mass')
    ax6.grid(True)
    cbar = plt.colorbar(ax6.collections[0], ax=ax6)
    cbar.set_label('Time (s)')

    # 5. 무게중심 vs 관성 모멘트
    ax7 = fig.add_subplot(3, 3, 7)
    ax7_twin = ax7.twinx()
    ax7.plot(results['time'], results['cg'], 'r-', label='CG')
    ax7_twin.plot(results['time'], results['Ixx'], 'b--', label='Ixx')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('CG Location (m)', color='r')
    ax7_twin.set_ylabel('Ixx (kg·m²)', color='b')
    ax7.set_title('CG vs Inertia')
    ax7.grid(True)
    ax7.tick_params(axis='y', labelcolor='r')
    ax7_twin.tick_params(axis='y', labelcolor='b')

    # 6. 관성 모멘트 변화율
    ax8 = fig.add_subplot(3, 3, 8)
    dIxx_dt = np.gradient(results['Ixx'], results['time'])
    dIyy_dt = np.gradient(results['Iyy'], results['time'])
    dIzz_dt = np.gradient(results['Izz'], results['time'])
    ax8.plot(results['time'], dIxx_dt, label='dIxx/dt')
    ax8.plot(results['time'], dIyy_dt, label='dIyy/dt')
    ax8.plot(results['time'], dIzz_dt, label='dIzz/dt')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Inertia Rate (kg·m²/s)')
    ax8.set_title('Inertia Change Rate')
    ax8.grid(True)
    ax8.legend()

    # 7. 연료 소모 vs 관성 감소
    ax9 = fig.add_subplot(3, 3, 9)
    fuel_consumed_fraction = results['fuel'] / sim.propellant_mass
    inertia_xx_fraction = results['Ixx'] / results['Ixx'][0]
    ax9.plot(fuel_consumed_fraction * 100, inertia_xx_fraction, 'o-', markersize=2)
    ax9.set_xlabel('Fuel Consumed (%)')
    ax9.set_ylabel('Ixx Fraction (normalized)')
    ax9.set_title('Fuel Burn vs Inertia Reduction')
    ax9.grid(True)
    ax9.set_xlim([0, 100])

    plt.tight_layout()

    # 저장
    import os
    os.makedirs("results_inertia_analysis", exist_ok=True)
    save_path = "results_inertia_analysis/inertia_analysis.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n관성 모멘트 분석 그래프 저장: {save_path}")

    plt.show()

    # 통계 출력
    print("\n" + "=" * 70)
    print("관성 모멘트 통계")
    print("=" * 70)

    print(f"\nIxx (피치/요 관성):")
    print(f"  초기값: {results['Ixx'][0]:.2f} kg·m²")
    print(f"  최종값: {results['Ixx'][-1]:.2f} kg·m²")
    print(f"  감소량: {results['Ixx'][0] - results['Ixx'][-1]:.2f} kg·m² ({(1 - results['Ixx'][-1]/results['Ixx'][0])*100:.1f}%)")

    print(f"\nIyy (피치/요 관성):")
    print(f"  초기값: {results['Iyy'][0]:.2f} kg·m²")
    print(f"  최종값: {results['Iyy'][-1]:.2f} kg·m²")
    print(f"  감소량: {results['Iyy'][0] - results['Iyy'][-1]:.2f} kg·m² ({(1 - results['Iyy'][-1]/results['Iyy'][0])*100:.1f}%)")

    print(f"\nIzz (롤 관성):")
    print(f"  초기값: {results['Izz'][0]:.2f} kg·m²")
    print(f"  최종값: {results['Izz'][-1]:.2f} kg·m²")
    print(f"  감소량: {results['Izz'][0] - results['Izz'][-1]:.2f} kg·m² ({(1 - results['Izz'][-1]/results['Izz'][0])*100:.1f}%)")

    print(f"\n무게중심 (CG):")
    print(f"  초기 위치: {results['cg'][0]:.2f} m")
    print(f"  최종 위치: {results['cg'][-1]:.2f} m")
    print(f"  이동 거리: {results['cg'][-1] - results['cg'][0]:.2f} m (뒤쪽)")

    print("=" * 70)

    return results

def compare_missiles_inertia():
    """여러 미사일의 관성 모멘트 비교"""

    print("\n" + "=" * 70)
    print("미사일별 관성 모멘트 비교")
    print("=" * 70)

    missiles = ["SCUD-B", "NODONG", "KN-23"]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    for missile_name in missiles:
        try:
            sim = Complete6DOFSimulation(missile_type=missile_name, apply_errors=False)
            sim.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=200)

            print(f"\n{missile_name} 시뮬레이션 실행 중...")
            results = sim.run_simulation()

            if results is not None:
                # Ixx
                axes[0].plot(results['time'], results['Ixx'], label=missile_name, linewidth=2)

                # Iyy
                axes[1].plot(results['time'], results['Iyy'], label=missile_name, linewidth=2)

                # Izz
                axes[2].plot(results['time'], results['Izz'], label=missile_name, linewidth=2)

        except Exception as e:
            print(f"  {missile_name} 시뮬레이션 실패: {e}")

    # 그래프 설정
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Ixx (kg·m²)')
    axes[0].set_title('Pitch/Yaw Inertia (Ixx) - Missile Comparison')
    axes[0].grid(True)
    axes[0].legend()

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Iyy (kg·m²)')
    axes[1].set_title('Pitch/Yaw Inertia (Iyy) - Missile Comparison')
    axes[1].grid(True)
    axes[1].legend()

    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Izz (kg·m²)')
    axes[2].set_title('Roll Inertia (Izz) - Missile Comparison')
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()

    import os
    os.makedirs("results_inertia_analysis", exist_ok=True)
    save_path = "results_inertia_analysis/missile_comparison.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n미사일 비교 그래프 저장: {save_path}")

    plt.show()

def main():
    """메인 함수"""
    print("🎯 관성 모멘트 상세 분석 도구\n")

    print("분석 옵션:")
    print("1. 단일 미사일 상세 분석 (9개 그래프)")
    print("2. 여러 미사일 비교")
    print("3. 전체 실행")

    choice = input("\n선택 (1-3, 기본값: 1): ").strip()

    if choice == "2":
        compare_missiles_inertia()
    elif choice == "3":
        analyze_inertia_effects()
        compare_missiles_inertia()
    else:
        analyze_inertia_effects()

    print("\n✅ 관성 모멘트 분석 완료!")

if __name__ == "__main__":
    main()
