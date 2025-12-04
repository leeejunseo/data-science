"""빠른 테스트 스크립트"""
import numpy as np
from main_integrated import IntegratedMissileSimulation, get_missile_configs

# SCUD-B 설정
configs = get_missile_configs()
missile_config = configs['SCUD-B']

print(f"미사일: {missile_config['name']}")
print(f"발사 질량: {missile_config['launch_weight']} kg")
print(f"추진제 질량: {missile_config['propellant_mass']} kg")
print(f"연소 시간: {missile_config['burn_time']} s")
print(f"ISP (해면): {missile_config['isp_sea']} s")

# 예상 추력 계산
mdot = missile_config['propellant_mass'] / missile_config['burn_time']
thrust_expected = missile_config['isp_sea'] * mdot * 9.81
print(f"\n예상 추력: {thrust_expected:.0f} N ({thrust_expected/1000:.1f} kN)")
print(f"추력/중량비: {thrust_expected / (missile_config['launch_weight'] * 9.81):.2f}")

# 시뮬레이션 실행
sim = IntegratedMissileSimulation(missile_config)
success = sim.run_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=600)

if success:
    npz_path = sim.save_to_npz(launch_angle_deg=45)
    sim.plot_comprehensive()
