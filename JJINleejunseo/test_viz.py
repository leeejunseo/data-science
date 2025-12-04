"""시각화 빠른 테스트"""
from main_visualization import MissileVisualization6DOF

# SCUD-B 시각화 객체 생성
viz = MissileVisualization6DOF(missile_type="SCUD-B")

# 시뮬레이션 실행
print("시뮬레이션 시작...")
success = viz.run_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=600)

if success:
    # NPZ 저장
    npz_path = viz.save_to_npz(launch_angle_deg=45)
    print(f"\nNPZ 저장: {npz_path}")
    
    # 시각화
    print("\n그래프 생성 중...")
    viz.plot_comprehensive()
    
    print("\n✓ 완료!")
else:
    print("\n✗ 시뮬레이션 실패")
