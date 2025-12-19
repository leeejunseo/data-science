from missile_6dof_FINAL_CORRECTED import Missile6DOF_Authentic

print("테스트 시작...")
m = Missile6DOF_Authentic('Nodong')
print("초기화 성공")

result = m.simulate(elevation_deg=45, azimuth_deg=90, add_disturbance=False)
print(f"시뮬레이션 완료: {len(result['time'])} 포인트")
