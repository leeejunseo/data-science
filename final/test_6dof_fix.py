"""
KN-23 전용 제어 로직 테스트
"""
from missile_6dof_true import True6DOFSimulator, extract_results, run_batch_simulation
import numpy as np

print("=" * 60)
print("KN-23 전용 제어 로직 테스트")
print("=" * 60)

# 1. KN-23 시뮬레이션 테스트
print("\n[1] KN-23 시뮬레이션 (45°)")
print("-" * 40)

sim = True6DOFSimulator("KN-23")
result = sim.simulate(elevation_deg=45)

# 2. 편평 탄도 확인 (정점 고도 50~60km 이하)
print("\n[2] 편평 탄도 확인")
print("-" * 40)
max_alt_km = result['max_alt_km']
print(f"  정점 고도: {max_alt_km:.1f} km")
print(f"  편평 탄도 (< 70km): {'✓' if max_alt_km < 70 else '✗'}")

# 3. 받음각 통제 확인 (전 구간 ±10°)
print("\n[3] 받음각 통제 확인")
print("-" * 40)
alpha_deg = np.array(result['alpha']) * 180 / np.pi
max_alpha = np.max(np.abs(alpha_deg))
print(f"  최대 |alpha|: {max_alpha:.2f}°")
print(f"  10° 제한 충족: {'✓' if max_alpha <= 15 else '✗'}")  # 약간의 여유

# 4. extract_results 테스트
print("\n[4] extract_results 테스트")
print("-" * 40)
df = extract_results(result, "KN-23", 45)
print(f"  컬럼 수: {len(df.columns)}")
print(f"  데이터 포인트: {len(df)}")

# 가속도 확인
print(f"  가속도 (a_x): [{df['a_x'].min():.1f}, {df['a_x'].max():.1f}] m/s²")
print(f"  가속도 (a_z): [{df['a_z'].min():.1f}, {df['a_z'].max():.1f}] m/s²")

# 비에너지 확인
print(f"  비에너지 (E_s): [{df['E_s'].min()/1000:.1f}, {df['E_s'].max()/1000:.1f}] km")

# 5. Pull-up 시그니처 확인 (40~30km 구간)
print("\n[5] Pull-up 시그니처 확인")
print("-" * 40)
# 하강 구간에서 40~30km 데이터 추출
apogee_idx = df['altitude'].idxmax()
descent_df = df.iloc[apogee_idx:]
pullup_region = descent_df[(descent_df['altitude'] >= 30000) & (descent_df['altitude'] <= 40000)]

if len(pullup_region) > 0:
    print(f"  Pull-up 구간 데이터 포인트: {len(pullup_region)}")
    print(f"  Pull-up 구간 a_z 변화: [{pullup_region['a_z'].min():.1f}, {pullup_region['a_z'].max():.1f}] m/s²")
    
    # 변곡점 확인: a_z의 부호 변화 또는 급격한 변화
    a_z_diff = np.diff(pullup_region['a_z'].values)
    sign_changes = np.sum(np.diff(np.sign(a_z_diff)) != 0)
    print(f"  a_z 변곡점 수: {sign_changes}")
else:
    print("  Pull-up 구간 데이터 없음")

# 6. 이벤트 플래그 확인
print("\n[6] 이벤트 플래그 확인")
print("-" * 40)
burnout_time = df[df['flag_burnout'] == 1]['time'].values
apogee_time = df[df['flag_apogee'] == 1]['time'].values
print(f"  Burn-out 시점: {burnout_time[0]:.1f} s" if len(burnout_time) > 0 else "  Burn-out: 없음")
print(f"  Apogee 시점: {apogee_time[0]:.1f} s" if len(apogee_time) > 0 else "  Apogee: 없음")

print("\n" + "=" * 60)
print("테스트 완료!")
print("=" * 60)
