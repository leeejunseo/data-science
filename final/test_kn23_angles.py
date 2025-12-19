"""
KN-23 다양한 발사각에서 편평 탄도 테스트
"""
from missile_6dof_true import True6DOFSimulator
import numpy as np

print("=" * 70)
print("KN-23 발사각별 정점 고도 테스트")
print("=" * 70)

for elev in [15, 20, 25, 30, 35, 40, 45]:
    sim = True6DOFSimulator("KN-23")
    r = sim.simulate(elevation_deg=elev)
    
    alpha_max = np.max(np.abs(r['alpha'])) * 180 / np.pi
    print(f"  {elev:2d}°: 정점={r['max_alt_km']:6.1f}km, 사거리={r['range_km']:6.1f}km, |α|_max={alpha_max:.1f}°")

print("=" * 70)
