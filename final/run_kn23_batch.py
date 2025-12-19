"""
KN-23 배치 시뮬레이션 실행 스크립트
발사각 15°~60°, 5° 간격으로 시뮬레이션 수행
결과를 CSV 파일로 저장 (가속도 a_x, a_z, 비에너지 E_s 포함)
"""
from missile_6dof_true import run_batch_simulation, extract_results, True6DOFSimulator
import pandas as pd
import numpy as np
from pathlib import Path

print("=" * 70)
print("KN-23 배치 시뮬레이션")
print("발사각: 15° ~ 60° (5° 간격)")
print("=" * 70)

# 출력 디렉토리
output_dir = Path(__file__).parent / "signature_datasets" / "KN-23"
output_dir.mkdir(parents=True, exist_ok=True)

# 배치 시뮬레이션 실행
results = run_batch_simulation(
    missile_type="KN-23",
    elevation_start=15,
    elevation_end=60,
    elevation_step=5,
    output_dir=str(output_dir.parent),
    file_format='csv',  # CSV 형식으로 저장
    verbose=True
)

# 결과 요약
print("\n" + "=" * 70)
print("배치 시뮬레이션 결과 요약")
print("=" * 70)

summary_data = []
for df in results:
    elev = df['elevation_deg'].iloc[0]
    max_alt = df['altitude'].max() / 1000
    max_alpha = np.max(np.abs(df['alpha'])) * 180 / np.pi
    max_E_s = df['E_s'].max() / 1000
    range_km = np.sqrt(df['x'].iloc[-1]**2 + df['y'].iloc[-1]**2) / 1000
    
    # Pull-up 구간 확인
    apogee_idx = df['altitude'].idxmax()
    descent_df = df.iloc[apogee_idx:]
    pullup_region = descent_df[(descent_df['altitude'] >= 25000) & (descent_df['altitude'] <= 50000)]
    has_pullup = len(pullup_region) > 0
    
    summary_data.append({
        'elevation_deg': elev,
        'range_km': range_km,
        'max_alt_km': max_alt,
        'max_alpha_deg': max_alpha,
        'max_E_s_km': max_E_s,
        'has_pullup': has_pullup,
        'n_points': len(df)
    })
    
    print(f"  {elev:2.0f}°: 사거리={range_km:6.1f}km, 정점={max_alt:5.1f}km, "
          f"|α|_max={max_alpha:4.1f}°, E_s_max={max_E_s:5.1f}km, Pull-up={'✓' if has_pullup else '✗'}")

# 요약 테이블 저장
summary_df = pd.DataFrame(summary_data)
summary_path = output_dir / "KN-23_batch_summary.csv"
summary_df.to_csv(summary_path, index=False)
print(f"\n✓ 요약 테이블 저장: {summary_path}")

# 통합 데이터셋 생성
print("\n통합 데이터셋 생성 중...")
all_data = pd.concat(results, ignore_index=True)

# 필수 컬럼 확인
required_cols = ['time', 'x', 'y', 'altitude', 'V', 'a_x', 'a_z', 'E_s', 
                 'alpha', 'theta', 'q', 'mach', 'flag_burnout', 'flag_apogee',
                 'phase', 'missile_type', 'elevation_deg']

print(f"  총 데이터 포인트: {len(all_data)}")
print(f"  컬럼 수: {len(all_data.columns)}")
print(f"  필수 컬럼 포함 여부:")
for col in required_cols:
    status = '✓' if col in all_data.columns else '✗'
    print(f"    {status} {col}")

# 통합 CSV 저장
merged_path = output_dir / "KN-23_all_elevations_dataset.csv"
all_data.to_csv(merged_path, index=False)
print(f"\n✓ 통합 데이터셋 저장: {merged_path}")

print("\n" + "=" * 70)
print("배치 시뮬레이션 완료!")
print("=" * 70)
