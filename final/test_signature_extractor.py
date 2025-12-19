"""
시그니처 추출 모듈 테스트 스크립트
"""

import sys
from pathlib import Path

# 단일 시뮬레이션 테스트
print("=" * 60)
print("시그니처 추출 모듈 테스트")
print("=" * 60)

from missile_6dof_true import True6DOFSimulator, MISSILES
from signature_extractor import (
    extract_results,
    compute_additional_features,
    save_signature_data,
    run_batch_simulation,
    print_signature_summary,
    get_signature_summary
)

# 1. 단일 시뮬레이션 테스트
print("\n[1] 단일 시뮬레이션 테스트 (SCUD-B @ 45°)")
print("-" * 40)

sim = True6DOFSimulator("SCUD-B")
result = sim.simulate(elevation_deg=45)

# 시그니처 데이터 추출
df = extract_results(result, "SCUD-B", 45, include_reference=True)
df = compute_additional_features(df)

print(f"\n추출된 컬럼 ({len(df.columns)}개):")
print("-" * 40)

# 컬럼 분류별 출력
sig_cols = ['a_u', 'a_v', 'a_w', 'a_x', 'a_y', 'a_z', 'a_total', 'E_s']
flag_cols = ['flag_burnout', 'flag_apogee', 'phase']
ref_cols = [c for c in df.columns if c.startswith('_ref')]

print("시그니처 컬럼:", sig_cols)
print("이벤트 플래그:", flag_cols)
print("참조용 컬럼:", ref_cols)

# 요약 출력
print_signature_summary(df)

# 2. 데이터 저장 테스트
print("\n[2] 데이터 저장 테스트")
print("-" * 40)

output_dir = Path(__file__).parent / "test_output"
csv_path, npz_path = save_signature_data(df, str(output_dir), file_format='both')

# 저장된 파일 확인
if csv_path:
    import pandas as pd
    df_loaded = pd.read_csv(csv_path)
    print(f"  CSV 로드 확인: {len(df_loaded)} rows, {len(df_loaded.columns)} cols")

if npz_path:
    import numpy as np
    data = np.load(npz_path)
    print(f"  NPZ 로드 확인: {list(data.keys())[:5]}...")

# 3. 배치 시뮬레이션 테스트 (작은 범위)
print("\n[3] 배치 시뮬레이션 테스트 (SCUD-B, 45°~55°, 5° 간격)")
print("-" * 40)

batch_results = run_batch_simulation(
    missile_type="SCUD-B",
    elevation_start=45,
    elevation_end=55,
    elevation_step=5,
    output_dir=str(output_dir),
    file_format='csv'
)

print(f"\n배치 결과: {len(batch_results)}개 시뮬레이션 완료")

# 4. 시그니처 데이터 검증
print("\n[4] 시그니처 데이터 검증")
print("-" * 40)

df_test = batch_results[0]

# 가속도 범위 확인
print(f"  Body 가속도 범위:")
print(f"    a_u: [{df_test['a_u'].min():.1f}, {df_test['a_u'].max():.1f}] m/s²")
print(f"    a_v: [{df_test['a_v'].min():.1f}, {df_test['a_v'].max():.1f}] m/s²")
print(f"    a_w: [{df_test['a_w'].min():.1f}, {df_test['a_w'].max():.1f}] m/s²")

# 비에너지 확인
print(f"  비에너지 (E_s):")
print(f"    최소: {df_test['E_s'].min()/1000:.1f} km")
print(f"    최대: {df_test['E_s'].max()/1000:.1f} km")

# 이벤트 플래그 확인
burnout_idx = df_test[df_test['flag_burnout'] == 1].index
apogee_idx = df_test[df_test['flag_apogee'] == 1].index

print(f"  이벤트 플래그:")
print(f"    Burn-out 시점: {df_test.loc[burnout_idx, 'time'].values[0]:.1f} s" if len(burnout_idx) > 0 else "    Burn-out: 없음")
print(f"    Apogee 시점: {df_test.loc[apogee_idx, 'time'].values[0]:.1f} s" if len(apogee_idx) > 0 else "    Apogee: 없음")

print("\n" + "=" * 60)
print("테스트 완료!")
print("=" * 60)
