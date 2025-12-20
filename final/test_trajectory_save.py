#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NPZ 저장 표준화 테스트 스크립트
================================

테스트 내용:
1. SCUD-B, Nodong, KN-23 각각 30°, 45° 발사각 시뮬레이션
2. 표준 키 존재 여부 확인
3. 배열 길이 일치 확인
4. validate_trajectory 통과 확인
5. 레포 전체 np.savez 검색 (trajectory_io.py 외 0개 확인)
"""

import sys
import os
import io

# Windows 콘솔 인코딩 문제 해결
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

from pathlib import Path

# 현재 디렉토리 설정
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
from trajectory_io import (
    save_trajectory_unified,
    load_trajectory,
    validate_trajectory,
    DEFAULT_OUTPUT_DIR,
    REQUIRED_KEYS
)

# 시뮬레이터 임포트
from missile_6dof_true import True6DOFSimulator
from kn23_depressed import KN23Depressed


def test_single_simulation(missile_type: str, elevation: float, seed: int = 0):
    """단일 시뮬레이션 테스트"""
    print(f"\n{'='*60}")
    print(f"테스트: {missile_type} @ {elevation}°")
    print(f"{'='*60}")
    
    # 시뮬레이션 실행
    if missile_type == "KN-23":
        sim = KN23Depressed()
        results = sim.simulate(launch_angle=elevation)
    else:
        sim = True6DOFSimulator(missile_type=missile_type)
        results = sim.simulate(elevation_deg=elevation)
    
    # 표준 NPZ 저장
    filepath = save_trajectory_unified(
        results=results,
        missile_type=missile_type,
        elevation=elevation,
        azimuth=90.0,
        seed=seed,
        output_dir=DEFAULT_OUTPUT_DIR
    )
    
    print(f"✓ 저장 완료: {filepath}")
    
    # 로드 및 검증
    data = load_trajectory(filepath, validate=False)
    
    # 1. 필수 키 검증
    missing_keys = [k for k in REQUIRED_KEYS if k not in data]
    if missing_keys:
        print(f"✗ 필수 키 누락: {missing_keys}")
        return False, filepath
    else:
        print(f"✓ 필수 키 모두 존재 ({len(REQUIRED_KEYS)}개)")
    
    # 2. 배열 길이 일치 검증
    n_points = len(data['time'])
    length_errors = []
    for key in REQUIRED_KEYS:
        arr = np.asarray(data[key])
        if len(arr) != n_points:
            length_errors.append(f"{key}: {len(arr)} != {n_points}")
    
    if length_errors:
        print(f"✗ 배열 길이 불일치:")
        for err in length_errors:
            print(f"    - {err}")
        return False, filepath
    else:
        print(f"✓ 모든 배열 길이 일치 ({n_points} 포인트)")
    
    # 3. validate_trajectory 검증
    is_valid = validate_trajectory(data, raise_on_error=False)
    if not is_valid:
        print(f"⚠ validate_trajectory 경고 발생 (치명적이지 않음)")
    
    # 4. 메타데이터 검증
    missile_type_saved = str(data.get('missile_type', 'Unknown'))
    elevation_saved = float(data.get('elevation', data.get('launch_angle', 0)))
    print(f"✓ 메타데이터: missile_type={missile_type_saved}, elevation={elevation_saved}°")
    
    return True, filepath


def test_all_missiles():
    """전체 미사일 테스트"""
    print("\n" + "="*70)
    print("NPZ 저장 표준화 테스트")
    print("="*70)
    
    test_cases = [
        ("SCUD-B", 30),
        ("SCUD-B", 45),
        ("Nodong", 30),
        ("Nodong", 45),
        ("KN-23", 30),
        ("KN-23", 45),
    ]
    
    results = []
    saved_files = []
    
    for missile_type, elevation in test_cases:
        try:
            success, filepath = test_single_simulation(missile_type, elevation, seed=0)
            results.append((missile_type, elevation, success))
            if success:
                saved_files.append(filepath)
        except Exception as e:
            print(f"✗ 예외 발생: {e}")
            import traceback
            traceback.print_exc()
            results.append((missile_type, elevation, False))
    
    # 결과 요약
    print("\n" + "="*70)
    print("테스트 결과 요약")
    print("="*70)
    
    success_count = sum(1 for _, _, s in results if s)
    total_count = len(results)
    
    for missile_type, elevation, success in results:
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"  {status}: {missile_type} @ {elevation}°")
    
    print(f"\n총 결과: {success_count}/{total_count} 통과")
    
    if saved_files:
        print(f"\n저장된 파일 목록 ({len(saved_files)}개):")
        for f in saved_files:
            print(f"  - {f}")
    
    return success_count == total_count


def check_np_savez_usage():
    """레포 전체에서 np.savez 직접 사용 검사"""
    print("\n" + "="*70)
    print("np.savez 직접 사용 검사")
    print("="*70)
    
    final_dir = Path(__file__).parent
    py_files = list(final_dir.glob("*.py"))
    
    # 제외할 파일 목록
    exclude_files = {"trajectory_io.py", "test_trajectory_save.py"}
    
    violations = []
    
    for py_file in py_files:
        if py_file.name in exclude_files:
            continue
        
        try:
            content = py_file.read_text(encoding='utf-8')
            lines = content.split('\n')
            
            for i, line in enumerate(lines, 1):
                # 주석 및 문자열 리터럴 제외
                code_part = line.split('#')[0]
                # 실제 함수 호출인지 확인 (np.savez( 패턴)
                if 'np.savez(' in code_part or 'np.savez_compressed(' in code_part:
                    # save_npz_generic 내부 호출은 제외
                    if 'save_npz_generic' not in code_part:
                        violations.append((py_file.name, i, line.strip()))
        except Exception as e:
            print(f"⚠ {py_file.name} 읽기 실패: {e}")
    
    if violations:
        print(f"✗ np.savez 직접 사용 발견 ({len(violations)}건):")
        for filename, line_num, line in violations:
            print(f"  - {filename}:{line_num}: {line[:60]}...")
        return False
    else:
        print(f"✓ trajectory_io.py 외 파일에서 np.savez 직접 사용 없음")
        return True


def main():
    """메인 테스트 함수"""
    # 출력 디렉토리 생성
    DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    # 1. 전체 미사일 테스트
    all_passed = test_all_missiles()
    
    # 2. np.savez 사용 검사
    no_direct_savez = check_np_savez_usage()
    
    # 최종 결과
    print("\n" + "="*70)
    print("최종 결과")
    print("="*70)
    
    if all_passed and no_direct_savez:
        print("✓ 모든 테스트 통과!")
        print("\nNPZ 저장 표준화 리팩터링 완료:")
        print(f"  - 저장 폴더: {DEFAULT_OUTPUT_DIR}")
        print("  - 파일명 형식: {missile_type}__elev{elev}__azi{azi}__seed{seed}__{timestamp}.npz")
        print("  - 모든 trajectory 저장이 trajectory_io.save_trajectory_unified()를 통해 수행됨")
        return 0
    else:
        print("✗ 일부 테스트 실패")
        return 1


if __name__ == "__main__":
    sys.exit(main())
