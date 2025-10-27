#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3DOF vs 6DOF 비교 테스트 스크립트
"""

import sys
import os

def test_6dof():
    """6DOF 테스트"""
    print("=" * 70)
    print("🚀 6DOF 미사일 시뮬레이션 테스트")
    print("=" * 70)
    
    try:
        import config_6dof as cfg
        from main_6dof import MissileSimulation6DOF
        
        print("✅ 6DOF 모듈 로드 성공")
        print(f"   - 상태 벡터 차원: {cfg.StateVector6DOF.STATE_DIM}")
        print(f"   - 미사일 종류: {list(cfg.MISSILE_TYPES.keys())}")
        
        # 간단한 시뮬레이션
        print("\n📊 SCUD-B 6DOF 시뮬레이션 시작...")
        sim = MissileSimulation6DOF(missile_type="SCUD-B", apply_errors=False)
        sim.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=300)
        
        print("   - 시뮬레이션 실행 중...")
        results = sim.run_simulation()
        
        if results is not None:
            print("✅ 6DOF 시뮬레이션 성공!")
            
            import numpy as np
            final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
            max_altitude = np.max(results['h']) / 1000
            final_roll = np.rad2deg(results['phi'][-1])
            final_pitch = np.rad2deg(results['theta'][-1])
            
            print(f"\n📈 6DOF 결과 요약:")
            print(f"   - 최종 사거리: {final_range:.2f} km")
            print(f"   - 최대 고도: {max_altitude:.2f} km")
            print(f"   - 비행 시간: {results['time'][-1]:.2f} s")
            print(f"   - 최종 롤각: {final_roll:.2f}°")
            print(f"   - 최종 피치각: {final_pitch:.2f}°")
            
            return True
        else:
            print("❌ 6DOF 시뮬레이션 실패")
            return False
            
    except ImportError as e:
        print(f"❌ 6DOF 모듈 로드 실패: {e}")
        print("\n💡 해결 방법:")
        print("   1. config_6dof.py 파일이 있는지 확인")
        print("   2. main_6dof.py 파일이 있는지 확인")
        return False
    except Exception as e:
        print(f"❌ 6DOF 테스트 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return False

def compare_3dof_6dof():
    """3DOF와 6DOF 비교"""
    print("\n" + "=" * 70)
    print("🔍 3DOF vs 6DOF 상태 벡터 비교")
    print("=" * 70)
    
    print("\n3DOF 상태 벡터 (8차원):")
    print("   [V, γ, ψ, x, y, h, W, M]")
    print("   - V: 속도")
    print("   - γ: 비행경로각 (피치)")
    print("   - ψ: 방위각")
    print("   - x, y: 위치")
    print("   - h: 고도")
    print("   - W: 중량")
    print("   - M: 질량")
    
    print("\n6DOF 상태 벡터 (14차원):")
    print("   [V, γ, ψ, x, y, h, φ, θ, ψ_e, p, q, r, M, fuel]")
    print("   병진 운동 (기존):")
    print("     - V: 속도")
    print("     - γ: 비행경로각")
    print("     - ψ: 방위각")
    print("     - x, y: 위치")
    print("     - h: 고도")
    print("   🆕 회전 운동 (추가):")
    print("     - φ: 롤각 (Roll)")
    print("     - θ: 피치각 (Pitch)")
    print("     - ψ_e: 요각 (Yaw)")
    print("     - p: 롤 각속도")
    print("     - q: 피치 각속도")
    print("     - r: 요 각속도")
    print("   질량:")
    print("     - M: 질량")
    print("     - fuel: 연료 소모")
    
    print("\n✨ 6DOF 추가 기능:")
    print("   ✅ 회전 운동 시뮬레이션")
    print("   ✅ 공력 모멘트 계산")
    print("   ✅ 관성 모멘트 고려")
    print("   ✅ 오일러 각도 추적")
    print("   ✅ 각속도 추적")

def show_usage():
    """사용 방법 안내"""
    print("\n" + "=" * 70)
    print("📚 사용 방법")
    print("=" * 70)
    
    print("\n1️⃣ 6DOF 단독 실행:")
    print("   python3 main_6dof.py")
    
    print("\n2️⃣ 기존 3DOF 실행:")
    print("   python3 main.py")
    
    print("\n3️⃣ 이 테스트 스크립트 실행:")
    print("   python3 compare_3dof_6dof.py")
    
    print("\n4️⃣ 기존 코드를 6DOF로 전환:")
    print("   방법 A: 파일 교체 (가장 간단)")
    print("     cp config.py config_3dof_backup.py")
    print("     cp main.py main_3dof_backup.py")
    print("     cp config_6dof.py config.py")
    print("     cp main_6dof.py main.py")
    print("     python3 main.py  # 6DOF로 실행됨")
    
    print("\n   방법 B: Import 수정")
    print("     # main.py 첫 줄을")
    print("     import config as cfg")
    print("     # 다음으로 변경:")
    print("     import config_6dof as cfg")
    
    print("\n📖 자세한 내용은 CONVERSION_GUIDE.md 참고")

def main():
    """메인 함수"""
    print("🎯 3DOF → 6DOF 전환 테스트 및 비교")
    
    # 비교 설명
    compare_3dof_6dof()
    
    # 6DOF 테스트
    success = test_6dof()
    
    # 사용 방법 안내
    show_usage()
    
    if success:
        print("\n" + "=" * 70)
        print("✅ 모든 테스트 통과!")
        print("=" * 70)
        print("\n🚀 이제 6DOF 시뮬레이션을 사용할 수 있습니다!")
        print("   다음 명령어로 실행하세요:")
        print("   python3 main_6dof.py")
    else:
        print("\n" + "=" * 70)
        print("⚠️ 테스트 실패")
        print("=" * 70)
        print("\n💡 해결 방법:")
        print("   1. config_6dof.py와 main_6dof.py 파일이 있는지 확인")
        print("   2. 현재 디렉토리에서 실행했는지 확인")
        print("   3. Python 버전이 3.6 이상인지 확인")

if __name__ == "__main__":
    main()
