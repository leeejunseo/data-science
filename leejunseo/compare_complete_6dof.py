#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
기존 6DOF vs Complete 6DOF 비교 테스트
"""

import sys
import numpy as np

def test_basic_6dof():
    """기존 6DOF 테스트"""
    print("=" * 80)
    print("📊 기존 6DOF 시뮬레이션 테스트")
    print("=" * 80)

    try:
        import config_6dof as cfg_basic
        from main_6dof import MissileSimulation6DOF

        print("✅ 기존 6DOF 모듈 로드 성공")

        sim = MissileSimulation6DOF(missile_type="SCUD-B", apply_errors=False)
        sim.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=300)

        print("   시뮬레이션 실행 중...")
        results = sim.run_simulation()

        if results is not None:
            print("✅ 기존 6DOF 시뮬레이션 성공!")

            final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
            max_altitude = np.max(results['h']) / 1000

            print(f"\n📈 기존 6DOF 결과:")
            print(f"   - 최종 사거리: {final_range:.2f} km")
            print(f"   - 최대 고도: {max_altitude:.2f} km")
            print(f"   - 비행 시간: {results['time'][-1]:.2f} s")

            return results, True
        else:
            print("❌ 기존 6DOF 시뮬레이션 실패")
            return None, False

    except Exception as e:
        print(f"❌ 기존 6DOF 테스트 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return None, False

def test_complete_6dof():
    """Complete 6DOF 테스트"""
    print("\n" + "=" * 80)
    print("🚀 Complete 6DOF 시뮬레이션 테스트")
    print("=" * 80)

    try:
        import config_6dof_complete as cfg_complete
        from main_6dof_complete import Complete6DOFSimulation

        print("✅ Complete 6DOF 모듈 로드 성공")

        sim = Complete6DOFSimulation(missile_type="SCUD-B", apply_errors=False)
        sim.initialize_simulation(launch_angle_deg=45, azimuth_deg=90, sim_time=300)

        print("   시뮬레이션 실행 중...")
        results = sim.run_simulation()

        if results is not None:
            print("✅ Complete 6DOF 시뮬레이션 성공!")

            final_range = np.sqrt(results['x'][-1]**2 + results['y'][-1]**2) / 1000
            max_altitude = np.max(results['h']) / 1000

            print(f"\n📈 Complete 6DOF 결과:")
            print(f"   - 최종 사거리: {final_range:.2f} km")
            print(f"   - 최대 고도: {max_altitude:.2f} km")
            print(f"   - 비행 시간: {results['time'][-1]:.2f} s")
            print(f"   - 최종 받음각: {np.rad2deg(results['alpha'][-1]):.2f}°")
            print(f"   - 최종 측면 받음각: {np.rad2deg(results['beta'][-1]):.2f}°")
            print(f"   - 최종 무게중심: {results['cg'][-1]:.2f} m")

            return results, True
        else:
            print("❌ Complete 6DOF 시뮬레이션 실패")
            return None, False

    except Exception as e:
        print(f"❌ Complete 6DOF 테스트 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return None, False

def compare_implementations():
    """구현 방식 비교"""
    print("\n" + "=" * 80)
    print("🔍 기존 6DOF vs Complete 6DOF 구현 비교")
    print("=" * 80)

    comparison = [
        ("좌표계 변환 (DCM)", "❌ 없음", "✅ 완전 구현"),
        ("측면 받음각(β)", "❌ 항상 0", "✅ 동체 속도로 계산"),
        ("추력 방향", "❌ 속도 방향", "✅ 동체 x축"),
        ("공력 모멘트 팔", "❌ 미적용", "✅ CG-CP 거리 적용"),
        ("교차 결합 효과", "❌ 없음", "✅ 자이로 모멘트"),
        ("공기역학 계수", "⚠️  선형", "✅ 비선형 (실속 포함)"),
        ("질량 분포", "⚠️  고정", "✅ 연료 소모 반영"),
        ("관성 모멘트", "⚠️  고정", "✅ 연료 소모 반영"),
    ]

    print("\n{:<25} {:<20} {:<20}".format("항목", "기존 6DOF", "Complete 6DOF"))
    print("-" * 80)
    for item, basic, complete in comparison:
        print("{:<25} {:<20} {:<20}".format(item, basic, complete))

def show_physics_explanation():
    """물리적 차이 설명"""
    print("\n" + "=" * 80)
    print("📚 주요 물리 개선 사항 설명")
    print("=" * 80)

    print("\n1️⃣ 좌표계 변환 (DCM - Direction Cosine Matrix)")
    print("   기존: 추력과 공력이 어느 좌표계인지 불명확")
    print("   개선: 동체 → 지구 좌표계 명확한 변환")
    print("   효과: 회전 운동이 병진 운동에 제대로 영향")

    print("\n2️⃣ 측면 받음각(β)")
    print("   기존: beta = 0 (항상)")
    print("   개선: beta = arcsin(v_y / V) from body frame velocity")
    print("   효과: 요잉(yaw) 모멘트 정확한 계산")

    print("\n3️⃣ 추력 벡터")
    print("   기존: 추력이 속도 방향으로 작용 (비현실적)")
    print("   개선: 추력이 미사일 동체 x축을 따라 작용")
    print("   효과: 피치/요 각도에 따라 추력 방향이 달라짐")

    print("\n4️⃣ 공력 모멘트 팔")
    print("   기존: moment_arm 미적용")
    print("   개선: M_aero = q * S * (CP - CG) * Cm")
    print("   효과: 정확한 피칭 모멘트 크기")

    print("\n5️⃣ 자이로스코픽 효과")
    print("   기존: dp/dt = L_aero / Ixx")
    print("   개선: dp/dt = (L_aero + (Iyy - Izz) * q * r) / Ixx")
    print("   효과: 회전축 간 교차 결합")

    print("\n6️⃣ 비선형 공기역학")
    print("   기존: CL = CL_alpha * alpha (선형)")
    print("   개선: 천음속 충격파, 실속 효과 포함")
    print("   효과: 높은 받음각/마하수에서 현실적")

    print("\n7️⃣ 질량 분포 변화")
    print("   기존: CG, I 고정")
    print("   개선: 연료 소모에 따라 CG 이동, I 감소")
    print("   효과: 비행 중 안정성 변화 반영")

def show_usage():
    """사용 방법"""
    print("\n" + "=" * 80)
    print("📖 사용 방법")
    print("=" * 80)

    print("\n기존 6DOF 실행:")
    print("  python main_6dof.py")
    print("  또는")
    print("  python main_fixed.py")

    print("\n✨ Complete 6DOF 실행 (권장):")
    print("  python main_6dof_complete.py")

    print("\n비교 테스트:")
    print("  python compare_complete_6dof.py")

def main():
    """메인 함수"""
    print("\n🎯 기존 6DOF → Complete 6DOF 업그레이드 비교\n")

    # 구현 방식 비교
    compare_implementations()

    # 물리적 차이 설명
    show_physics_explanation()

    # 실제 테스트
    print("\n" + "=" * 80)
    print("🧪 실제 시뮬레이션 비교 테스트")
    print("=" * 80)

    basic_results, basic_success = test_basic_6dof()
    complete_results, complete_success = test_complete_6dof()

    # 결과 비교
    if basic_success and complete_success:
        print("\n" + "=" * 80)
        print("📊 결과 비교 분석")
        print("=" * 80)

        basic_range = np.sqrt(basic_results['x'][-1]**2 + basic_results['y'][-1]**2) / 1000
        complete_range = np.sqrt(complete_results['x'][-1]**2 + complete_results['y'][-1]**2) / 1000

        basic_alt = np.max(basic_results['h']) / 1000
        complete_alt = np.max(complete_results['h']) / 1000

        print(f"\n사거리:")
        print(f"  기존:     {basic_range:.2f} km")
        print(f"  Complete: {complete_range:.2f} km")
        print(f"  차이:     {abs(complete_range - basic_range):.2f} km ({abs(complete_range - basic_range)/basic_range*100:.1f}%)")

        print(f"\n최대 고도:")
        print(f"  기존:     {basic_alt:.2f} km")
        print(f"  Complete: {complete_alt:.2f} km")
        print(f"  차이:     {abs(complete_alt - basic_alt):.2f} km ({abs(complete_alt - basic_alt)/basic_alt*100:.1f}%)")

        print("\n💡 차이가 나는 이유:")
        print("  - 좌표계 변환으로 인한 정확한 힘 분해")
        print("  - 비선형 공기역학 계수")
        print("  - 연료 소모에 따른 질량 분포 변화")
        print("  - 자이로스코픽 효과")

    # 사용 방법
    show_usage()

    # 최종 권장사항
    print("\n" + "=" * 80)
    print("✅ 권장사항")
    print("=" * 80)
    print("\n진정한 6DOF 물리 시뮬레이션을 원한다면:")
    print("  👉 main_6dof_complete.py 사용을 강력히 권장합니다!")
    print("\n이유:")
    print("  ✅ 좌표계 변환 완전 구현")
    print("  ✅ 측면 받음각 정확한 계산")
    print("  ✅ 물리적으로 정확한 모델")
    print("  ✅ 연료 소모에 따른 질량 분포 변화")
    print("  ✅ 비선형 공기역학 효과")
    print("=" * 80)

if __name__ == "__main__":
    main()
