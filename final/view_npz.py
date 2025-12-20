"""
NPZ 파일 시각화 스크립트
- 미사일 비행 데이터를 그래프로 표시
- game_launcher.py에서 호출됨
"""
import numpy as np
import sys
from pathlib import Path
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def plot_npz_graphs(filepath):
    """NPZ 파일에서 그래프 표시"""
    
    if not Path(filepath).exists():
        print(f"❌ 파일을 찾을 수 없습니다: {filepath}")
        return
    
    # NPZ 파일 로드
    data = np.load(filepath, allow_pickle=True)
    
    # 미사일 타입 추출
    missile_type = "Unknown"
    if 'missile_type' in data.files:
        missile_type = str(data['missile_type'])
    else:
        # 파일명에서 추출
        fname = Path(filepath).name
        if 'SCUD' in fname:
            missile_type = "SCUD-B"
        elif 'Nodong' in fname:
            missile_type = "Nodong"
        elif 'KN-23' in fname:
            missile_type = "KN-23"
    
    # 데이터 추출
    time = data['time'] if 'time' in data.files else np.array([])
    pos_x = data['position_x'] if 'position_x' in data.files else np.array([])
    pos_y = data['position_y'] if 'position_y' in data.files else np.array([])
    pos_z = data['position_z'] if 'position_z' in data.files else np.array([])
    V = data['V'] if 'V' in data.files else np.array([])
    alpha = data['alpha'] if 'alpha' in data.files else np.array([])
    gamma = data['gamma'] if 'gamma' in data.files else np.array([])
    mach = data['mach'] if 'mach' in data.files else np.array([])
    theta = data['theta'] if 'theta' in data.files else np.array([])
    
    # 거리 계산 (km)
    range_km = np.sqrt(pos_x**2 + pos_y**2) / 1000 if len(pos_x) > 0 else np.array([])
    alt_km = pos_z / 1000 if len(pos_z) > 0 else np.array([])
    
    # 그래프 생성 (2x3 레이아웃)
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'미사일 비행 분석: {missile_type}\n({Path(filepath).name})', fontsize=14, fontweight='bold')
    
    # 1. Altitude vs Range (가장 중요!)
    ax1 = axes[0, 0]
    if len(range_km) > 0 and len(alt_km) > 0:
        ax1.plot(range_km, alt_km, 'b-', linewidth=2)
        ax1.set_xlabel('Range (km)')
        ax1.set_ylabel('Altitude (km)')
        ax1.set_title('고도 vs 사거리')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=70, color='r', linestyle='--', alpha=0.5, label='KN-23 기준 (70km)')
        ax1.legend()
        # 최대 고도 표시
        max_alt_idx = np.argmax(alt_km)
        ax1.annotate(f'Max: {alt_km[max_alt_idx]:.1f}km', 
                    xy=(range_km[max_alt_idx], alt_km[max_alt_idx]),
                    xytext=(10, 10), textcoords='offset points',
                    fontsize=10, color='red')
    
    # 2. Altitude vs Time
    ax2 = axes[0, 1]
    if len(time) > 0 and len(alt_km) > 0:
        ax2.plot(time, alt_km, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Altitude (km)')
        ax2.set_title('고도 vs 시간')
        ax2.grid(True, alpha=0.3)
    
    # 3. Velocity vs Time
    ax3 = axes[0, 2]
    if len(time) > 0 and len(V) > 0:
        ax3.plot(time, V, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.set_title('속도 vs 시간')
        ax3.grid(True, alpha=0.3)
        # 마하 수 보조 축
        ax3_mach = ax3.twinx()
        ax3_mach.set_ylabel('Mach', color='orange')
        if len(mach) > 0:
            ax3_mach.plot(time, mach, 'orange', linestyle='--', alpha=0.7)
    
    # 4. Alpha (받음각) vs Time - KN-23 식별에 중요!
    ax4 = axes[1, 0]
    if len(time) > 0 and len(alpha) > 0:
        alpha_deg = np.degrees(alpha) if np.max(np.abs(alpha)) < 2 else alpha
        ax4.plot(time, alpha_deg, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Alpha (deg)')
        ax4.set_title('받음각 (Alpha) vs 시간')
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # 5. Gamma (비행경로각) vs Time
    ax5 = axes[1, 1]
    if len(time) > 0 and len(gamma) > 0:
        gamma_deg = np.degrees(gamma) if np.max(np.abs(gamma)) < 2 else gamma
        ax5.plot(time, gamma_deg, 'c-', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Gamma (deg)')
        ax5.set_title('비행경로각 (Gamma) vs 시간')
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # 6. 3D Trajectory
    ax6 = axes[1, 2]
    ax6.remove()
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    if len(pos_x) > 0 and len(pos_y) > 0 and len(pos_z) > 0:
        ax6.plot(pos_x/1000, pos_y/1000, pos_z/1000, 'b-', linewidth=2)
        ax6.set_xlabel('X (km)')
        ax6.set_ylabel('Y (km)')
        ax6.set_zlabel('Z (km)')
        ax6.set_title('3D 궤적')
    
    # 통계 정보 텍스트
    stats_text = f"최대 고도: {np.max(alt_km):.1f} km\n"
    stats_text += f"최대 사거리: {np.max(range_km):.1f} km\n"
    stats_text += f"비행 시간: {time[-1]:.1f} s\n"
    if len(V) > 0:
        stats_text += f"최대 속도: {np.max(V):.0f} m/s (M{np.max(V)/340:.1f})"
    
    fig.text(0.02, 0.02, stats_text, fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.92, bottom=0.1)
    
    print(f"✓ 그래프 표시: {missile_type}")
    plt.show()
    
    data.close()


def view_npz(filepath):
    """NPZ 파일 내용 출력"""
    
    if not Path(filepath).exists():
        print(f"❌ 파일을 찾을 수 없습니다: {filepath}")
        return
    
    print("=" * 70)
    print(f"NPZ 파일: {filepath}")
    print("=" * 70)
    
    # NPZ 파일 로드
    data = np.load(filepath, allow_pickle=True)
    
    # 1. 포함된 변수 목록
    print("\n📋 포함된 변수 목록:")
    print("-" * 70)
    for i, key in enumerate(data.files, 1):
        print(f"{i:2d}. {key}")
    
    # 2. 메타데이터
    print("\n📊 메타데이터:")
    print("-" * 70)
    meta_keys = ['missile_type', 'launch_angle', 'generation_time', 'n_points', 'duration']
    for key in meta_keys:
        if key in data.files:
            value = data[key]
            if isinstance(value, np.ndarray) and value.size == 1:
                value = value.item()
            print(f"  {key:20s}: {value}")
    
    # 3. 배열 데이터 정보
    print("\n📈 배열 데이터 정보:")
    print("-" * 70)
    print(f"{'변수명':<20s} {'타입':<15s} {'크기':<10s} {'최소값':<15s} {'최대값':<15s}")
    print("-" * 70)
    
    array_keys = ['time', 'position_x', 'position_y', 'position_z', 
                  'u', 'v', 'w', 'phi', 'theta', 'psi', 
                  'p', 'q', 'r', 'mass', 'V', 'gamma', 'chi',
                  'alpha', 'beta', 'mach',
                  'a_x', 'a_y', 'a_z', 'E_s', 'q_dyn']  # 시그니처 분석용
    
    for key in array_keys:
        if key in data.files:
            arr = data[key]
            if isinstance(arr, np.ndarray) and arr.size > 1:
                print(f"{key:<20s} {str(arr.dtype):<15s} {len(arr):<10d} {arr.min():<15.2f} {arr.max():<15.2f}")
    
    # 4. 샘플 데이터 (처음 5개, 마지막 5개)
    print("\n🔍 샘플 데이터 (시간 기준):")
    print("-" * 70)
    
    if 'time' in data.files:
        time = data['time']
        n = len(time)
        
        print(f"\n처음 5개 데이터 포인트:")
        print(f"{'Index':<8s} {'Time(s)':<12s} {'Altitude(m)':<15s} {'Velocity(m/s)':<15s}")
        print("-" * 50)
        for i in range(min(5, n)):
            t = time[i]
            h = data['position_z'][i] if 'position_z' in data.files else 0
            v = data['V'][i] if 'V' in data.files else 0
            print(f"{i:<8d} {t:<12.2f} {h:<15.2f} {v:<15.2f}")
        
        print(f"\n마지막 5개 데이터 포인트:")
        print(f"{'Index':<8s} {'Time(s)':<12s} {'Altitude(m)':<15s} {'Velocity(m/s)':<15s}")
        print("-" * 50)
        for i in range(max(0, n-5), n):
            t = time[i]
            h = data['position_z'][i] if 'position_z' in data.files else 0
            v = data['V'][i] if 'V' in data.files else 0
            print(f"{i:<8d} {t:<12.2f} {h:<15.2f} {v:<15.2f}")
    
    # 5. 통계 요약
    print("\n📊 주요 통계:")
    print("-" * 70)
    
    stats = {}
    if 'time' in data.files:
        stats['비행 시간'] = f"{data['time'][-1]:.2f} s"
    if 'position_z' in data.files:
        stats['최대 고도'] = f"{data['position_z'].max()/1000:.2f} km"
    if 'V' in data.files:
        stats['최대 속도'] = f"{data['V'].max():.2f} m/s (마하 {data['V'].max()/340:.2f})"
    if 'position_x' in data.files and 'position_y' in data.files:
        final_range = np.sqrt(data['position_x'][-1]**2 + data['position_y'][-1]**2)
        stats['최종 거리'] = f"{final_range/1000:.2f} km"
    
    # 시그니처 분석용 통계
    if 'a_x' in data.files:
        stats['가속도 a_x 범위'] = f"[{data['a_x'].min():.1f}, {data['a_x'].max():.1f}] m/s²"
    if 'a_z' in data.files:
        stats['가속도 a_z 범위'] = f"[{data['a_z'].min():.1f}, {data['a_z'].max():.1f}] m/s²"
    if 'E_s' in data.files:
        stats['비에너지 E_s 최대'] = f"{data['E_s'].max()/1000:.2f} km"
    if 'q_dyn' in data.files:
        stats['동압 q_dyn 최대'] = f"{data['q_dyn'].max()/1000:.2f} kPa"
    
    for key, value in stats.items():
        print(f"  {key:20s}: {value}")
    
    print("\n" + "=" * 70)
    print("✓ 완료")
    print("=" * 70)
    
    data.close()


def list_npz_files(directory='results_6dof'):
    """디렉토리 내 NPZ 파일 목록 출력"""
    
    path = Path(directory)
    if not path.exists():
        print(f"❌ 디렉토리를 찾을 수 없습니다: {directory}")
        return
    
    npz_files = list(path.glob('*.npz'))
    
    if not npz_files:
        print(f"❌ NPZ 파일이 없습니다: {directory}")
        return
    
    print("\n" + "=" * 70)
    print(f"📁 NPZ 파일 목록 ({directory})")
    print("=" * 70)
    
    for i, file in enumerate(sorted(npz_files), 1):
        size_kb = file.stat().st_size / 1024
        print(f"{i:2d}. {file.name:<50s} ({size_kb:>8.1f} KB)")
    
    print("=" * 70)


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("NPZ 파일 시각화")
    print("=" * 70)
    
    # 명령줄 인자가 있으면 해당 파일의 그래프 표시
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
        print(f"📊 그래프 표시: {filepath}")
        plot_npz_graphs(filepath)
    else:
        # 없으면 목록 표시 후 선택
        list_npz_files()
        
        print("\n파일 경로를 입력하세요 (예: results_6dof/SCUD-B_45deg_xxx.npz)")
        print("또는 Enter를 눌러 가장 최근 파일 열기")
        
        filepath = input("\n파일 경로: ").strip()
        
        if not filepath:
            # 가장 최근 파일 자동 선택
            path = Path('results_6dof')
            if path.exists():
                npz_files = sorted(path.glob('*.npz'), key=lambda x: x.stat().st_mtime, reverse=True)
                if npz_files:
                    filepath = str(npz_files[0])
                    print(f"\n가장 최근 파일 선택: {filepath}")
        
        if filepath:
            plot_npz_graphs(filepath)
