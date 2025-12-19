"""
NPZ 파일 내용 확인 스크립트
"""
import numpy as np
import sys
from pathlib import Path


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
    print("NPZ 파일 뷰어")
    print("=" * 70)
    
    # 명령줄 인자가 있으면 해당 파일 열기
    if len(sys.argv) > 1:
        view_npz(sys.argv[1])
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
            view_npz(filepath)
