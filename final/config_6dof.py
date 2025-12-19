# config_6dof.py - 6DOF 탄도 미사일 시뮬레이션 설정
# 교수님 config.py 기반 + 6DOF 추가 파라미터 (리서치 검증됨)

import numpy as np

# =============================================================================
# 단위 환산
# =============================================================================
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# =============================================================================
# 물리 상수 (WGS84 기준)
# =============================================================================
G = 9.80665           # 표준 중력가속도 (m/s²)
R_EARTH = 6371000     # 지구 평균 반지름 (m)
R = R_EARTH           # main.py 호환성
GM_EARTH = 3.986004418e14  # 지구 중력 매개변수 (m³/s²)

# =============================================================================
# 대기 상수
# =============================================================================
AIR_GAS_CONSTANT = 287.0531        # 건조 공기 기체상수 (J/(kg·K))
STD_TEMPERATURE_SEA_LEVEL = 288.15 # 해면 표준온도 (K)
STD_PRESSURE_SEA_LEVEL = 101325.0  # 해면 표준기압 (Pa)
STD_DENSITY_SEA_LEVEL = 1.225      # 해면 표준밀도 (kg/m³)
EXOSPHERE_HEIGHT = 600000          # 외기권 시작 고도 (m)
EXOSPHERE_ALTITUDE = EXOSPHERE_HEIGHT

# main.py 호환
STD_GRAVITY = G
AIR_MOLAR_MASS = 0.0289644
UNIVERSAL_GAS_CONSTANT = 8.314462

# =============================================================================
# 대기층 정의 (ISA 1976)
# =============================================================================
ATMOSPHERIC_LAYERS = [
    (0, 11000, -0.0065, 0, 288.15),
    (11000, 20000, 0, 11000, 216.65),
    (20000, 32000, 0.001, 20000, 216.65),
    (32000, 47000, 0.0028, 32000, 228.65),
    (47000, 51000, 0, 47000, 270.65),
    (51000, 71000, -0.0028, 51000, 270.65),
    (71000, 84852, -0.002, 71000, 214.65)
]

HIGH_ALTITUDE_REFERENCE = [
    (100000, 0.0000552, 8500),
    (120000, 0.0000024, 9000),
    (150000, 0.0000002, 10000)
]

# =============================================================================
# 공력 계수 (3DOF 호환)
# =============================================================================
CL_VERTICAL = 0.0
CL_PITCH = 0.5
CL_CONSTANT = 0.3
CL_TERMINAL = 0.1
K = 0.05  # 유도항력 계수

# 추진 관련
TSFC = 0.0004  # 추력비연료소모율 (kg/N/s)

# =============================================================================
# 미사일 타입별 항력계수 테이블 (KAIST 검증)
# =============================================================================
MISSILE_CD_TABLES = {
    'SCUD-B': {
        'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
        'cd': np.array([0.32, 0.36, 0.47, 0.82, 0.62, 0.42, 0.37, 0.32, 0.30, 0.27])
    },
    'Nodong': {
        'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
        'cd': np.array([0.28, 0.33, 0.43, 0.78, 0.58, 0.38, 0.33, 0.28, 0.26, 0.23])
    },
    'KN-23': {
        'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
        'cd': np.array([0.35, 0.38, 0.48, 0.85, 0.65, 0.45, 0.40, 0.35, 0.33, 0.30])
    }
}

# 기본 항력계수 테이블 (호환성)
BASE_CD_TABLE = {0: 0.3, 1: 0.8, 2: 1.2, 3: 1.0, 4: 0.9, 5: 0.8}

# =============================================================================
# 미사일 정보 (교수님 + 6DOF 파라미터)
# =============================================================================
MISSILE_TYPES = {
    "SCUD-B": {
        # === 기본 정보 (교수님 동일) ===
        "name": "SCUD-B",
        "diameter": 0.88,           # m
        "length": 10.94,            # m
        "nozzle_diameter": 0.6,     # m
        "launch_weight": 5860,      # kg
        "payload": 985,             # kg
        "propellant_mass": 4875,    # kg
        "range_km": 300,            # km (참조용)
        
        # === 추진 정보 (교수님 동일) ===
        "propellant_type": "LIQUID",
        "isp_sea": 230,             # s (해면 비추력)
        "isp_vac": 258,             # s (진공 비추력)
        "burn_time": 65,            # s
        
        # === 비행 프로그램 (교수님 동일) ===
        "vertical_time": 10,        # s
        "pitch_time": 15,           # s
        "pitch_angle_deg": 20,      # deg
        
        # === 공기역학 (교수님 동일) ===
        "reference_area": np.pi * (0.88/2)**2,  # m² (0.608)
        "cd_base": 0.25,
        
        # === 6DOF 추가 파라미터 (수치 안정성 최적화) ===
        "I_xx": 500,                # kg·m² (롤)
        "I_yy": 45000,              # kg·m² (피치)
        "I_zz": 45000,              # kg·m² (요)
        
        # 정적 안정성: 수치 안정성 위해 작은 값 사용
        # FFT 신호는 연소종료 교란으로 생성
        "C_m_alpha": -0.3,          # /rad (작은 값)
        
        # 동적 안정성/댐핑
        "C_mq": -2.0,               # /rad (감소)
        "C_nr": -2.0,               # /rad
        "C_lp": -0.3,               # /rad
        
        # 양력 기울기
        "CL_alpha": 4.5,            # /rad
        "CY_beta": -0.5,            # /rad
        
        # 비행 프로그램 게인
        "K_theta": 0.5,             # 피치 제어 게인 (감소)
    },
    
    "Nodong": {
        # === 기본 정보 (교수님 동일) ===
        "name": "노동 1호",
        "diameter": 1.36,           # m
        "length": 16.4,             # m
        "nozzle_diameter": 0.8,     # m
        "launch_weight": 16500,     # kg
        "payload": 1200,            # kg
        "propellant_mass": 15300,   # kg
        "range_km": 1500,           # km
        
        # === 추진 정보 (교수님 동일) ===
        "propellant_type": "UDMH/RFNA",
        "isp_sea": 255,             # s
        "isp_vac": 280,             # s (isp_vacuum)
        "isp_vacuum": 280,          # 호환성
        "burn_time": 70,            # s
        
        # === 비행 프로그램 (교수님 동일) ===
        "vertical_time": 10,        # s
        "pitch_time": 20,           # s
        "pitch_angle_deg": 15,      # deg
        
        # === 공기역학 (교수님 동일) ===
        "reference_area": np.pi * (1.36/2)**2,  # m² (1.452)
        "cd_base": 0.22,
        
        # === 6DOF 추가 파라미터 (수치 안정성 최적화) ===
        "I_xx": 2000,               # kg·m²
        "I_yy": 200000,             # kg·m²
        "I_zz": 200000,             # kg·m²
        
        "C_m_alpha": -0.2,          # /rad (작은 값)
        "C_mq": -2.0,               # /rad
        "C_nr": -2.0,               # /rad
        "C_lp": -0.3,               # /rad
        
        "CL_alpha": 4.0,            # /rad
        "CY_beta": -0.5,            # /rad
        
        "K_theta": 0.3,             # 더 큰 관성 = 더 낮은 게인
    },
    
    "KN-23": {
        # === 기본 정보 (교수님 동일) ===
        "name": "KN-23",
        "diameter": 0.95,           # m
        "length": 7.5,              # m
        "nozzle_diameter": 0.5,     # m
        "launch_weight": 3415,      # kg
        "payload": 500,             # kg
        "propellant_mass": 2915,    # kg
        "range_km": 690,            # km
        
        # === 추진 정보 (교수님 동일) ===
        "propellant_type": "SOLID",
        "isp_sea": 260,             # s
        "isp_vac": 265,             # s (고체는 차이 적음)
        "burn_time": 40,            # s
        
        # === 비행 프로그램 (교수님 동일) ===
        "vertical_time": 6,         # s (고체 = 빠른 초기 가속)
        "pitch_time": 10,           # s
        "pitch_angle_deg": 25,      # deg (quasi-ballistic = 큰 피치)
        
        # === 공기역학 (교수님 동일) ===
        "reference_area": np.pi * (0.95/2)**2,  # m² (0.709)
        "cd_base": 0.28,
        
        # === 6DOF 추가 파라미터 (수치 안정성 최적화) ===
        "I_xx": 300,                # kg·m² (짧은 미사일)
        "I_yy": 15000,              # kg·m²
        "I_zz": 15000,              # kg·m²
        
        "C_m_alpha": -0.4,          # /rad (고기동 = 약간 더 높은 안정성)
        "C_mq": -2.0,               # /rad
        "C_nr": -2.0,               # /rad
        "C_lp": -0.3,               # /rad
        
        "CL_alpha": 5.0,            # /rad (더 높은 기동성)
        "CY_beta": -0.6,            # /rad
        
        "K_theta": 0.8,             # 빠른 응답 필요
    }
}

# 호환성: ENHANCED_MISSILE_TYPES 별칭
ENHANCED_MISSILE_TYPES = MISSILE_TYPES

# =============================================================================
# 기본값 초기화 (SCUD-B)
# =============================================================================
_default = MISSILE_TYPES["SCUD-B"]
MISSILE_MASS = _default["launch_weight"]
MISSILE_WEIGHT = MISSILE_MASS * G
PROPELLANT_MASS = _default["propellant_mass"]
ISP = _default["isp_sea"]
WING_AREA = _default["reference_area"]

VERTICAL_TIME = _default["vertical_time"]
PITCH_TIME = _default["pitch_time"]
PITCH_ANGLE_DEG = _default["pitch_angle_deg"]
BURN_TIME = _default["burn_time"]

# main.py 호환 변수
VERTICAL_PHASE_TIME = VERTICAL_TIME
PITCH_PHASE_TIME = PITCH_TIME
CONSTANT_PHASE_TIME = max(0, BURN_TIME - VERTICAL_TIME - PITCH_TIME)

# 시뮬레이션 설정
INTERVAL = 0.1
SIM_TIME = 2000
SIMULATION_END_TIME = SIM_TIME

# =============================================================================
# 유틸리티 함수
# =============================================================================

def get_density(h):
    """ISA 1976 대기 밀도"""
    if h < 0:
        h = 0
    if h < 11000:
        T = 288.15 - 0.0065 * h
        p = 101325 * (T / 288.15) ** 5.2561
    elif h < 25000:
        T = 216.65
        p = 22632.1 * np.exp(-0.00015768 * (h - 11000))
    else:
        T = 216.65 + 0.003 * (h - 25000)
        T = max(T, 180)  # 최소 온도
        p = 2488.7 * (T / 216.65) ** (-11.388)
        p = max(p, 0)
    
    rho = p / (287.05 * T) if T > 0 else 0
    return max(rho, 1e-12)  # 최소값 보장


def get_pressure(h):
    """고도별 대기압 (Pa)"""
    if h < 0:
        h = 0
    if h < 11000:
        T = 288.15 - 0.0065 * h
        p = 101325 * (T / 288.15) ** 5.2561
    elif h < 25000:
        T = 216.65
        p = 22632.1 * np.exp(-0.00015768 * (h - 11000))
    else:
        T = 216.65 + 0.003 * (h - 25000)
        p = 2488.7 * (T / 216.65) ** (-11.388)
    return max(p, 0)


def get_sound_speed(h):
    """고도별 음속 (m/s)"""
    if h < 11000:
        T = 288.15 - 0.0065 * h
    elif h < 25000:
        T = 216.65
    else:
        T = 216.65 + 0.003 * (h - 25000)
    T = max(T, 180)
    return np.sqrt(1.4 * 287.05 * T)


def get_cd_interpolated(missile_type, mach, alpha_deg=0):
    """마하수/받음각에 따른 항력계수 보간"""
    if missile_type in MISSILE_CD_TABLES:
        table = MISSILE_CD_TABLES[missile_type]
        cd_base = np.interp(mach, table['mach'], table['cd'])
    else:
        # 기본 테이블
        cd_base = 0.3 + 0.5 * np.exp(-((mach - 1.2) / 0.5)**2)
    
    # 받음각 효과 (작은 각도에서 2차 근사)
    alpha_rad = abs(alpha_deg) * DEG_TO_RAD
    cd_alpha = 0.05 * alpha_rad ** 2
    
    return cd_base + cd_alpha


def get_isp_at_altitude(missile_type, h):
    """고도별 비추력 보간"""
    if missile_type not in MISSILE_TYPES:
        return 250  # 기본값
    
    info = MISSILE_TYPES[missile_type]
    isp_sea = info["isp_sea"]
    isp_vac = info.get("isp_vac", info.get("isp_vacuum", isp_sea * 1.1))
    
    # 압력비 기반 보간
    p_ratio = get_pressure(h) / STD_PRESSURE_SEA_LEVEL
    isp = isp_sea + (isp_vac - isp_sea) * (1 - p_ratio)
    
    return isp


def get_missile_info(missile_type):
    """미사일 정보 반환"""
    if missile_type not in MISSILE_TYPES:
        raise ValueError(f"Unknown missile type: {missile_type}")
    return MISSILE_TYPES[missile_type].copy()


def set_missile_type(missile_type_key):
    """미사일 타입 설정 (전역 변수 업데이트)"""
    global MISSILE_MASS, MISSILE_WEIGHT, PROPELLANT_MASS, ISP, WING_AREA
    global VERTICAL_TIME, PITCH_TIME, PITCH_ANGLE_DEG, BURN_TIME
    global VERTICAL_PHASE_TIME, PITCH_PHASE_TIME, CONSTANT_PHASE_TIME
    
    if missile_type_key not in MISSILE_TYPES:
        print(f"오류: 미사일 유형 '{missile_type_key}'을(를) 찾을 수 없습니다.")
        return False
    
    info = MISSILE_TYPES[missile_type_key]
    
    MISSILE_MASS = info["launch_weight"]
    MISSILE_WEIGHT = MISSILE_MASS * G
    PROPELLANT_MASS = info["propellant_mass"]
    ISP = info["isp_sea"]
    WING_AREA = info["reference_area"]
    
    VERTICAL_TIME = info["vertical_time"]
    PITCH_TIME = info["pitch_time"]
    PITCH_ANGLE_DEG = info["pitch_angle_deg"]
    BURN_TIME = info["burn_time"]
    
    VERTICAL_PHASE_TIME = VERTICAL_TIME
    PITCH_PHASE_TIME = PITCH_TIME
    CONSTANT_PHASE_TIME = max(0, BURN_TIME - VERTICAL_TIME - PITCH_TIME)
    
    print(f"미사일 유형이 '{info['name']}'(으)로 설정되었습니다.")
    return True


def get_available_missile_types():
    """사용 가능한 미사일 타입 목록"""
    return list(MISSILE_TYPES.keys())


def calculate_cd_table(diameter, length, nozzle_diameter, propellant_type):
    """항력계수 테이블 생성 (main.py 호환)"""
    mach_numbers = np.linspace(0, 5, 51)
    cd_values = np.array([0.3 + 0.5 * np.exp(-((m - 1.2) / 0.5)**2) for m in mach_numbers])
    return dict(zip(mach_numbers, cd_values))


# =============================================================================
# 6DOF 전용 함수
# =============================================================================

def get_inertia_tensor(missile_type, mass_fraction=1.0):
    """
    질량 분율에 따른 관성모멘트 텐서 반환
    
    Args:
        missile_type: 미사일 타입
        mass_fraction: 현재질량/초기질량 (1.0 = 만재, ~0.2 = 연료 소진)
    
    Returns:
        (I_xx, I_yy, I_zz) tuple
    """
    if missile_type not in MISSILE_TYPES:
        return (500, 45000, 45000)  # 기본값
    
    info = MISSILE_TYPES[missile_type]
    I_xx = info["I_xx"]
    I_yy = info["I_yy"]
    I_zz = info["I_zz"]
    
    # 연료 소모에 따른 관성모멘트 감소 (선형 근사)
    # 연료는 동체 중앙에 있으므로 주로 I_yy, I_zz에 영향
    fuel_factor = 0.5 + 0.5 * mass_fraction  # 0.5 ~ 1.0
    
    return (I_xx * fuel_factor, I_yy * fuel_factor, I_zz * fuel_factor)


def get_stability_derivatives(missile_type, mass_fraction=1.0, mach=1.0):
    """
    질량/마하수에 따른 안정성 미계수 반환
    
    Returns:
        dict with C_m_alpha, C_mq, C_nr, C_lp, CL_alpha, CY_beta
    """
    if missile_type not in MISSILE_TYPES:
        return {
            "C_m_alpha": -2.5,
            "C_mq": -8.0,
            "C_nr": -8.0,
            "C_lp": -0.5,
            "CL_alpha": 4.5,
            "CY_beta": -0.5
        }
    
    info = MISSILE_TYPES[missile_type]
    
    # 기본값
    C_m_alpha_base = info["C_m_alpha"]
    C_mq = info["C_mq"]
    C_nr = info["C_nr"]
    C_lp = info["C_lp"]
    CL_alpha = info["CL_alpha"]
    CY_beta = info["CY_beta"]
    
    # CG 이동 효과: 연료 소모 → CG 후방 → 안정성 감소
    cg_effect = -1.5 * (1 - mass_fraction)  # 0 ~ -1.5 (덜 안정해짐)
    C_m_alpha = C_m_alpha_base + cg_effect
    
    # 마하수 효과: 초음속에서 안정성 증가
    if mach > 1.0:
        mach_effect = -0.3 * min(mach - 1.0, 2.0)  # 0 ~ -0.6
        C_m_alpha += mach_effect
    
    return {
        "C_m_alpha": C_m_alpha,
        "C_mq": C_mq,
        "C_nr": C_nr,
        "C_lp": C_lp,
        "CL_alpha": CL_alpha,
        "CY_beta": CY_beta
    }


# =============================================================================
# 호환성 클래스 (main.py의 PhysicsUtils)
# =============================================================================

class PhysicsUtils:
    """Physics-Informed Neural ODE와 호환되는 물리 유틸리티"""
    
    @staticmethod
    def gravity_at_altitude(h):
        return G * (R_EARTH / (R_EARTH + h))**2
    
    @staticmethod
    def atmospheric_density(h):
        return get_density(h)
    
    @staticmethod
    def sound_speed(h):
        return get_sound_speed(h)
    
    @staticmethod
    def mach_number(velocity, altitude):
        return velocity / get_sound_speed(altitude)
    
    @staticmethod
    def drag_coefficient_model(mach, alpha_deg=0):
        return get_cd_interpolated("SCUD-B", mach, alpha_deg)


# =============================================================================
# 상태 벡터 정의 (3DOF/6DOF 공통)
# =============================================================================

class StateVector3DOF:
    """3DOF 상태 벡터 (main.py 호환)"""
    VELOCITY = 0
    GAMMA = 1
    PSI = 2
    X = 3
    Y = 4
    H = 5
    MASS = 6
    FUEL_CONSUMED = 7
    STATE_DIM = 8


class StateVector6DOF:
    """6DOF 상태 벡터"""
    X = 0
    Y = 1
    Z = 2       # 고도 (ENU: Up)
    U = 3       # Body X 속도
    V = 4       # Body Y 속도
    W = 5       # Body Z 속도
    PHI = 6     # Roll
    THETA = 7   # Pitch
    PSI = 8     # Yaw
    P = 9       # Roll rate
    Q = 10      # Pitch rate
    R = 11      # Yaw rate
    MASS = 12
    STATE_DIM = 13


if __name__ == "__main__":
    print("6DOF Config 테스트")
    print("=" * 50)
    
    for missile in get_available_missile_types():
        info = get_missile_info(missile)
        print(f"\n{missile}:")
        print(f"  발사질량: {info['launch_weight']} kg")
        print(f"  추진제: {info['propellant_mass']} kg")
        print(f"  연소시간: {info['burn_time']} s")
        print(f"  ISP (해면/진공): {info['isp_sea']}/{info.get('isp_vac', 'N/A')} s")
        print(f"  관성모멘트: I_yy = {info['I_yy']} kg·m²")
        print(f"  정적 안정성: C_m_alpha = {info['C_m_alpha']} /rad")
        
        # 연료 소진 시 안정성 변화
        derivs_full = get_stability_derivatives(missile, 1.0, 2.0)
        derivs_empty = get_stability_derivatives(missile, 0.2, 2.0)
        print(f"  C_m_alpha 변화: {derivs_full['C_m_alpha']:.2f} → {derivs_empty['C_m_alpha']:.2f}")
