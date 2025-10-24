# 6DOF config.py - 회전 운동 추가
import numpy as np

# 기본 단위 환산
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# 물리 상수
G = 9.80665  # 표준 중력가속도 (m/s²)
R_EARTH = 6371000  # 지구 평균 반지름 (m)
R = R_EARTH
GM_EARTH = 3.986004418e14  # 지구 중력 매개변수 (m³/s²)

# 대기 상수
AIR_GAS_CONSTANT = 287.0531  # 건조 공기 기체상수 (J/(kg·K))
STD_TEMPERATURE_SEA_LEVEL = 288.15  # 해면 표준온도 (K)
STD_PRESSURE_SEA_LEVEL = 101325.0  # 해면 표준기압 (Pa)
STD_DENSITY_SEA_LEVEL = 1.225  # 해면 표준밀도 (kg/m³)
EXOSPHERE_HEIGHT = 600000  # 외기권 시작 고도 (m)

# Physics Utilities (기존과 동일)
class PhysicsUtils:
    """Physics-Informed Neural ODE와 호환되는 물리 유틸리티"""
    
    @staticmethod
    def gravity_at_altitude(h):
        """고도에 따른 중력 계산"""
        return G * (R_EARTH / (R_EARTH + h))**2
    
    @staticmethod
    def atmospheric_density(h):
        """표준 대기 모델 기반 밀도 계산"""
        if h < 0:
            return STD_DENSITY_SEA_LEVEL
        
        # 11km 이하 (대류권)
        if h <= 11000:
            T = STD_TEMPERATURE_SEA_LEVEL - 0.0065 * h
            P = STD_PRESSURE_SEA_LEVEL * (T / STD_TEMPERATURE_SEA_LEVEL)**5.2561
        # 11-20km (성층권 하부)
        elif h <= 20000:
            T = 216.65
            P = 22632.1 * np.exp(-0.00015768 * (h - 11000))
        # 20km 이상 (근사)
        else:
            T = 216.65 + 0.001 * (h - 20000)
            P = 5474.89 * (T / 216.65)**(-34.163)
        
        return P / (AIR_GAS_CONSTANT * T)
    
    @staticmethod
    def sound_speed(h):
        """고도에 따른 음속 계산"""
        if h <= 11000:
            T = STD_TEMPERATURE_SEA_LEVEL - 0.0065 * h
        elif h <= 20000:
            T = 216.65
        else:
            T = 216.65 + 0.001 * (h - 20000)
        
        gamma = 1.4  # 비열비
        R = AIR_GAS_CONSTANT
        return np.sqrt(gamma * R * T)
    
    @staticmethod
    def mach_number(velocity, altitude):
        """마하수 계산"""
        sound_speed = PhysicsUtils.sound_speed(altitude)
        return velocity / sound_speed
    
    @staticmethod
    def drag_coefficient_model(mach, alpha_deg=0):
        """Neural ODE 호환 항력계수 모델"""
        cd_base = 0.2 + 0.3 * np.exp(-((mach - 1.2) / 0.8)**2)
        alpha_rad = alpha_deg * DEG_TO_RAD
        cd_alpha = 0.1 * np.sin(2 * alpha_rad)**2
        return cd_base + cd_alpha

# 6DOF 상태 벡터 정의
class StateVector6DOF:
    """6DOF 상태 벡터 관리 클래스"""
    
    # 상태 인덱스 정의 (14차원)
    # 병진 운동 (6개)
    VELOCITY = 0      # 속도 (m/s)
    GAMMA = 1         # 피치각 (rad)
    PSI = 2           # 방위각 (rad)
    X = 3             # X 위치 (m)
    Y = 4             # Y 위치 (m)
    H = 5             # 고도 (m)
    
    # 회전 운동 (6개)
    PHI = 6           # 롤각 (rad)
    THETA = 7         # 피치각 (오일러각) (rad)
    PSI_EULER = 8     # 요각 (오일러각) (rad)
    P = 9             # 롤 각속도 (rad/s)
    Q = 10            # 피치 각속도 (rad/s)
    R_RATE = 11       # 요 각속도 (rad/s)
    
    # 질량 (2개)
    MASS = 12         # 질량 (kg)
    FUEL_CONSUMED = 13 # 연료소모 (kg)
    
    STATE_DIM = 14
    
    STATE_NAMES = [
        'velocity', 'gamma_rad', 'psi_rad', 'x', 'y', 'h',
        'phi_rad', 'theta_rad', 'psi_euler_rad', 'p_rate', 'q_rate', 'r_rate',
        'mass', 'fuel_consumed'
    ]
    
    STATE_UNITS = [
        'm/s', 'rad', 'rad', 'm', 'm', 'm',
        'rad', 'rad', 'rad', 'rad/s', 'rad/s', 'rad/s',
        'kg', 'kg'
    ]
    
    @staticmethod
    def create_initial_state(missile_info, launch_angle_deg, launch_azimuth_deg=90):
        """초기 상태 벡터 생성 (6DOF)"""
        initial_state = np.zeros(StateVector6DOF.STATE_DIM)
        
        # 병진 운동 초기값
        initial_state[StateVector6DOF.VELOCITY] = 0.0
        initial_state[StateVector6DOF.GAMMA] = np.deg2rad(launch_angle_deg)
        initial_state[StateVector6DOF.PSI] = np.deg2rad(launch_azimuth_deg)
        initial_state[StateVector6DOF.X] = 0.0
        initial_state[StateVector6DOF.Y] = 0.0
        initial_state[StateVector6DOF.H] = 0.0
        
        # 회전 운동 초기값 (발사대에서 정렬된 상태)
        initial_state[StateVector6DOF.PHI] = 0.0  # 롤 0
        initial_state[StateVector6DOF.THETA] = np.deg2rad(launch_angle_deg)  # 피치 = 발사각
        initial_state[StateVector6DOF.PSI_EULER] = np.deg2rad(launch_azimuth_deg)  # 요 = 방위각
        initial_state[StateVector6DOF.P] = 0.0  # 롤 각속도 0
        initial_state[StateVector6DOF.Q] = 0.0  # 피치 각속도 0
        initial_state[StateVector6DOF.R_RATE] = 0.0  # 요 각속도 0
        
        # 질량 초기값
        initial_state[StateVector6DOF.MASS] = missile_info["launch_weight"]
        initial_state[StateVector6DOF.FUEL_CONSUMED] = 0.0
        
        return initial_state
    
    @staticmethod
    def validate_state(state):
        """상태 벡터 유효성 검증"""
        if len(state) != StateVector6DOF.STATE_DIM:
            raise ValueError(f"상태 벡터 차원 오류: {len(state)} != {StateVector6DOF.STATE_DIM}")
        
        velocity = state[StateVector6DOF.VELOCITY]
        altitude = state[StateVector6DOF.H]
        mass = state[StateVector6DOF.MASS]
        
        if velocity < 0:
            raise ValueError("속도는 음수가 될 수 없습니다")
        if altitude < -100:
            raise ValueError("고도가 너무 낮습니다")
        if mass <= 0:
            raise ValueError("질량은 양수여야 합니다")
        
        return True

# 6DOF 미사일 정보 (관성 모멘트 추가)
ENHANCED_MISSILE_TYPES_6DOF = {
    "SCUD-B": {
        # 기본 정보
        "name": "SCUD-B",
        "diameter": 0.88,
        "length": 10.94,
        "nozzle_diameter": 0.6,
        "launch_weight": 5860,
        "payload": 985,
        "propellant_mass": 4875,
        "range_km": 300,
        
        # 추진 정보
        "propellant_type": "LIQUID",
        "isp_sea": 230,
        "isp_vac": 258,
        "burn_time": 65,
        
        # 비행 프로그램
        "vertical_time": 10,
        "pitch_time": 15,
        "pitch_angle_deg": 20,
        
        # 공기역학
        "reference_area": np.pi * (0.88/2)**2,
        "cd_base": 0.25,
        
        # 🆕 6DOF: 관성 모멘트 (kg·m²)
        # 원통형 로켓 가정: Ixx = Iyy = (1/12)*m*L² + (1/4)*m*r²
        #                  Izz = (1/2)*m*r²
        "inertia_xx": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),  # 피치/요 관성
        "inertia_yy": 5860 * (10.94**2 / 12 + (0.88/2)**2 / 4),
        "inertia_zz": 5860 * ((0.88/2)**2 / 2),  # 롤 관성
        
        # 🆕 6DOF: 공력 모멘트 계수 (댐핑 강화로 진동 감소)
        "cl_alpha": 0.5,      # 양력 기울기 (per rad)
        "cm_alpha": -0.15,    # 피칭 모멘트 기울기 (per rad, 정적 안정)
        "cn_beta": -0.1,      # 요잉 모멘트 기울기 (per rad)
        "cl_p": -0.5,         # 롤 댐핑 계수 (증가: -0.02 → -0.5)
        "cm_q": -0.8,         # 피치 댐핑 계수 (증가: -0.05 → -0.8)
        "cn_r": -0.8,         # 요 댐핑 계수 (증가: -0.05 → -0.8)
        
        # 🆕 6DOF: 무게중심 및 압력중심 (미사일 앞쪽에서 거리, m)
        "cg_location": 5.5,   # 무게중심 (길이 중간)
        "cp_location": 6.0,   # 압력중심 (무게중심 뒤)
        
        # Neural ODE 호환 함수들
        "thrust_profile": lambda t: 5860 * 9.81 * 230 / 65 if t < 65 else 0,
        "mass_flow_rate": lambda t: 4875 / 65 if t < 65 else 0,
        "drag_model": lambda mach, alpha: PhysicsUtils.drag_coefficient_model(mach, alpha),
    },
    
    "NODONG": {
        "name": "노동 1호",
        "diameter": 1.36,
        "length": 16.4,
        "nozzle_diameter": 0.8,
        "launch_weight": 16500,
        "payload": 1200,
        "propellant_mass": 15300,
        "range_km": 1500,
        
        "propellant_type": "UDMH/RFNA",
        "isp_sea": 255,
        "isp_vacuum": 280,
        "reference_area": np.pi * (1.36/2)**2,
        
        "vertical_time": 10,
        "pitch_time": 20,
        "pitch_angle_deg": 15,
        "burn_time": 70,
        
        # 🆕 6DOF: 관성 모멘트
        "inertia_xx": 16500 * (16.4**2 / 12 + (1.36/2)**2 / 4),
        "inertia_yy": 16500 * (16.4**2 / 12 + (1.36/2)**2 / 4),
        "inertia_zz": 16500 * ((1.36/2)**2 / 2),
        
        # 🆕 6DOF: 공력 모멘트 계수 (댐핑 강화)
        "cl_alpha": 0.55,
        "cm_alpha": -0.18,
        "cn_beta": -0.12,
        "cl_p": -0.6,         # 롤 댐핑 증가
        "cm_q": -0.9,         # 피치 댐핑 증가
        "cn_r": -0.9,         # 요 댐핑 증가
        
        # 🆕 6DOF: 무게중심 및 압력중심
        "cg_location": 8.2,
        "cp_location": 9.0,
        
        "thrust_profile": lambda t: 16500 * 9.81 * 280 / 70 * (1.2 if t < 15 else 1.0) if t < 70 else 0,
        "mass_flow_rate": lambda t: 15300 / 70 if t < 70 else 0,
        "drag_model": lambda mach, alpha: PhysicsUtils.drag_coefficient_model(mach, alpha) * 0.9,
    },
    
    "KN-23": {
        "name": "KN-23",
        "diameter": 0.95,
        "length": 7.5,
        "nozzle_diameter": 0.5,
        "launch_weight": 3415,
        "payload": 500,
        "propellant_mass": 2915,
        "range_km": 690,
        
        "propellant_type": "SOLID",
        "isp_sea": 260,
        "isp_vac": 265,
        "burn_time": 40,
        
        "vertical_time": 6,
        "pitch_time": 10,
        "pitch_angle_deg": 25,
        
        "reference_area": np.pi * (0.95/2)**2,
        "cd_base": 0.28,
        
        # 🆕 6DOF: 관성 모멘트
        "inertia_xx": 3415 * (7.5**2 / 12 + (0.95/2)**2 / 4),
        "inertia_yy": 3415 * (7.5**2 / 12 + (0.95/2)**2 / 4),
        "inertia_zz": 3415 * ((0.95/2)**2 / 2),
        
        # 🆕 6DOF: 공력 모멘트 계수 (댐핑 강화)
        "cl_alpha": 0.6,
        "cm_alpha": -0.2,
        "cn_beta": -0.15,
        "cl_p": -0.7,         # 롤 댐핑 증가
        "cm_q": -1.0,         # 피치 댐핑 증가
        "cn_r": -1.0,         # 요 댐핑 증가
        
        # 🆕 6DOF: 무게중심 및 압력중심
        "cg_location": 3.75,
        "cp_location": 4.2,
        
        "thrust_profile": lambda t: 3415 * 9.81 * 260 / 40 if t < 40 else 0,
        "mass_flow_rate": lambda t: 2915 / 40 if t < 40 else 0,
        "drag_model": lambda mach, alpha: PhysicsUtils.drag_coefficient_model(mach, alpha),
    }
}

# 역호환성을 위한 함수들
def set_missile_type(missile_type_key):
    """기존 main.py와의 호환성을 위한 함수"""
    global MISSILE_MASS, MISSILE_WEIGHT, PROPELLANT_MASS, ISP, WING_AREA
    global VERTICAL_TIME, PITCH_TIME, PITCH_ANGLE_DEG, BURN_TIME
    global INERTIA_XX, INERTIA_YY, INERTIA_ZZ
    
    if missile_type_key not in ENHANCED_MISSILE_TYPES_6DOF:
        print(f"오류: 미사일 유형 '{missile_type_key}'을(를) 찾을 수 없습니다.")
        return False
    
    missile_info = ENHANCED_MISSILE_TYPES_6DOF[missile_type_key]
    
    # 전역 변수 업데이트
    global VERTICAL_PHASE_TIME, PITCH_PHASE_TIME, CONSTANT_PHASE_TIME
    
    MISSILE_MASS = missile_info["launch_weight"]
    MISSILE_WEIGHT = MISSILE_MASS * G
    PROPELLANT_MASS = missile_info["propellant_mass"]
    ISP = missile_info["isp_sea"]
    WING_AREA = missile_info["reference_area"]
    
    # 관성 모멘트
    INERTIA_XX = missile_info["inertia_xx"]
    INERTIA_YY = missile_info["inertia_yy"]
    INERTIA_ZZ = missile_info["inertia_zz"]
    
    VERTICAL_TIME = missile_info["vertical_time"]
    PITCH_TIME = missile_info["pitch_time"]
    PITCH_ANGLE_DEG = missile_info["pitch_angle_deg"]
    BURN_TIME = missile_info["burn_time"]
    
    VERTICAL_PHASE_TIME = VERTICAL_TIME
    PITCH_PHASE_TIME = PITCH_TIME
    CONSTANT_PHASE_TIME = max(0, BURN_TIME - VERTICAL_TIME - PITCH_TIME)
    
    print(f"미사일 유형이 '{missile_info['name']}'(으)로 설정되었습니다 (6DOF).")
    return True

def get_available_missile_types():
    """사용 가능한 미사일 유형 목록 반환"""
    return list(ENHANCED_MISSILE_TYPES_6DOF.keys())

def get_cd_table_for_missile(missile_name):
    """미사일별 항력계수 테이블 반환"""
    default_cd_table = {
        'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
        'cd': np.array([0.3, 0.35, 0.45, 0.8, 0.6, 0.4, 0.35, 0.3, 0.28, 0.25])
    }
    
    missile_cd_tables = {
        'SCUD-B': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.32, 0.36, 0.47, 0.82, 0.62, 0.42, 0.37, 0.32, 0.30, 0.27])
        },
        'NODONG': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.28, 0.33, 0.43, 0.78, 0.58, 0.38, 0.33, 0.28, 0.26, 0.23])
        },
        'KN-23': {
            'mach': np.array([0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0]),
            'cd': np.array([0.35, 0.38, 0.48, 0.85, 0.65, 0.45, 0.40, 0.35, 0.33, 0.30])
        }
    }
    
    return missile_cd_tables.get(missile_name, default_cd_table)

# 기본 값 초기화 (SCUD-B)
default_missile = ENHANCED_MISSILE_TYPES_6DOF["SCUD-B"]
MISSILE_MASS = default_missile["launch_weight"]
MISSILE_WEIGHT = MISSILE_MASS * G
PROPELLANT_MASS = default_missile["propellant_mass"]
ISP = default_missile["isp_sea"]
WING_AREA = default_missile["reference_area"]

# 🆕 6DOF 관성 모멘트
INERTIA_XX = default_missile["inertia_xx"]
INERTIA_YY = default_missile["inertia_yy"]
INERTIA_ZZ = default_missile["inertia_zz"]

VERTICAL_TIME = default_missile["vertical_time"]
PITCH_TIME = default_missile["pitch_time"]
PITCH_ANGLE_DEG = default_missile["pitch_angle_deg"]
BURN_TIME = default_missile["burn_time"]

VERTICAL_PHASE_TIME = VERTICAL_TIME
PITCH_PHASE_TIME = PITCH_TIME
CONSTANT_PHASE_TIME = max(0, BURN_TIME - VERTICAL_TIME - PITCH_TIME)

# 시뮬레이션 설정
INTERVAL = 0.1
SIM_TIME = 2000
SIMULATION_END_TIME = SIM_TIME

# 역호환성
MISSILE_TYPES = ENHANCED_MISSILE_TYPES_6DOF
STD_DENSITY_SEA_LEVEL = 1.225
STD_PRESSURE_SEA_LEVEL = 101325
STD_GRAVITY = 9.80665
AIR_MOLAR_MASS = 0.0289644
UNIVERSAL_GAS_CONSTANT = 8.314462
AIR_GAS_CONSTANT = 287.058

# 대기층 정의
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

# 공력 계수
CL_VERTICAL = 0.0
CL_PITCH = 0.5
CL_CONSTANT = 0.3
CL_TERMINAL = 0.1
K = 0.05

# 추진 관련
TSFC = 0.0004

# 고도 상수
EXOSPHERE_ALTITUDE = 600000

# 기본 항력계수 테이블
BASE_CD_TABLE = {0: 0.3, 1: 0.8, 2: 1.2, 3: 1.0, 4: 0.9, 5: 0.8}

def calculate_cd_table(diameter, length, nozzle_diameter, propellant_type):
    """항력계수 테이블 계산"""
    mach_numbers = np.linspace(0, 5, 51)
    cd_values = np.array([PhysicsUtils.drag_coefficient_model(m) for m in mach_numbers])
    return dict(zip(mach_numbers, cd_values))
