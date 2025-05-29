#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 시스템 설정 파일
모든 포트, 해상도, 주기 등의 설정값들을 관리
"""

# 포트 설정
IMU_PORT = '/dev/ttyUSB0'
MOTOR_PORT = '/dev/ttyUSB1'
IMU_BAUDRATE = 115200
MOTOR_BAUDRATE = 57600

# 카메라 설정
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Isaac Lab 조인트 매핑 (중요!)
# Isaac Lab 조인트 순서: [0, 1, 2, 3, 4, 5, 6, 7]
# 실제 Dynamixel ID: [7, 8, 1, 2, 10, 11, 4, 5]
ISAAC_TO_DYNAMIXEL_MAPPING = [7, 8, 1, 2, 10, 11, 4, 5]
DYNAMIXEL_IDS = [1, 2, 4, 5, 7, 8, 10, 11]  # 실제 연결된 모터들

# 🚗 바퀴 모터 설정 (NEW!)
WHEEL_MOTOR_IDS = [3, 6, 9, 12]  # 바퀴 모터 ID들
WHEEL_POSITIONS = {
    3: 'lf',   # 왼쪽 앞
    6: 'rf',   # 오른쪽 앞  
    9: 'lb',   # 왼쪽 뒤
    12: 'rb'   # 오른쪽 뒤
}

# 🚗 바퀴 방향 반전 설정 (수정됨)
# 전진 시 모든 바퀴가 같은 방향으로 회전하도록 설정
WHEEL_DIRECTION_INVERT = {
    3: False,   # lf (왼쪽 앞) - 정방향
    6: False,   # rf (오른쪽 앞) - 정방향 (반전 해제) ✅
    9: False,   # lb (왼쪽 뒤) - 정방향  
    12: False   # rb (오른쪽 뒤) - 정방향 (반전 해제) ✅
}

# 🚗 바퀴 속도 설정 (2배 증가)
WHEEL_MAX_VELOCITY = 100    # RPM (최대 속도 - 50→100)
WHEEL_DEFAULT_SPEED = 60    # RPM (기본 속도 - 30→60)
WHEEL_SLOW_SPEED = 30       # RPM (저속 - 15→30)
WHEEL_FAST_SPEED = 90       # RPM (고속 - 45→90)

# 🔧 방향 반전 설정 (기존)
JOINT_DIRECTION_INVERT = [
    False,  # 조인트 0 (lb_leg, 모터ID 7) - 왼쪽 뒷다리
    False,  # 조인트 1 (lb_knee, 모터ID 8) - 왼쪽 뒷무릎
    False,  # 조인트 2 (lf_leg, 모터ID 1) - 왼쪽 앞다리
    False,  # 조인트 3 (lf_knee, 모터ID 2) - 왼쪽 앞무릎
    True,   # 조인트 4 (rb_leg, 모터ID 10) - 오른쪽 뒷다리 ⚠️ 반전
    True,   # 조인트 5 (rb_knee, 모터ID 11) - 오른쪽 뒷무릎 ⚠️ 반전
    True,   # 조인트 6 (rf_leg, 모터ID 4) - 오른쪽 앞다리 ⚠️ 반전
    True,   # 조인트 7 (rf_knee, 모터ID 5) - 오른쪽 앞무릎 ⚠️ 반전
]

# 모터 설정
MOTOR_MIN_ANGLE = -50     # 도
MOTOR_MAX_ANGLE = 70    # 도
MOTOR_CENTER_POSITION = 2048

# 시스템 주기 설정
SYSTEM_FREQUENCY = 30   # Hz (AI 모델 실행 주기)
SENSOR_UPDATE_RATE = 0.001  # 초 (센서 업데이트 주기)

# 단위 변환 상수
DEG_TO_RAD = 3.14159265359 / 180.0
RAD_TO_DEG = 180.0 / 3.14159265359
MM_TO_M = 0.001

# 파일 경로
SAVE_DIRECTORY = "/home/ubuntu/test_folder"
COMBINED_OBS_PATH = "/home/ubuntu/test_folder/combined_obs.npy"
SENSOR_LOG_PATH = "/home/ubuntu/test_folder/sensor_log.txt"

# IMU 필터링 설정
IMU_VELOCITY_ALPHA = 0.3        # 속도 필터 계수
IMU_ACCELERATION_THRESHOLD = 0.05   # 노이즈 임계값
IMU_VELOCITY_DECAY = 0.98       # 속도 감쇠 계수

# 조인트 이름 매핑 (디버깅용)
JOINT_NAMES = [
    "lb_leg",   # 조인트 0 (왼뒤 다리) - 모터 ID 7
    "lb_knee",  # 조인트 1 (왼뒤 무릎) - 모터 ID 8  
    "lf_leg",   # 조인트 2 (왼앞 다리) - 모터 ID 1
    "lf_knee",  # 조인트 3 (왼앞 무릎) - 모터 ID 2
    "rb_leg",   # 조인트 4 (우뒤 다리) - 모터 ID 10 ⚠️ 반전
    "rb_knee",  # 조인트 5 (우뒤 무릎) - 모터 ID 11 ⚠️ 반전
    "rf_leg",   # 조인트 6 (우앞 다리) - 모터 ID 4 ⚠️ 반전
    "rf_knee",  # 조인트 7 (우앞 무릎) - 모터 ID 5 ⚠️ 반전
]

# 디버그 설정
DEBUG_MODE = True
VERBOSE_LOGGING = False

# 🔧 방향 반전 유틸리티 함수들 (기존)
def apply_direction_inversion(joint_angles_deg):
    """
    조인트 각도에 방향 반전 적용
    
    Args:
        joint_angles_deg: 8개 조인트 각도 리스트 (도 단위)
    
    Returns:
        list: 방향 반전이 적용된 각도 리스트
    """
    if len(joint_angles_deg) != 8:
        raise ValueError(f"조인트 각도는 8개여야 함, 현재: {len(joint_angles_deg)}")
    
    inverted_angles = []
    for i, angle in enumerate(joint_angles_deg):
        if JOINT_DIRECTION_INVERT[i]:
            inverted_angles.append(-angle)  # 방향 반전
        else:
            inverted_angles.append(angle)   # 원본 유지
    
    return inverted_angles

# 🚗 바퀴 방향 반전 유틸리티 함수들 (NEW!)
def apply_wheel_direction_inversion(wheel_id, velocity_rpm):
    """
    바퀴 속도에 방향 반전 적용
    
    Args:
        wheel_id: 바퀴 모터 ID
        velocity_rpm: 속도 (RPM)
    
    Returns:
        float: 방향 반전이 적용된 속도
    """
    if wheel_id in WHEEL_DIRECTION_INVERT and WHEEL_DIRECTION_INVERT[wheel_id]:
        return -velocity_rpm  # 오른쪽 바퀴 반전
    return velocity_rpm

def get_wheel_info():
    """바퀴 설정 정보 반환 (디버깅용)"""
    info = []
    for wheel_id in WHEEL_MOTOR_IDS:
        position = WHEEL_POSITIONS[wheel_id]
        invert = WHEEL_DIRECTION_INVERT[wheel_id]
        side = "오른쪽" if position.startswith('r') else "왼쪽"
        pos_name = "앞" if position.endswith('f') else "뒤"
        
        info.append({
            'wheel_id': wheel_id,
            'position': position,
            'side': side,
            'position_name': pos_name,
            'direction_inverted': invert
        })
    
    return info

# 🚗 옴니휠 홀로노믹 운동학 (원래대로 복원)
def calculate_omni_wheel_velocities(vx, vy, omega):
    """
    홀로노믹 이동을 위한 바퀴 속도 계산
    
    Args:
        vx: 전후 속도 (양수: 전진)
        vy: 좌우 속도 (양수: 우측)  
        omega: 회전 속도 (양수: 반시계)
    
    Returns:
        dict: 각 바퀴의 속도 {wheel_id: rpm}
    """
    # 🔧 표준 옴니휠 운동학 (원래대로)
    wheel_velocities = {
        3: vx - vy - omega,  # lf (왼쪽 앞)
        6: -vx - vy - omega,  # rf (오른쪽 앞)
        9: vx + vy - omega,  # lb (왼쪽 뒤)
        12: -vx + vy - omega  # rb (오른쪽 뒤)
    }
    
    # 방향 반전 적용
    for wheel_id in wheel_velocities:
        wheel_velocities[wheel_id] = apply_wheel_direction_inversion(
            wheel_id, wheel_velocities[wheel_id]
        )
    
    return wheel_velocities

def test_individual_wheel_directions():
    """개별 바퀴 방향 테스트용 함수"""
    print("\n🔧 개별 바퀴 방향 테스트:")
    
    # 각 바퀴를 개별적으로 테스트
    test_cases = [
        ("왼쪽 앞 (lf, ID3) 단독", {3: 30, 6: 0, 9: 0, 12: 0}),
        ("오른쪽 앞 (rf, ID6) 단독", {3: 0, 6: 30, 9: 0, 12: 0}),
        ("왼쪽 뒤 (lb, ID9) 단독", {3: 0, 6: 0, 9: 30, 12: 0}),
        ("오른쪽 뒤 (rb, ID12) 단독", {3: 0, 6: 0, 9: 0, 12: 30}),
    ]
    
    for description, velocities in test_cases:
        print(f"  {description}: {velocities}")
    
    return test_cases
    """모터 방향 설정 정보 반환 (디버깅용)"""
    info = []
    for i in range(8):
        joint_name = JOINT_NAMES[i]
        motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[i]
        invert = JOINT_DIRECTION_INVERT[i]
        side = "오른쪽" if joint_name.startswith('r') else "왼쪽"
        
        info.append({
            'joint_index': i,
            'joint_name': joint_name,
            'motor_id': motor_id,
            'side': side,
            'direction_inverted': invert
        })
    
    return info

# 설정 유효성 검사
def validate_motor_config():
    """모터 설정 유효성 검사"""
    assert len(ISAAC_TO_DYNAMIXEL_MAPPING) == 8, "매핑 배열 크기 오류"
    assert len(JOINT_DIRECTION_INVERT) == 8, "방향 반전 배열 크기 오류"
    assert len(JOINT_NAMES) == 8, "조인트 이름 배열 크기 오류"
    
    # 오른쪽 조인트들이 모두 반전 설정되어 있는지 확인
    right_joints = [4, 5, 6, 7]  # rb_leg, rb_knee, rf_leg, rf_knee
    for i in right_joints:
        assert JOINT_DIRECTION_INVERT[i] == True, f"오른쪽 조인트 {i}가 반전 설정되지 않음"
    
    print("✅ 모터 설정 유효성 검사 통과")

def validate_wheel_config():
    """바퀴 설정 유효성 검사"""
    assert len(WHEEL_MOTOR_IDS) == 4, "바퀴 모터 ID 배열 크기 오류"
    assert len(WHEEL_POSITIONS) == 4, "바퀴 위치 배열 크기 오류"
    assert len(WHEEL_DIRECTION_INVERT) == 4, "바퀴 방향 반전 배열 크기 오류"
    
    # 이제 모든 바퀴가 정방향으로 설정되어야 함
    for wheel_id in WHEEL_MOTOR_IDS:
        assert WHEEL_DIRECTION_INVERT[wheel_id] == False, f"바퀴 {wheel_id}가 정방향으로 설정되지 않음"
    
    print("✅ 바퀴 설정 유효성 검사 통과")

# 모듈 임포트 시 자동 검사
if __name__ == "__main__":
    print("🔧 시스템 설정 테스트...")
    validate_motor_config()
    validate_wheel_config()
    
    print("\n📊 모터 방향 설정 정보:")
    for info in get_motor_direction_info():
        invert_str = "반전" if info['direction_inverted'] else "정방향"
        print(f"  조인트{info['joint_index']} ({info['joint_name']}) → 모터ID{info['motor_id']} - {info['side']} - {invert_str}")
    
    print("\n🚗 바퀴 설정 정보:")
    for info in get_wheel_info():
        invert_str = "반전" if info['direction_inverted'] else "정방향"
        print(f"  {info['position']} ({info['side']} {info['position_name']}) → 모터ID{info['wheel_id']} - {invert_str}")
    
    print("\n🧪 방향 반전 테스트:")
    test_angles = [10, 15, -5, 20, 12, -8, 25, -10]
    print(f"원본 각도: {test_angles}")
    inverted = apply_direction_inversion(test_angles)
    print(f"반전 적용: {inverted}")
    
    print("\n🚗 바퀴 속도 테스트 (간단 수정):")
    test_forward = calculate_omni_wheel_velocities(60, 0, 0)  # 전진
    print(f"전진 w키 (vx=60): {test_forward}")
    
    test_backward = calculate_omni_wheel_velocities(-60, 0, 0)  # 후진  
    print(f"후진 백스페이스 (vx=-60): {test_backward}")
    
    test_left = calculate_omni_wheel_velocities(0, -30, 0)  # 좌측
    print(f"좌측 a키 (vy=-30): {test_left}")
    
    test_right = calculate_omni_wheel_velocities(0, 30, 0)  # 우측
    print(f"우측 d키 (vy=30): {test_right}")
    
    print("\n✅ 시스템 설정 테스트 완료!")