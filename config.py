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

# 🔧 모터 방향 반전 설정 (NEW!)
# 오른쪽 모터들의 회전방향을 반대로 하기 위한 설정
# True: 해당 조인트 각도에 -1을 곱함 (방향 반전)
# False: 원본 각도 그대로 사용
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
MOTOR_MAX_ANGLE = 50    # 도
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

# 🔧 방향 반전 유틸리티 함수들 (NEW!)
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

def get_motor_direction_info():
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

# 모듈 임포트 시 자동 검사
if __name__ == "__main__":
    print("🔧 모터 설정 테스트...")
    validate_motor_config()
    
    print("\n📊 모터 방향 설정 정보:")
    for info in get_motor_direction_info():
        invert_str = "반전" if info['direction_inverted'] else "정방향"
        print(f"  조인트{info['joint_index']} ({info['joint_name']}) → 모터ID{info['motor_id']} - {info['side']} - {invert_str}")
    
    print("\n🧪 방향 반전 테스트:")
    test_angles = [10, 15, -5, 20, 12, -8, 25, -10]
    print(f"원본 각도: {test_angles}")
    inverted = apply_direction_inversion(test_angles)
    print(f"반전 적용: {inverted}")
    
    print("\n✅ 모터 설정 테스트 완료!")
