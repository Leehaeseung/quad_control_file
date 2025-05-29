#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamixel 모터 센서 모듈 (방향 반전 적용 + 바퀴 속도 제어)
조인트 위치와 속도를 Isaac Lab 형식으로 제공 
실제 모터 ID를 Isaac Lab 조인트 순서로 매핑
오른쪽 모터들의 회전방향 자동 보정
바퀴 모터는 속도 제어 모드 사용
"""

import time
import threading
import numpy as np
from dynamixel_sdk import *
from config import *


class MotorSensor:
    def __init__(self, motor_ids=DYNAMIXEL_IDS, port=MOTOR_PORT, baudrate=MOTOR_BAUDRATE):
        """Dynamixel 모터 센서 초기화"""
        self.motor_ids = motor_ids
        
        # Dynamixel 설정 (포지션 제어)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_POSITION_P_GAIN = 84
        self.ADDR_POSITION_I_GAIN = 85
        self.ADDR_POSITION_D_GAIN = 86
        
        # 🚗 Dynamixel 설정 (속도 제어) - NEW!
        self.ADDR_OPERATING_MODE = 11        # 운영 모드 설정
        self.ADDR_GOAL_VELOCITY = 104        # 목표 속도
        self.ADDR_PRESENT_VELOCITY = 128     # 현재 속도
        self.OPERATING_MODE_VELOCITY = 1     # 속도 제어 모드
        self.OPERATING_MODE_POSITION = 3     # 포지션 제어 모드
        
        # 통신 설정
        self.PROTOCOL_VERSION = 2.0
        self.DEVICE_NAME = port
        self.BAUDRATE = baudrate
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        
        # 핸들러 초기화
        self.port_handler = PortHandler(self.DEVICE_NAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        
        # 연결 상태
        self.is_connected = False
        self.connected_motors = []
        self.connected_wheels = []  # 🚗 바퀴 모터 별도 관리
        
        # Isaac Lab 형식 데이터
        self.joint_positions = [0.0] * 8      # rad
        self.joint_velocities = [0.0] * 8     # rad/s
        
        # 속도 계산용
        self.prev_positions = [0.0] * 8
        self.prev_time = None
        
        # 스레드 안전성
        self.lock = threading.Lock()
        
        # PID 게인 기본값
        self.default_p_gain = 3000
        self.default_i_gain = 2000
        self.default_d_gain = 500000
        
        if DEBUG_MODE:
            print(f"✅ 모터 센서 초기화: {port}")
            print(f"🔧 방향 반전 설정 로드됨")
            print(f"🚗 바퀴 속도 제어 준비됨")
    
    def connect(self):
        """모터 연결 및 초기화"""
        if not self.port_handler.openPort():
            print("❌ 모터 포트 열기 실패!")
            return False
            
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            print("❌ 모터 통신속도 설정 실패!")
            return False
            
        if DEBUG_MODE:
            print(f"✅ 모터 포트 연결: {self.DEVICE_NAME}")
        
        # 각 모터 개별 연결 테스트
        self.connected_motors = []
        self.connected_wheels = []
        
        for motor_id in self.motor_ids:
            if self._test_motor_connection(motor_id):
                self.connected_motors.append(motor_id)
                if DEBUG_MODE:
                    print(f"✅ 모터 ID {motor_id} 연결")
                self._set_pid_gains(motor_id)
                self._enable_torque(motor_id)
            else:
                print(f"❌ 모터 ID {motor_id} 연결 실패")
        
        # 🚗 바퀴 모터 연결 테스트 및 속도 모드 설정
        for wheel_id in WHEEL_MOTOR_IDS:
            if self._test_motor_connection(wheel_id):
                self.connected_wheels.append(wheel_id)
                if DEBUG_MODE:
                    wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                    print(f"🚗 바퀴 ID {wheel_id} ({wheel_pos}) 연결")
                self._set_wheel_velocity_mode(wheel_id)
                self._enable_torque(wheel_id)
            else:
                print(f"❌ 바퀴 ID {wheel_id} 연결 실패")
        
        if len(self.connected_motors) == 0 and len(self.connected_wheels) == 0:
            print("❌ 연결된 모터가 없습니다!")
            return False
            
        self.is_connected = True
        if DEBUG_MODE:
            total_connected = len(self.connected_motors) + len(self.connected_wheels)
            print(f"🎉 총 {total_connected}개 모터 연결 완료!")
            print(f"다리 모터: {self.connected_motors}")
            print(f"바퀴 모터: {self.connected_wheels}")
            
        return True
    
    def _test_motor_connection(self, motor_id):
        """개별 모터 연결 테스트"""
        position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_POSITION
        )
        return result == COMM_SUCCESS
    
    def _enable_torque(self, motor_id):
        """토크 활성화"""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
        )
        return result == COMM_SUCCESS and error == 0
    
    def _disable_torque(self, motor_id):
        """토크 비활성화"""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
        )
        return result == COMM_SUCCESS and error == 0
    
    def _set_pid_gains(self, motor_id):
        """PID 게인 설정"""
        self._disable_torque(motor_id)
        self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.ADDR_POSITION_P_GAIN, self.default_p_gain)
        self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.ADDR_POSITION_I_GAIN, self.default_i_gain)
        self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.ADDR_POSITION_D_GAIN, self.default_d_gain)
    
    def _set_wheel_velocity_mode(self, wheel_id):
        """🚗 바퀴 모터를 속도 제어 모드로 설정"""
        try:
            # 토크 비활성화
            self._disable_torque(wheel_id)
            
            # 속도 제어 모드로 설정
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, wheel_id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE_VELOCITY
            )
            
            if result == COMM_SUCCESS and error == 0:
                if DEBUG_MODE:
                    wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                    print(f"🚗 바퀴 {wheel_id} ({wheel_pos}) 속도 제어 모드 설정 완료")
                return True
            else:
                print(f"❌ 바퀴 {wheel_id} 모드 설정 실패")
                return False
        except Exception as e:
            print(f"❌ 바퀴 {wheel_id} 모드 설정 오류: {e}")
            return False
    
    def _position_to_angle(self, position):
        """포지션 값을 각도로 변환 (degree)"""
        return ((position - MOTOR_CENTER_POSITION) / 4095.0) * 360
    
    def _angle_to_position(self, angle_deg):
        """각도를 포지션 값으로 변환"""
        return int(MOTOR_CENTER_POSITION + (angle_deg / 360.0) * 4095)
    
    def update_joint_data(self):
        """조인트 데이터 업데이트 (Isaac Lab 형식)"""
        if not self.is_connected:
            return False
        
        current_time = time.time()
        
        # 모든 모터 위치 읽기
        raw_positions = {}
        for motor_id in self.connected_motors:
            position, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS and error == 0:
                angle_deg = self._position_to_angle(position)
                raw_positions[motor_id] = angle_deg
            else:
                raw_positions[motor_id] = 0.0
        
        with self.lock:
            # Isaac Lab 조인트 순서로 재배열 및 단위 변환
            new_positions = []
            for isaac_joint_idx in range(8):
                motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[isaac_joint_idx]
                
                if motor_id in raw_positions:
                    angle_deg = raw_positions[motor_id]
                    
                    # 🔧 방향 반전 적용 (읽기 시에도 적용)
                    if JOINT_DIRECTION_INVERT[isaac_joint_idx]:
                        angle_deg = -angle_deg  # True일 때 부호 반전
                    
                    angle_rad = angle_deg * DEG_TO_RAD  # degree → radian
                    new_positions.append(angle_rad)
                else:
                    new_positions.append(0.0)
            
            # 속도 계산 (수치 미분)
            new_velocities = []
            if self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 0:
                    for i in range(8):
                        velocity = (new_positions[i] - self.prev_positions[i]) / dt
                        new_velocities.append(velocity)
                else:
                    new_velocities = [0.0] * 8
            else:
                new_velocities = [0.0] * 8
            
            # 데이터 업데이트
            self.joint_positions = new_positions
            self.joint_velocities = new_velocities
            self.prev_positions = new_positions.copy()
            self.prev_time = current_time
        
        return True
    
    def get_isaac_lab_data(self):
        """Isaac Lab 호환 형식으로 조인트 데이터 반환"""
        # 최신 데이터로 업데이트
        self.update_joint_data()
        
        with self.lock:
            return {
                'joint_positions': self.joint_positions.copy(),    # rad
                'joint_velocities': self.joint_velocities.copy()   # rad/s
            }
    
    def get_joint_positions(self):
        """조인트 위치 반환 (rad)"""
        with self.lock:
            return self.joint_positions.copy()
    
    def get_joint_velocities(self):
        """조인트 속도 반환 (rad/s)"""  
        with self.lock:
            return self.joint_velocities.copy()
    
    def move_joint(self, isaac_joint_idx, angle_deg):
        """
        특정 조인트를 각도로 이동 (Isaac Lab 인덱스 사용)
        🔧 방향 반전 자동 적용
        """
        if not (0 <= isaac_joint_idx < 8):
            return False
            
        motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[isaac_joint_idx]
        if motor_id not in self.connected_motors:
            return False
        
        # 🔧 방향 반전 적용 (쓰기 시)
        actual_angle = angle_deg
        if JOINT_DIRECTION_INVERT[isaac_joint_idx]:
            actual_angle = -angle_deg  # True일 때 부호 반전
            if VERBOSE_LOGGING:
                print(f"조인트 {isaac_joint_idx} 방향 반전: {angle_deg}° → {actual_angle}°")
        
        # 안전 범위 제한
        safe_angle = max(MOTOR_MIN_ANGLE, min(MOTOR_MAX_ANGLE, abs(actual_angle)))
        if actual_angle < 0:
            safe_angle = -safe_angle
        
        position = self._angle_to_position(safe_angle)
        
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_GOAL_POSITION, position
        )
        
        success = result == COMM_SUCCESS and error == 0
        
        if DEBUG_MODE and success:
            joint_name = JOINT_NAMES[isaac_joint_idx]
            invert_info = " (반전)" if JOINT_DIRECTION_INVERT[isaac_joint_idx] else ""
            print(f"🎯 조인트 {isaac_joint_idx} ({joint_name}) → 모터ID {motor_id}: {angle_deg}°{invert_info}")
        
        return success
    
    def move_all_joints(self, angles_deg):
        """
        모든 조인트를 각도로 이동 (Isaac Lab 순서)
        🔧 자동 방향 반전 적용
        """
        if len(angles_deg) != 8:
            if DEBUG_MODE:
                print(f"❌ 각도 배열 크기 오류: {len(angles_deg)}, 기대값: 8")
            return False
        
        if DEBUG_MODE:
            print(f"🎯 모든 조인트 이동 명령:")
            for i, angle in enumerate(angles_deg):
                joint_name = JOINT_NAMES[i]
                invert_info = " (반전)" if JOINT_DIRECTION_INVERT[i] else ""
                print(f"   조인트 {i} ({joint_name}): {angle}°{invert_info}")
        
        success_count = 0
        for isaac_joint_idx, angle in enumerate(angles_deg):
            if self.move_joint(isaac_joint_idx, angle):
                success_count += 1
            else:
                if VERBOSE_LOGGING:
                    print(f"⚠️ 조인트 {isaac_joint_idx} 이동 실패")
        
        success = success_count >= 6  # 8개 중 최소 6개 성공하면 OK
        
        if DEBUG_MODE:
            print(f"✅ 조인트 이동 결과: {success_count}/8 성공")
        
        return success
    
    # 🚗 바퀴 제어 메소드들 (NEW!)
    def set_wheel_velocity(self, wheel_id, velocity_rpm):
        """바퀴 속도 설정 (RPM 단위)"""
        if wheel_id not in self.connected_wheels:
            if VERBOSE_LOGGING:
                print(f"❌ 바퀴 {wheel_id}가 연결되지 않음")
            return False
        
        try:
            # 방향 반전 적용
            actual_velocity = apply_wheel_direction_inversion(wheel_id, velocity_rpm)
            
            # 안전 범위 제한
            safe_velocity = max(-WHEEL_MAX_VELOCITY, min(WHEEL_MAX_VELOCITY, actual_velocity))
            
            # Dynamixel 속도 값으로 변환 (RPM을 그대로 사용)
            velocity_value = int(safe_velocity)
            
            # 속도 명령 전송
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, wheel_id, self.ADDR_GOAL_VELOCITY, velocity_value
            )
            
            success = result == COMM_SUCCESS and error == 0
            
            if VERBOSE_LOGGING and success:
                wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                invert_info = " (반전)" if WHEEL_DIRECTION_INVERT.get(wheel_id, False) else ""
                print(f"🚗 바퀴 {wheel_id} ({wheel_pos}): {velocity_rpm}→{safe_velocity} RPM{invert_info}")
            
            return success
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"❌ 바퀴 {wheel_id} 속도 설정 오류: {e}")
            return False
    
    def set_all_wheel_velocities(self, velocities):
        """모든 바퀴 속도 설정"""
        if not isinstance(velocities, dict):
            print("❌ 속도는 딕셔너리 형태여야 함: {wheel_id: rpm}")
            return False
        
        success_count = 0
        for wheel_id, velocity in velocities.items():
            if self.set_wheel_velocity(wheel_id, velocity):
                success_count += 1
        
        return success_count >= len(self.connected_wheels) // 2  # 절반 이상 성공하면 OK
    
    def stop_wheel(self, wheel_id):
        """특정 바퀴 정지"""
        return self.set_wheel_velocity(wheel_id, 0)
    
    def stop_all_wheels(self):
        """모든 바퀴 정지"""
        stop_velocities = {wheel_id: 0 for wheel_id in self.connected_wheels}
        return self.set_all_wheel_velocities(stop_velocities)
    
    def get_wheel_velocity(self, wheel_id):
        """바퀴 현재 속도 읽기 (RPM)"""
        if wheel_id not in self.connected_wheels:
            return 0
        
        try:
            velocity, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, wheel_id, self.ADDR_PRESENT_VELOCITY
            )
            
            if result == COMM_SUCCESS and error == 0:
                # 방향 반전 적용 (읽기 시에도)
                actual_velocity = apply_wheel_direction_inversion(wheel_id, velocity)
                return actual_velocity
            else:
                return 0
        except:
            return 0
    
    def get_all_wheel_velocities(self):
        """모든 바퀴 현재 속도 읽기"""
        velocities = {}
        for wheel_id in self.connected_wheels:
            velocities[wheel_id] = self.get_wheel_velocity(wheel_id)
        return velocities
    
    def move_robot(self, vx, vy, omega):
        """
        🚗 홀로노믹 로봇 이동
        
        Args:
            vx: 전후 속도 (양수: 전진)
            vy: 좌우 속도 (양수: 우측)
            omega: 회전 속도 (양수: 반시계)
        """
        # 옴니휠 운동학 계산
        wheel_velocities = calculate_omni_wheel_velocities(vx, vy, omega)
        
        # 바퀴 속도 설정
        return self.set_all_wheel_velocities(wheel_velocities)
    
    def get_debug_info(self):
        """디버그 정보 반환"""
        return {
            'connected_motors': self.connected_motors,
            'connected_wheels': self.connected_wheels,
            'isaac_mapping': ISAAC_TO_DYNAMIXEL_MAPPING,
            'joint_names': JOINT_NAMES,
            'direction_invert': JOINT_DIRECTION_INVERT,
            'wheel_positions': WHEEL_POSITIONS,
            'wheel_direction_invert': WHEEL_DIRECTION_INVERT,
            'motor_direction_info': get_motor_direction_info(),
            'wheel_info': get_wheel_info()
        }
    
    def test_direction_inversion(self):
        """방향 반전 테스트"""
        if not self.is_connected:
            print("❌ 모터가 연결되지 않음")
            return False
        
        print("🧪 모터 센서 방향 반전 테스트 시작...")
        
        # 테스트 각도들
        test_angles = [10, -10, 15, -15]
        
        for angle in test_angles:
            print(f"\n모든 조인트를 {angle}°로 이동:")
            if self.move_all_joints([angle] * 8):
                time.sleep(1.5)
                
                # 현재 위치 확인
                data = self.get_isaac_lab_data()
                print("현재 조인트 위치 (읽기):")
                for i, pos_rad in enumerate(data['joint_positions']):
                    pos_deg = pos_rad * RAD_TO_DEG
                    joint_name = JOINT_NAMES[i]
                    invert_info = " (반전됨)" if JOINT_DIRECTION_INVERT[i] else ""
                    print(f"  조인트 {i} ({joint_name}): {pos_deg:.1f}°{invert_info}")
            else:
                print("❌ 이동 실패")
        
        # 홈 포지션으로 복귀
        print(f"\n홈 포지션 (0도)으로 복귀:")
        self.move_all_joints([0] * 8)
        
        print("✅ 방향 반전 테스트 완료")
        return True
    
    def test_wheel_control(self):
        """🚗 바퀴 제어 테스트"""
        if not self.connected_wheels:
            print("❌ 연결된 바퀴가 없음")
            return False
        
        print("🚗 바퀴 제어 테스트 시작...")
        
        # 테스트 속도들
        test_speeds = [15, -15, 30, -30]
        
        for speed in test_speeds:
            print(f"\n모든 바퀴를 {speed} RPM으로 설정:")
            wheel_velocities = {wheel_id: speed for wheel_id in self.connected_wheels}
            
            if self.set_all_wheel_velocities(wheel_velocities):
                time.sleep(2)
                
                # 현재 속도 확인
                current_velocities = self.get_all_wheel_velocities()
                print("현재 바퀴 속도:")
                for wheel_id, velocity in current_velocities.items():
                    wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                    invert_info = " (반전됨)" if WHEEL_DIRECTION_INVERT.get(wheel_id, False) else ""
                    print(f"  바퀴 {wheel_id} ({wheel_pos}): {velocity} RPM{invert_info}")
            else:
                print("❌ 속도 설정 실패")
        
        # 바퀴 정지
        print(f"\n모든 바퀴 정지:")
        self.stop_all_wheels()
        
        print("✅ 바퀴 제어 테스트 완료")
        return True
    
    def disconnect(self):
        """모터 연결 해제"""
        if not self.is_connected:
            return
            
        # 모든 모터 토크 비활성화
        for motor_id in self.connected_motors:
            self._disable_torque(motor_id)
        
        # 모든 바퀴 정지 및 토크 비활성화
        for wheel_id in self.connected_wheels:
            self.stop_wheel(wheel_id)
            self._disable_torque(wheel_id)
                
        self.port_handler.closePort()
        self.is_connected = False
        
        if DEBUG_MODE:
            print("🔌 모터 연결 해제")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 모터 센서 테스트 시작...")
    
    try:
        motor = MotorSensor()
        
        if motor.connect():
            print("\n📊 방향 반전 설정 확인:")
            debug_info = motor.get_debug_info()
            
            # 다리 모터 정보
            print("🦵 다리 모터:")
            for info in debug_info['motor_direction_info']:
                invert_str = "반전" if info['direction_inverted'] else "정방향"
                print(f"  조인트{info['joint_index']} ({info['joint_name']}) → 모터ID{info['motor_id']} - {info['side']} - {invert_str}")
            
            # 바퀴 모터 정보
            print("\n🚗 바퀴 모터:")
            for info in debug_info['wheel_info']:
                invert_str = "반전" if info['direction_inverted'] else "정방향"
                print(f"  {info['position']} ({info['side']} {info['position_name']}) → 모터ID{info['wheel_id']} - {invert_str}")
            
            print("\n🔧 방향 반전 테스트:")
            motor.test_direction_inversion()
            
            print("\n🚗 바퀴 제어 테스트:")
            motor.test_wheel_control()
            
            print("\n📊 5초간 조인트 데이터 모니터링...")
            for i in range(5):
                data = motor.get_isaac_lab_data()
                print(f"\nFrame {i+1}:")
                print("조인트 위치 (rad):")
                for j, (pos, name) in enumerate(zip(data['joint_positions'], JOINT_NAMES)):
                    invert_info = " (반전)" if JOINT_DIRECTION_INVERT[j] else ""
                    print(f"  {name}: {pos:.4f} rad ({pos*RAD_TO_DEG:.1f}°){invert_info}")
                
                print("조인트 속도 (rad/s):")
                for j, (vel, name) in enumerate(zip(data['joint_velocities'], JOINT_NAMES)):
                    print(f"  {name}: {vel:.4f} rad/s")
                
                # 바퀴 속도 확인
                wheel_velocities = motor.get_all_wheel_velocities()
                if wheel_velocities:
                    print("바퀴 속도 (RPM):")
                    for wheel_id, velocity in wheel_velocities.items():
                        wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                        print(f"  {wheel_pos}: {velocity} RPM")
                
                time.sleep(1)
        else:
            print("❌ 모터 연결 실패")
            
    except KeyboardInterrupt:
        print("테스트 중단")
    except Exception as e:
        print(f"테스트 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'motor' in locals():
            motor.disconnect()