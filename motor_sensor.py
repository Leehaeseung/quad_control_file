#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamixel 모터 센서 모듈 (방향 반전 적용)
조인트 위치와 속도를 Isaac Lab 형식으로 제공 
실제 모터 ID를 Isaac Lab 조인트 순서로 매핑
오른쪽 모터들의 회전방향 자동 보정
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
        
        # Dynamixel 설정
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_POSITION_P_GAIN = 84
        self.ADDR_POSITION_I_GAIN = 85
        self.ADDR_POSITION_D_GAIN = 86
        
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
        
        # Isaac Lab 형식 데이터
        self.joint_positions = [0.0] * 8      # rad
        self.joint_velocities = [0.0] * 8     # rad/s
        
        # 속도 계산용
        self.prev_positions = [0.0] * 8
        self.prev_time = None
        
        # 스레드 안전성
        self.lock = threading.Lock()
        
        # PID 게인 기본값
        self.default_p_gain = 2000
        self.default_i_gain = 800
        self.default_d_gain = 3000
        
        if DEBUG_MODE:
            print(f"✅ 모터 센서 초기화: {port}")
            print(f"🔧 방향 반전 설정 로드됨")
    
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
        for motor_id in self.motor_ids:
            if self._test_motor_connection(motor_id):
                self.connected_motors.append(motor_id)
                if DEBUG_MODE:
                    print(f"✅ 모터 ID {motor_id} 연결")
                self._set_pid_gains(motor_id)
                self._enable_torque(motor_id)
            else:
                print(f"❌ 모터 ID {motor_id} 연결 실패")
        
        if len(self.connected_motors) == 0:
            print("❌ 연결된 모터가 없습니다!")
            return False
            
        self.is_connected = True
        if DEBUG_MODE:
            print(f"🎉 총 {len(self.connected_motors)}개 모터 연결 완료!")
            print(f"연결된 모터: {self.connected_motors}")
            
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
                    
                    # 🔧 방향 반전 적용 (읽기 시에도 적용) - 수정됨!
                    if JOINT_DIRECTION_INVERT[isaac_joint_idx]:
                        angle_deg = -angle_deg  # True일 때 부호 반전 (시계 → 양수)
                    
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
        🔧 방향 반전 자동 적용 - 수정됨!
        """
        if not (0 <= isaac_joint_idx < 8):
            return False
            
        motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[isaac_joint_idx]
        if motor_id not in self.connected_motors:
            return False
        
        # 🔧 방향 반전 적용 (쓰기 시) - 수정됨!
        actual_angle = angle_deg
        if JOINT_DIRECTION_INVERT[isaac_joint_idx]:
            actual_angle = -angle_deg  # True일 때 부호 반전 (오른쪽을 시계방향으로)
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
        🔧 자동 방향 반전 적용 - 수정됨!
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
    
    def get_debug_info(self):
        """디버그 정보 반환"""
        return {
            'connected_motors': self.connected_motors,
            'isaac_mapping': ISAAC_TO_DYNAMIXEL_MAPPING,
            'joint_names': JOINT_NAMES,
            'direction_invert': JOINT_DIRECTION_INVERT,
            'motor_direction_info': get_motor_direction_info()
        }
    
    def test_direction_inversion(self):
        """방향 반전 테스트"""
        if not self.is_connected:
            print("❌ 모터가 연결되지 않음")
            return False
        
        print("🧪 방향 반전 테스트 시작...")
        
        test_angle = 10  # 10도 테스트
        
        print(f"모든 조인트를 {test_angle}도로 이동:")
        self.move_all_joints([test_angle] * 8)
        time.sleep(2)
        
        # 현재 위치 확인
        data = self.get_isaac_lab_data()
        print("\n현재 조인트 위치 (읽기):")
        for i, pos_rad in enumerate(data['joint_positions']):
            pos_deg = pos_rad * RAD_TO_DEG
            joint_name = JOINT_NAMES[i]
            invert_info = " (반전)" if JOINT_DIRECTION_INVERT[i] else ""
            print(f"  조인트 {i} ({joint_name}): {pos_deg:.1f}°{invert_info}")
        
        # 홈 포지션으로 복귀
        print(f"\n홈 포지션 (0도)으로 복귀:")
        self.move_all_joints([0] * 8)
        
        print("✅ 방향 반전 테스트 완료")
        return True
    
    def disconnect(self):
        """모터 연결 해제"""
        if not self.is_connected:
            return
            
        # 모든 모터 토크 비활성화
        for motor_id in self.connected_motors:
            self._disable_torque(motor_id)
                
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
            for info in debug_info['motor_direction_info']:
                invert_str = "반전" if info['direction_inverted'] else "정방향"
                print(f"  조인트{info['joint_index']} ({info['joint_name']}) → 모터ID{info['motor_id']} - {info['side']} - {invert_str}")
            
            print("\n🔧 방향 반전 테스트:")
            motor.test_direction_inversion()
            
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
