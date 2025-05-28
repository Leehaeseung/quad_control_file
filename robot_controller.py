#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 컨트롤러 (방향 반전 적용)
AI 모델 출력을 실제 모터 제어 명령으로 변환
오른쪽 모터들의 회전방향 자동 보정
"""

import time
import numpy as np
from config import *
from motor_sensor import MotorSensor


class RobotController:
    def __init__(self):
        """로봇 컨트롤러 초기화"""
        print("🦾 로봇 컨트롤러 초기화...")
        
        # 모터 센서 (제어도 담당)
        self.motor_sensor = None
        
        # 제어 상태
        self.is_connected = False
        self.is_control_enabled = False
        
        # 안전 설정
        self.max_angle_change_per_step = 5.0  # 도/스텝
        self.min_command_interval = 0.01      # 초
        
        # 이전 상태 저장
        self.prev_joint_angles = np.zeros(8)
        self.prev_command_time = 0.0
        
        # 성능 모니터링
        self.command_count = 0
        self.successful_commands = 0
        
        if DEBUG_MODE:
            print("✅ 로봇 컨트롤러 초기화 완료")
            print("🔧 방향 반전 기능 활성화")
    
    def connect_motors(self):
        """모터 연결"""
        try:
            self.motor_sensor = MotorSensor()
            
            if self.motor_sensor.connect():
                self.is_connected = True
                
                # 현재 위치를 기준점으로 설정
                motor_data = self.motor_sensor.get_isaac_lab_data()
                self.prev_joint_angles = np.array(motor_data['joint_positions']) * RAD_TO_DEG
                
                print("✅ 로봇 컨트롤러 연결 완료")
                print(f"   현재 조인트 각도: {[f'{a:.1f}°' for a in self.prev_joint_angles]}")
                
                # 방향 반전 설정 확인
                if DEBUG_MODE:
                    print("🔧 방향 반전 설정:")
                    for i, invert in enumerate(JOINT_DIRECTION_INVERT):
                        joint_name = JOINT_NAMES[i]
                        status = "반전" if invert else "정방향"
                        print(f"   조인트 {i} ({joint_name}): {status}")
                
                return True
            else:
                print("❌ 모터 연결 실패")
                return False
                
        except Exception as e:
            print(f"❌ 로봇 컨트롤러 연결 오류: {e}")
            return False
    
    def enable_control(self):
        """제어 활성화"""
        if not self.is_connected:
            print("❌ 모터가 연결되지 않음")
            return False
        
        self.is_control_enabled = True
        print("🟢 로봇 제어 활성화 (방향 반전 적용)")
        return True
    
    def disable_control(self):
        """제어 비활성화"""
        self.is_control_enabled = False
        print("🔴 로봇 제어 비활성화")
    
    def apply_safety_limits(self, target_angles_deg):
        """
        안전 제한 적용
        🔧 방향 반전은 motor_sensor에서 자동 처리되므로 여기서는 안전 제한만 적용
        """
        if target_angles_deg is None or len(target_angles_deg) != 8:
            return self.prev_joint_angles.copy()
        
        # NumPy 배열로 변환
        target = np.array(target_angles_deg, dtype=np.float32)
        prev = np.array(self.prev_joint_angles, dtype=np.float32)
        
        # 1. 각도 범위 제한 (절댓값 기준)
        target = np.clip(target, -MOTOR_MAX_ANGLE, MOTOR_MAX_ANGLE)
        
        # 2. 변화량 제한
        angle_changes = target - prev
        max_changes = np.full(8, self.max_angle_change_per_step)
        
        # 변화량이 너무 크면 제한
        limited_changes = np.clip(angle_changes, -max_changes, max_changes)
        safe_angles = prev + limited_changes
        
        # 3. 최종 범위 재확인
        safe_angles = np.clip(safe_angles, -MOTOR_MAX_ANGLE, MOTOR_MAX_ANGLE)
        
        return safe_angles
    
    def execute_joint_command(self, ai_output_angles_deg):
        """
        AI 모델 출력을 실제 모터 명령으로 실행
        🔧 방향 반전 자동 적용
        
        Args:
            ai_output_angles_deg: AI 모델이 출력한 8개 조인트 각도 (도)
        
        Returns:
            bool: 명령 실행 성공 여부
        """
        if not self.is_control_enabled:
            if VERBOSE_LOGGING:
                print("⚠️ 제어가 비활성화됨")
            return False
        
        # 명령 간격 체크
        current_time = time.time()
        if (current_time - self.prev_command_time) < self.min_command_interval:
            return False  # 너무 빈번한 명령 무시
        
        try:
            # 안전 제한 적용
            safe_angles = self.apply_safety_limits(ai_output_angles_deg)
            
            if DEBUG_MODE and self.command_count % 30 == 0:  # 30번마다 출력
                print(f"🎯 조인트 명령 (방향 반전 자동 적용):")
                for i, (orig, safe) in enumerate(zip(ai_output_angles_deg, safe_angles)):
                    joint_name = JOINT_NAMES[i]
                    invert_info = " ⚡반전" if JOINT_DIRECTION_INVERT[i] else ""
                    if abs(orig - safe) > 0.1:  # 안전 제한이 적용된 경우
                        print(f"   조인트 {i} ({joint_name}): {orig:.1f}° → {safe:.1f}° (제한){invert_info}")
                    else:
                        print(f"   조인트 {i} ({joint_name}): {safe:.1f}°{invert_info}")
            
            # 🔧 모터 명령 실행 (motor_sensor에서 자동으로 방향 반전 처리)
            success = self.motor_sensor.move_all_joints(safe_angles)
            
            if success:
                self.prev_joint_angles = safe_angles.copy()
                self.successful_commands += 1
                
                if VERBOSE_LOGGING:
                    print(f"✅ 조인트 명령 실행 완료 (방향 반전 적용)")
            else:
                if VERBOSE_LOGGING:
                    print("❌ 조인트 명령 실행 실패")
            
            self.command_count += 1
            self.prev_command_time = current_time
            
            return success
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"조인트 명령 실행 오류: {e}")
            return False
    
    def execute_individual_joint(self, isaac_joint_idx, angle_deg):
        """
        개별 조인트 제어
        🔧 방향 반전 자동 적용
        """
        if not self.is_control_enabled:
            return False
        
        if not (0 <= isaac_joint_idx < 8):
            return False
        
        try:
            # 안전 제한
            current_angle = self.prev_joint_angles[isaac_joint_idx]
            angle_change = angle_deg - current_angle
            
            if abs(angle_change) > self.max_angle_change_per_step:
                if angle_change > 0:
                    angle_deg = current_angle + self.max_angle_change_per_step
                else:
                    angle_deg = current_angle - self.max_angle_change_per_step
            
            # 범위 제한
            safe_angle = np.clip(angle_deg, -MOTOR_MAX_ANGLE, MOTOR_MAX_ANGLE)
            
            if DEBUG_MODE:
                joint_name = JOINT_NAMES[isaac_joint_idx]
                invert_info = " (반전)" if JOINT_DIRECTION_INVERT[isaac_joint_idx] else ""
                print(f"🎯 개별 조인트 {isaac_joint_idx} ({joint_name}): {safe_angle:.1f}°{invert_info}")
            
            # 🔧 명령 실행 (motor_sensor에서 자동 방향 반전)
            if self.motor_sensor.move_joint(isaac_joint_idx, safe_angle):
                self.prev_joint_angles[isaac_joint_idx] = safe_angle
                return True
            else:
                return False
                
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"개별 조인트 제어 오류: {e}")
            return False
    
    def get_current_joint_angles(self):
        """현재 조인트 각도 반환 (도, 방향 반전 적용됨)"""
        if not self.is_connected:
            return np.zeros(8)
        
        try:
            motor_data = self.motor_sensor.get_isaac_lab_data()
            angles_rad = np.array(motor_data['joint_positions'])
            angles_deg = angles_rad * RAD_TO_DEG
            return angles_deg
        except:
            return self.prev_joint_angles.copy()
    
    def move_to_home_position(self, home_angles=None):
        """
        홈 포지션으로 이동
        🔧 방향 반전 자동 적용
        """
        if home_angles is None:
            home_angles = np.zeros(8)  # 0도 홈 포지션
        
        print("🏠 홈 포지션으로 이동 중... (방향 반전 적용)")
        
        if not self.is_control_enabled:
            print("❌ 제어가 비활성화됨")
            return False
        
        try:
            # 점진적으로 홈 포지션으로 이동
            current_angles = self.get_current_joint_angles()
            
            max_steps = 20  # 최대 스텝 수
            for step in range(max_steps):
                # 목표와 현재 위치의 차이 계산
                diff = np.array(home_angles) - current_angles
                
                # 완료 체크
                if np.max(np.abs(diff)) < 1.0:  # 1도 이내
                    print(f"✅ 홈 포지션 도달 완료 (스텝 {step+1})")
                    break
                
                # 점진적 이동
                step_size = np.clip(diff, -self.max_angle_change_per_step, self.max_angle_change_per_step)
                target_angles = current_angles + step_size
                
                # 안전 제한 적용
                safe_angles = self.apply_safety_limits(target_angles)
                
                # 🔧 모터 명령 실행 (방향 반전 자동 적용)
                if self.motor_sensor.move_all_joints(safe_angles):
                    current_angles = safe_angles
                    self.prev_joint_angles = safe_angles.copy()
                    
                    if DEBUG_MODE:
                        max_diff = np.max(np.abs(diff))
                        invert_count = sum(JOINT_DIRECTION_INVERT)
                        print(f"스텝 {step+1}/{max_steps}: 최대 차이 {max_diff:.1f}° (반전 적용: {invert_count}개)")
                else:
                    print(f"⚠️ 스텝 {step+1}에서 모터 이동 실패")
                
                # 짧은 대기
                time.sleep(0.1)
            
            else:
                print(f"⚠️ 홈 포지션 이동 미완료 (최대 스텝 {max_steps} 도달)")
                return False
            
            return True
            
        except Exception as e:
            print(f"❌ 홈 포지션 이동 오류: {e}")
            return False
    
    def test_direction_inversion(self):
        """방향 반전 테스트"""
        if not self.is_control_enabled:
            print("❌ 제어가 활성화되지 않음")
            return False
        
        print("🧪 로봇 컨트롤러 방향 반전 테스트 시작...")
        
        # 테스트 각도들
        test_angles = [10, -10, 15, -15]
        
        for angle in test_angles:
            print(f"\n모든 조인트를 {angle}°로 이동:")
            if self.execute_joint_command([angle] * 8):
                time.sleep(1.5)
                
                # 현재 위치 확인
                current = self.get_current_joint_angles()
                print("현재 조인트 위치:")
                for i, pos in enumerate(current):
                    joint_name = JOINT_NAMES[i]
                    invert_info = " (반전됨)" if JOINT_DIRECTION_INVERT[i] else ""
                    print(f"  조인트 {i} ({joint_name}): {pos:.1f}°{invert_info}")
            else:
                print("❌ 이동 실패")
        
        # 홈 포지션으로 복귀
        print(f"\n홈 포지션으로 복귀:")
        self.move_to_home_position()
        
        print("✅ 방향 반전 테스트 완료")
        return True
    
    def emergency_stop(self):
        """비상 정지"""
        print("🚨 비상 정지!")
        
        self.disable_control()
        
        if self.motor_sensor and self.motor_sensor.is_connected:
            try:
                # 모든 모터 토크 비활성화
                for motor_id in self.motor_sensor.connected_motors:
                    self.motor_sensor._disable_torque(motor_id)
                print("✅ 모든 모터 토크 비활성화 완료")
            except Exception as e:
                print(f"❌ 비상 정지 중 오류: {e}")
    
    def get_control_stats(self):
        """제어 통계 반환"""
        success_rate = 0.0
        if self.command_count > 0:
            success_rate = (self.successful_commands / self.command_count) * 100
        
        return {
            'total_commands': self.command_count,
            'successful_commands': self.successful_commands,
            'success_rate_percent': success_rate,
            'is_control_enabled': self.is_control_enabled,
            'is_connected': self.is_connected,
            'direction_inversion_enabled': True,
            'inverted_joints': sum(JOINT_DIRECTION_INVERT)
        }
    
    def disconnect(self):
        """로봇 컨트롤러 연결 해제"""
        self.disable_control()
        
        if self.motor_sensor:
            self.motor_sensor.disconnect()
            self.motor_sensor = None
        
        self.is_connected = False
        print("🔌 로봇 컨트롤러 연결 해제")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 로봇 컨트롤러 테스트 시작...")
    
    try:
        controller = RobotController()
        
        if controller.connect_motors():
            if controller.enable_control():
                print("\n🔧 방향 반전 테스트:")
                controller.test_direction_inversion()
                
                print("\n📊 제어 통계:")
                stats = controller.get_control_stats()
                for key, value in stats.items():
                    print(f"  {key}: {value}")
            else:
                print("❌ 제어 활성화 실패")
        else:
            print("❌ 모터 연결 실패")
            
    except KeyboardInterrupt:
        print("테스트 중단")
    except Exception as e:
        print(f"테스트 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.disconnect()