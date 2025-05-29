#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
바퀴 컨트롤러
옴니휠을 이용한 홀로노믹 이동 제어
전진/후진/좌우이동/회전 기능 제공
"""

import time
import threading
import numpy as np
from config import *


class WheelController:
    def __init__(self, motor_sensor):
        """바퀴 컨트롤러 초기화"""
        self.motor_sensor = motor_sensor
        self.is_enabled = False
        self.is_moving = False
        
        # 성능 모니터링
        self.command_count = 0
        self.successful_commands = 0
        self.last_command_time = 0.0
        
        # 안전 설정
        self.max_speed = WHEEL_MAX_VELOCITY
        self.default_speed = WHEEL_DEFAULT_SPEED
        self.min_command_interval = 0.05  # 20Hz 최대
        
        if self.motor_sensor and len(self.motor_sensor.connected_wheels) > 0:
            self.is_enabled = True
            if DEBUG_MODE:
                print(f"✅ 바퀴 컨트롤러 초기화 완료")
                print(f"   연결된 바퀴: {len(self.motor_sensor.connected_wheels)}개")
                print(f"   최대 속도: {self.max_speed} RPM")
        else:
            print("❌ 바퀴 컨트롤러 초기화 실패 - 연결된 바퀴 없음")
    
    def _execute_wheel_command(self, velocities, duration=0):
        """바퀴 명령 실행 (내부 메소드)"""
        if not self.is_enabled:
            return False
        
        # 명령 간격 체크
        current_time = time.time()
        if (current_time - self.last_command_time) < self.min_command_interval:
            return False
        
        try:
            success = self.motor_sensor.set_all_wheel_velocities(velocities)
            
            if success:
                self.successful_commands += 1
                if duration > 0:
                    self.is_moving = True
            
            self.command_count += 1
            self.last_command_time = current_time
            
            return success
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"❌ 바퀴 명령 실행 오류: {e}")
            return False
    
    def move_forward(self, speed=None, duration=2.0):
        """전진 이동"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 🔧 전진: vx를 음수로 보내기 (방향 수정)
        velocities = calculate_omni_wheel_velocities(-speed, 0, 0)
        
        if DEBUG_MODE:
            print(f"🚗 전진: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            # 지정된 시간 후 정지
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def move_backward(self, speed=None, duration=2.0):
        """후진 이동"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 🔧 후진: vx를 양수로 보내기 (방향 수정)
        velocities = calculate_omni_wheel_velocities(speed, 0, 0)
        
        if DEBUG_MODE:
            print(f"🚗 후진: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def move_left(self, speed=None, duration=2.0):
        """좌측 이동"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 좌측: vy 음수
        velocities = calculate_omni_wheel_velocities(0, -speed, 0)
        
        if DEBUG_MODE:
            print(f"🚗 좌측 이동: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def move_right(self, speed=None, duration=2.0):
        """우측 이동"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 우측: vy 양수
        velocities = calculate_omni_wheel_velocities(0, speed, 0)
        
        if DEBUG_MODE:
            print(f"🚗 우측 이동: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def turn_left(self, speed=None, duration=2.0):
        """좌회전 (반시계방향)"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 좌회전: omega 양수 (반시계방향)
        velocities = calculate_omni_wheel_velocities(0, 0, speed)
        
        if DEBUG_MODE:
            print(f"🚗 좌회전: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def turn_right(self, speed=None, duration=2.0):
        """우회전 (시계방향)"""
        if speed is None:
            speed = self.default_speed
        
        speed = min(self.max_speed, abs(speed))  # 안전 제한
        
        # 우회전: omega 음수 (시계방향)
        velocities = calculate_omni_wheel_velocities(0, 0, -speed)
        
        if DEBUG_MODE:
            print(f"🚗 우회전: {speed} RPM, {duration}초")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def move_custom(self, vx, vy, omega, duration=0):
        """사용자 정의 이동"""
        # 안전 제한
        vx = max(-self.max_speed, min(self.max_speed, vx))
        vy = max(-self.max_speed, min(self.max_speed, vy))
        omega = max(-self.max_speed, min(self.max_speed, omega))
        
        velocities = calculate_omni_wheel_velocities(vx, vy, omega)
        
        if DEBUG_MODE:
            print(f"🚗 사용자 정의 이동: vx={vx}, vy={vy}, ω={omega}")
        
        success = self._execute_wheel_command(velocities)
        
        if success and duration > 0:
            threading.Timer(duration, self.stop).start()
        
        return success
    
    def stop(self):
        """바퀴 정지"""
        if not self.is_enabled:
            return False
        
        success = self.motor_sensor.stop_all_wheels()
        self.is_moving = False
        
        if DEBUG_MODE:
            print("🛑 바퀴 정지")
        
        return success
    
    def emergency_stop(self):
        """비상 정지"""
        print("🚨 바퀴 비상 정지!")
        return self.stop()
    
    def get_wheel_status(self):
        """바퀴 상태 정보 반환"""
        if not self.is_enabled:
            return None
        
        try:
            velocities = self.motor_sensor.get_all_wheel_velocities()
            
            status = {
                'is_enabled': self.is_enabled,
                'is_moving': self.is_moving,
                'wheel_velocities': velocities,
                'connected_wheels': len(self.motor_sensor.connected_wheels),
                'max_speed': self.max_speed,
                'command_count': self.command_count,
                'success_rate': (self.successful_commands / max(1, self.command_count)) * 100
            }
            
            return status
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"❌ 바퀴 상태 확인 오류: {e}")
            return None
    
    def print_wheel_status(self):
        """바퀴 상태 출력"""
        status = self.get_wheel_status()
        
        if status is None:
            print("❌ 바퀴 컨트롤러가 비활성화됨")
            return
        
        print(f"\n🚗 바퀴 컨트롤러 상태:")
        print(f"  활성화: {'✅' if status['is_enabled'] else '❌'}")
        print(f"  이동 중: {'✅' if status['is_moving'] else '❌'}")
        print(f"  연결된 바퀴: {status['connected_wheels']}개")
        print(f"  최대 속도: {status['max_speed']} RPM")
        print(f"  명령 횟수: {status['command_count']}")
        print(f"  성공률: {status['success_rate']:.1f}%")
        
        if status['wheel_velocities']:
            print(f"  현재 바퀴 속도:")
            for wheel_id, velocity in status['wheel_velocities'].items():
                wheel_pos = WHEEL_POSITIONS.get(wheel_id, f"ID{wheel_id}")
                print(f"    {wheel_pos}: {velocity} RPM")
    
    def test_individual_wheels(self, test_speed=20, test_duration=2.0):
        """개별 바퀴 테스트 (방향 확인용)"""
        if not self.is_enabled:
            print("❌ 바퀴 컨트롤러가 비활성화됨")
            return False
        
        print(f"🔧 개별 바퀴 방향 테스트 ({test_speed} RPM, {test_duration}초씩)")
        
        wheel_tests = [
            ("왼쪽 앞 (lf, ID3)", {3: test_speed, 6: 0, 9: 0, 12: 0}),
            ("오른쪽 앞 (rf, ID6)", {3: 0, 6: test_speed, 9: 0, 12: 0}),
            ("왼쪽 뒤 (lb, ID9)", {3: 0, 6: 0, 9: test_speed, 12: 0}),
            ("오른쪽 뒤 (rb, ID12)", {3: 0, 6: 0, 9: 0, 12: test_speed}),
        ]
        
        for name, velocities in wheel_tests:
            try:
                print(f"\n🚗 {name} 테스트...")
                if self.motor_sensor.set_all_wheel_velocities(velocities):
                    print(f"✅ {name} 회전 중 - 방향 확인하세요!")
                    time.sleep(test_duration)
                    self.motor_sensor.stop_all_wheels()
                    print(f"🛑 {name} 정지")
                else:
                    print(f"❌ {name} 실패")
                
                time.sleep(0.5)  # 테스트 간 대기
                
            except Exception as e:
                print(f"❌ {name} 테스트 오류: {e}")
                self.stop()
        
        print(f"\n✅ 개별 바퀴 테스트 완료")
        return True
    
    def test_all_movements(self, test_speed=20, test_duration=1.5):
        """모든 이동 패턴 테스트"""
        if not self.is_enabled:
            print("❌ 바퀴 컨트롤러가 비활성화됨")
            return False
        
        print(f"🧪 바퀴 이동 테스트 시작 ({test_speed} RPM, {test_duration}초씩)")
        
        movements = [
            ("전진", lambda: self.move_forward(test_speed, 0)),
            ("후진", lambda: self.move_backward(test_speed, 0)),
            ("좌측", lambda: self.move_left(test_speed, 0)),
            ("우측", lambda: self.move_right(test_speed, 0)),
            ("좌회전", lambda: self.turn_left(test_speed, 0)),
            ("우회전", lambda: self.turn_right(test_speed, 0))
        ]
        
        for name, movement_func in movements:
            try:
                print(f"\n🚗 {name} 테스트...")
                if movement_func():
                    time.sleep(test_duration)
                    self.stop()
                    print(f"✅ {name} 완료")
                else:
                    print(f"❌ {name} 실패")
                
                time.sleep(0.5)  # 테스트 간 대기
                
            except Exception as e:
                print(f"❌ {name} 테스트 오류: {e}")
                self.stop()
        
        print(f"\n✅ 바퀴 이동 테스트 완료")
        return True


# 테스트 코드
if __name__ == "__main__":
    print("🧪 바퀴 컨트롤러 테스트 시작...")
    
    try:
        from motor_sensor import MotorSensor
        
        motor = MotorSensor()
        
        if motor.connect():
            wheel_controller = WheelController(motor)
            
            if wheel_controller.is_enabled:
                # 상태 확인
                wheel_controller.print_wheel_status()
                
                # 모든 이동 테스트
                wheel_controller.test_all_movements()
                
                # 사용자 정의 이동 테스트
                print(f"\n🚗 사용자 정의 이동 테스트...")
                wheel_controller.move_custom(15, 10, 5, 2.0)  # 대각선 + 회전
                time.sleep(2.5)
                
                # 최종 상태 확인
                wheel_controller.print_wheel_status()
            else:
                print("❌ 바퀴 컨트롤러 초기화 실패")
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