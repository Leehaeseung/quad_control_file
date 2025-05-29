#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
완전한 시스템 데모 - 걷기 + 바퀴 제어 통합
V2, V3 걷기 패턴 + 옴니휠 홀로노믹 이동
"""

import os
import sys
import time
import cv2
import threading
import numpy as np
from config import *
from data_combiner import DataCombiner
from robot_controller import RobotController
from walking_controller import WalkingController
from wheel_controller import WheelController

# 크로스 플랫폼 키 입력 함수
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class CompleteSystemDemo:
    def __init__(self):
        """완전한 시스템 데모 초기화"""
        print("🚀 완전한 시스템 데모 초기화 중... (걷기 + 바퀴)")
        
        self.data_combiner = DataCombiner()
        self.robot_controller = RobotController()
        self.walking_controller = None
        self.wheel_controller = None
        
        self.is_running = False
        self.monitoring_mode = False
        
        print("✅ 완전한 시스템 데모 초기화 완료")
    
    def initialize_system(self):
        """시스템 초기화"""
        print("🔧 통합 시스템 초기화 중...")
        
        success_count = 0
        
        # 센서 시스템 초기화
        try:
            if self.data_combiner.start_data_collection():
                print("✅ 센서 시스템 초기화 성공")
                success_count += 1
            else:
                print("❌ 센서 시스템 초기화 실패")
        except Exception as e:
            print(f"❌ 센서 시스템 오류: {e}")
        
        # 로봇 제어기 초기화
        try:
            if self.robot_controller.connect_motors():
                if self.robot_controller.enable_control():
                    print("✅ 로봇 제어기 초기화 성공")
                    success_count += 1
                    
                    # 걷기 컨트롤러 초기화
                    self.walking_controller = WalkingController(self.robot_controller)
                    print("✅ 걷기 컨트롤러 초기화 성공")
                    
                    # 바퀴 컨트롤러 초기화
                    if self.robot_controller.motor_sensor:
                        self.wheel_controller = WheelController(self.robot_controller.motor_sensor)
                        if self.wheel_controller.is_enabled:
                            print("✅ 바퀴 컨트롤러 초기화 성공")
                        else:
                            print("⚠️ 바퀴 컨트롤러 초기화 실패 (바퀴 모터 없음)")
                else:
                    print("⚠️ 로봇 제어 활성화 실패")
            else:
                print("❌ 로봇 제어기 초기화 실패")
        except Exception as e:
            print(f"❌ 로봇 제어기 오류: {e}")
        
        print(f"\n📊 초기화 결과: {success_count}/2 성공")
        return success_count > 0
    
    def show_help(self):
        """도움말 표시"""
        print("\n" + "="*80)
        print("🎮 완전한 로봇 데모 - 걷기 + 바퀴 제어")
        print("="*80)
        print("📊 시스템:")
        print("  s: 시스템 상태")
        print("  h: 홈 포지션")
        print("  m: 모니터링 토글")
        print()
        print("🚶 걷기 제어 (10초간):")
        print("  2: V2 연속 밀어내기 - 매끄러운 연속 움직임")
        print("  3: V3 4포인트 보행 - 불연속 구간 제거")
        print("  t: 모든 걷기 버전 순차 테스트")
        print("  x: 걷기 중지")
        print("  z: 걷기 상태 확인")
        print()
        print("🚗 바퀴 제어 (옴니휠, 2초간):")
        print("  w: 전진")
        print("  a: 좌측 이동")
        print("  d: 우측 이동")
        print("  백스페이스: 후진")
        print("  [: 좌회전 (반시계)")
        print("  e: 우회전 (시계)")
        print("  스페이스: 바퀴 정지")
        print("  ]: 바퀴 전체 테스트")
        print()
        print("🎮 통합 제어:")
        print("  f: 앞으로 걸으면서 바퀴로 이동")
        print("  b: 뒤로 걸으면서 바퀴로 이동") 
        print("  l: 왼쪽으로 걸으면서 바퀴로 회전")
        print("  r: 오른쪽으로 걸으면서 바퀴로 회전")
        print()
        print("🦾 기본 제어:")
        print("  0-9: 각도 이동 (-20°~63° 범위)")
        print("  r: 범위 테스트")
        print()
        print("📷 카메라:")
        print("  v: 이미지 표시")
        print("  c: 통계")
        print()
        print("🔧 기타:")
        print("  help: 도움말")
        print("  ESC/q: 종료")
        print("="*80)
    
    def show_system_status(self):
        """시스템 상태 출력"""
        print("\n🔍 완전한 시스템 상태:")
        print("="*60)
        
        try:
            self.data_combiner.update_data()
            self.data_combiner.print_system_status()
            
            if self.robot_controller.is_connected:
                control_stats = self.robot_controller.get_control_stats()
                print(f"\n🦾 로봇 제어기:")
                print(f"  연결됨: ✅")
                print(f"  제어 활성화: {'✅' if control_stats['is_control_enabled'] else '❌'}")
                print(f"  총 명령 수: {control_stats['total_commands']}")
                print(f"  성공률: {control_stats['success_rate_percent']:.1f}%")
            
            if self.walking_controller:
                print(f"\n🚶 걷기 컨트롤러:")
                print(f"  초기화됨: ✅")
                print(f"  걷기 중: {'✅' if self.walking_controller.walking else '❌'}")
                print(f"  현재 버전: V{self.walking_controller.current_version}")
                print(f"  LEG 범위: -20°~63° (뒤로 밀기 강화)")
                print(f"  KNEE 범위: 0°~90° (확장된 범위)")
            
            if self.wheel_controller:
                print(f"\n🚗 바퀴 컨트롤러:")
                wheel_status = self.wheel_controller.get_wheel_status()
                if wheel_status:
                    print(f"  초기화됨: ✅")
                    print(f"  이동 중: {'✅' if wheel_status['is_moving'] else '❌'}")
                    print(f"  연결된 바퀴: {wheel_status['connected_wheels']}개")
                    print(f"  최대 속도: {wheel_status['max_speed']} RPM")
                    print(f"  성공률: {wheel_status['success_rate']:.1f}%")
                else:
                    print(f"  초기화됨: ❌")
            else:
                print(f"\n🚗 바퀴 컨트롤러: ❌ (바퀴 모터 없음)")
                
        except Exception as e:
            print(f"❌ 시스템 상태 오류: {e}")
        
        print("="*60)
    
    def move_all_motors(self, target_angle_deg):
        """모든 모터 이동"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        try:
            print(f"🎯 모든 모터를 {target_angle_deg}°로 이동...")
            angles = [target_angle_deg] * 8
            
            if self.robot_controller.motor_sensor.move_all_joints(angles):
                print(f"✅ 모터 이동 완료")
                time.sleep(1)
            else:
                print("❌ 모터 이동 실패")
        except Exception as e:
            print(f"❌ 모터 이동 오류: {e}")
    
    def move_to_home(self):
        """홈 포지션으로 이동"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        try:
            print("🏠 홈 포지션으로 이동...")
            home_angles = [0.0] * 8
            
            if self.robot_controller.motor_sensor.move_all_joints(home_angles):
                print("✅ 홈 포지션 이동 완료")
                time.sleep(1)
            else:
                print("❌ 홈 포지션 이동 실패")
        except Exception as e:
            print(f"❌ 홈 포지션 이동 오류: {e}")
    
    def test_range_safely(self):
        """안전한 범위 테스트"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        try:
            print("🧪 확장된 범위 테스트")
            print("  LEG: -20°~63° (뒤로 밀기 강화)")
            print("  KNEE: 0°~90° (확장된 무릎 범위)")
            
            # LEG 테스트 각도들
            leg_test_angles = [-20, -10, 0, 15, 30, 45, 60, 63]
            
            print("\n🦵 LEG 각도 테스트:")
            for i, angle in enumerate(leg_test_angles):
                try:
                    print(f"LEG 스텝 {i+1}/{len(leg_test_angles)}: {angle}°")
                    joint_angles = [angle, 30, angle, 30, angle, 30, angle, 30]
                    
                    if self.robot_controller.motor_sensor.move_all_joints(joint_angles):
                        print(f"✅ LEG {angle}° 이동 성공")
                    else:
                        print(f"❌ LEG {angle}° 이동 실패")
                    
                    time.sleep(1.0)
                except Exception as e:
                    print(f"⚠️ LEG 스텝 {i+1} 오류: {e}")
                    continue
            
            print("✅ 범위 테스트 완료!")
            
        except Exception as e:
            print(f"❌ 범위 테스트 오류: {e}")
    
    # 🚶 걷기 제어 메소드들
    def start_walking_version(self, version):
        """특정 버전 걷기 시작"""
        if self.walking_controller:
            try:
                self.walking_controller.start_walking_version(version, duration=10.0, speed=1.0)
            except Exception as e:
                print(f"❌ 걷기 V{version} 시작 오류: {e}")
        else:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
    
    def test_all_walking_versions(self):
        """모든 걷기 버전 순차 테스트"""
        if self.walking_controller:
            try:
                self.walking_controller.test_all_versions(duration_each=5.0)
            except Exception as e:
                print(f"❌ 전체 걷기 테스트 오류: {e}")
        else:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
    
    def stop_walking(self):
        """걷기 중지"""
        if self.walking_controller:
            try:
                self.walking_controller.stop_walking()
            except Exception as e:
                print(f"❌ 걷기 중지 오류: {e}")
        else:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
    
    def check_walking_status(self):
        """걷기 상태 확인"""
        if self.walking_controller:
            try:
                status = "걷기 중" if self.walking_controller.walking else "정지"
                current_v = self.walking_controller.current_version
                
                version_names = {
                    2: "연속 밀어내기 (멈춤 없는 연속성)",
                    3: "4포인트 보행 사이클 (불연속 구간 제거)"
                }
                
                print(f"🚶 걷기 상태: {status}")
                print(f"  현재 버전: V{current_v} - {version_names.get(current_v, '알 수 없음')}")
            except Exception as e:
                print(f"❌ 걷기 상태 확인 오류: {e}")
        else:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
    
    # 🚗 바퀴 제어 메소드들
    def move_wheels_direction(self, direction, speed=WHEEL_DEFAULT_SPEED, duration=2.0):
        """바퀴로 특정 방향 이동"""
        if not self.wheel_controller or not self.wheel_controller.is_enabled:
            print("❌ 바퀴 컨트롤러가 초기화되지 않음")
            return
        
        try:
            if direction == 'forward':
                self.wheel_controller.move_forward(speed, duration)
            elif direction == 'backward':
                self.wheel_controller.move_backward(speed, duration)
            elif direction == 'left':
                self.wheel_controller.move_left(speed, duration)
            elif direction == 'right':
                self.wheel_controller.move_right(speed, duration)
            elif direction == 'turn_left':
                self.wheel_controller.turn_left(speed, duration)
            elif direction == 'turn_right':
                self.wheel_controller.turn_right(speed, duration)
            else:
                print(f"❌ 알 수 없는 방향: {direction}")
        except Exception as e:
            print(f"❌ 바퀴 이동 오류: {e}")
    
    def stop_wheels(self):
        """바퀴 정지"""
        if self.wheel_controller:
            try:
                self.wheel_controller.stop()
                print("🛑 바퀴 정지")
            except Exception as e:
                print(f"❌ 바퀴 정지 오류: {e}")
        else:
            print("❌ 바퀴 컨트롤러가 초기화되지 않음")
    
    def test_wheel_movements(self):
        """바퀴 이동 테스트"""
        if self.wheel_controller:
            try:
                self.wheel_controller.test_all_movements()
            except Exception as e:
                print(f"❌ 바퀴 테스트 오러: {e}")
        else:
            print("❌ 바퀴 컨트롤러가 초기화되지 않음")
    
    # 🎮 통합 제어 메소드들 (걷기 + 바퀴 동시 제어)
    def combined_forward_movement(self):
        """앞으로 걸으면서 바퀴로도 전진"""
        print("🚀 통합 전진: 걷기 + 바퀴")
        if self.walking_controller:
            self.walking_controller.start_walking_version(2, duration=5.0, speed=0.8)
        if self.wheel_controller:
            self.wheel_controller.move_forward(WHEEL_SLOW_SPEED, 5.0)
    
    def combined_backward_movement(self):
        """뒤로 걸으면서 바퀴로도 후진"""
        print("🚀 통합 후진: 걷기 + 바퀴")
        if self.walking_controller:
            self.walking_controller.start_walking_version(3, duration=5.0, speed=0.8)
        if self.wheel_controller:
            self.wheel_controller.move_backward(WHEEL_SLOW_SPEED, 5.0)
    
    def combined_left_movement(self):
        """왼쪽으로 걸으면서 바퀴로 좌회전"""
        print("🚀 통합 좌측: 걷기 + 회전")
        if self.walking_controller:
            self.walking_controller.start_walking_version(2, duration=5.0, speed=0.6)
        if self.wheel_controller:
            self.wheel_controller.turn_left(WHEEL_SLOW_SPEED, 5.0)
    
    def combined_right_movement(self):
        """오른쪽으로 걸으면서 바퀴로 우회전"""
        print("🚀 통합 우측: 걷기 + 회전")
        if self.walking_controller:
            self.walking_controller.start_walking_version(3, duration=5.0, speed=0.6)
        if self.wheel_controller:
            self.wheel_controller.turn_right(WHEEL_SLOW_SPEED, 5.0)
    
    # 📷 카메라 메소드들
    def show_camera_image(self):
        """카메라 이미지 표시"""
        if not self.data_combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        try:
            print("📷 카메라 이미지 표시 중... (3초간)")
            start_time = time.time()
            
            while time.time() - start_time < 3.0:
                self.data_combiner.update_data()
                combined = self.data_combiner.camera.get_combined_image()
                
                if combined is not None:
                    cv2.imshow('Complete Demo - Camera', combined)
                    key = cv2.waitKey(30) & 0xFF
                    if key == 27:
                        break
                else:
                    break
            
            cv2.destroyAllWindows()
            print("📷 이미지 표시 종료")
        except Exception as e:
            print(f"❌ 카메라 이미지 오류: {e}")
    
    def show_camera_stats(self):
        """카메라 통계"""
        if not self.data_combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        try:
            self.data_combiner.update_data()
            stats = self.data_combiner.camera.get_depth_statistics()
            
            print(f"\n📊 카메라 통계:")
            print(f"  해상도: {CAMERA_WIDTH}×{CAMERA_HEIGHT}")
            print(f"  최소 깊이: {stats['min_depth']:.3f}m")
            print(f"  최대 깊이: {stats['max_depth']:.3f}m")
            print(f"  평균 깊이: {stats['mean_depth']:.3f}m")
            print(f"  유효 픽셀: {stats['valid_pixels']}/{stats['total_pixels']}")
        except Exception as e:
            print(f"❌ 카메라 통계 오류: {e}")
    
    def toggle_monitoring(self):
        """모니터링 모드 토글"""
        try:
            self.monitoring_mode = not self.monitoring_mode
            if self.monitoring_mode:
                print("🔄 자동 모니터링 시작")
            else:
                print("⏹️ 자동 모니터링 중지")
        except Exception as e:
            print(f"❌ 모니터링 토글 오류: {e}")
    
    def run_demo(self):
        """데모 실행"""
        if not self.initialize_system():
            print("❌ 시스템 초기화 실패")
            return False
        
        self.show_help()
        self.is_running = True
        
        # 키 매핑
        motor_angle_keys = {}
        for i in range(10):
            angle = -20 + i * 9  # -20, -11, -2, 7, 16, 25, 34, 43, 52, 61
            motor_angle_keys[str(i)] = min(63, angle)
        
        print("\n🎮 완전한 시스템 데모 시작!")
        print("걷기: 2, 3 / 바퀴: WASD / 통합: FBLR")
        
        try:
            while self.is_running:
                try:
                    key = getch()
                    
                    if key == chr(27) or key == 'q':  # ESC 또는 q
                        print("\n데모 종료...")
                        break
                    elif key == 's':
                        self.show_system_status()
                    elif key == 'h':
                        self.move_to_home()
                    elif key == 'm':
                        self.toggle_monitoring()
                    
                    # 🚶 걷기 제어
                    elif key in ['2', '3']:
                        version = int(key)
                        self.start_walking_version(version)
                    elif key == 't':
                        self.test_all_walking_versions()
                    elif key == 'x':
                        self.stop_walking()
                    elif key == 'z':
                        self.check_walking_status()
                    
                    # 🚗 바퀴 제어
                    elif key == 'k':  # 전진
                        self.move_wheels_direction('forward')
                    elif key == 'i':  # 백스페이스 (후진)
                        self.move_wheels_direction('backward')
                    elif key == 'a':  # 좌측
                        self.move_wheels_direction('left')
                    elif key == 'd':  # 우측
                        self.move_wheels_direction('right')
                    elif key == 'j':  # 좌회전
                        self.move_wheels_direction('turn_left')
                    elif key == 'l':  # 우회전
                        self.move_wheels_direction('turn_right')
                    elif key == ' ':  # 스페이스 - 바퀴 정지
                        self.stop_wheels()
                    elif key == ']':  # 바퀴 테스트
                        self.test_wheel_movements()
                    
                    # 🎮 통합 제어
                    elif key == 'f':  # 통합 전진
                        self.combined_forward_movement()
                    elif key == 'b':  # 통합 후진
                        self.combined_backward_movement()
                    elif key == 'l':  # 통합 좌측
                        self.combined_left_movement()
                    elif key == 'r':  # 통합 우측
                        self.combined_right_movement()
                    
                    # 🦾 기본 제어
                    elif key in motor_angle_keys:
                        angle = motor_angle_keys[key]
                        self.move_all_motors(angle)
                    elif key == 'r':
                        self.test_range_safely()
                    
                    # 📷 카메라
                    elif key == 'v':
                        self.show_camera_image()
                    elif key == 'c':
                        self.show_camera_stats()
                    
                    elif key.lower() == 'help':
                        self.show_help()
                    else:
                        print(f"🔤 '{key}' - 알 수 없는 명령어 (help: 도움말)")
                
                except Exception as e:
                    print(f"❌ 키 처리 오류: {e}")
                    continue
        
        except KeyboardInterrupt:
            print("\n⚠️ 키보드 인터럽트")
        except Exception as e:
            print(f"❌ 데모 실행 오류: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.is_running = False
        
        return True
    
    def run_text_commands(self):
        """텍스트 명령어 모드"""
        if not self.initialize_system():
            print("❌ 시스템 초기화 실패")
            return False
        
        print("\n🎮 완전한 시스템 텍스트 명령어 모드")
        print("명령어:")
        print("🚶 걷기: v2, v3, walk_test, walk_stop")
        print("🚗 바퀴: forward, backward, left, right, turn_left, turn_right, wheel_stop, wheel_test")
        print("🎮 통합: combo_forward, combo_backward, combo_left, combo_right")
        print("🔧 기타: status, home, range, camera, help, quit")
        
        try:
            while True:
                try:
                    cmd = input("\n명령어 입력: ").strip().lower()
                    
                    if cmd in ['quit', 'q', 'exit']:
                        break
                    elif cmd in ['status', 's']:
                        self.show_system_status()
                    elif cmd == 'home':
                        self.move_to_home()
                    
                    # 걷기 명령어
                    elif cmd == 'v2':
                        self.start_walking_version(2)
                    elif cmd == 'v3':
                        self.start_walking_version(3)
                    elif cmd == 'walk_test':
                        self.test_all_walking_versions()
                    elif cmd == 'walk_stop':
                        self.stop_walking()
                    
                    # 바퀴 명령어
                    elif cmd == 'forward':
                        self.move_wheels_direction('forward')
                    elif cmd == 'backward':
                        self.move_wheels_direction('backward')
                    elif cmd == 'left':
                        self.move_wheels_direction('left')
                    elif cmd == 'right':
                        self.move_wheels_direction('right')
                    elif cmd == 'turn_left':
                        self.move_wheels_direction('turn_left')
                    elif cmd == 'turn_right':
                        self.move_wheels_direction('turn_right')
                    elif cmd == 'wheel_stop':
                        self.stop_wheels()
                    elif cmd == 'wheel_test':
                        self.test_wheel_movements()
                    
                    # 통합 명령어
                    elif cmd == 'combo_forward':
                        self.combined_forward_movement()
                    elif cmd == 'combo_backward':
                        self.combined_backward_movement()
                    elif cmd == 'combo_left':
                        self.combined_left_movement()
                    elif cmd == 'combo_right':
                        self.combined_right_movement()
                    
                    # 기타
                    elif cmd == 'range':
                        self.test_range_safely()
                    elif cmd == 'camera':
                        self.show_camera_image()
                    elif cmd in ['help', 'h']:
                        self.show_help()
                    elif cmd.startswith('move '):
                        try:
                            angle = int(cmd.split()[1])
                            if -20 <= angle <= 63:
                                self.move_all_motors(angle)
                            else:
                                print("각도는 -20~63도 사이여야 합니다")
                        except (IndexError, ValueError):
                            print("사용법: move <각도> (예: move 30)")
                    else:
                        print("알 수 없는 명령어. 'help' 입력시 도움말")
                        print("🚶 걷기: v2, v3 | 🚗 바퀴: forward, left | 🎮 통합: combo_forward")
                
                except Exception as e:
                    print(f"❌ 명령어 처리 오류: {e}")
                    continue
        
        except KeyboardInterrupt:
            print("\n⚠️ 데모 중단")
        except Exception as e:
            print(f"❌ 텍스트 모드 오류: {e}")
            import traceback
            traceback.print_exc()
        
        return True
    
    def shutdown(self):
        """시스템 안전 종료"""
        print("🛑 완전한 시스템 종료 중...")
        
        try:
            self.is_running = False
            self.monitoring_mode = False
            
            # 걷기 중지
            if self.walking_controller and self.walking_controller.walking:
                print("🚶 걷기 중지 중...")
                try:
                    self.walking_controller.stop_walking()
                    time.sleep(1)
                except Exception as e:
                    print(f"⚠️ 걷기 중지 오류: {e}")
            
            # 바퀴 정지
            if self.wheel_controller and self.wheel_controller.is_enabled:
                print("🚗 바퀴 정지 중...")
                try:
                    self.wheel_controller.stop()
                    time.sleep(0.5)
                except Exception as e:
                    print(f"⚠️ 바퀴 정지 오류: {e}")
            
            # 홈 포지션으로 복귀
            if self.robot_controller.is_connected:
                print("🏠 홈 포지션으로 복귀...")
                try:
                    self.move_to_home()
                    time.sleep(0.5)
                    self.robot_controller.disconnect()
                except Exception as e:
                    print(f"⚠️ 홈 복귀 오류: {e}")
            
            # 데이터 수집 중지
            try:
                self.data_combiner.close()
            except Exception as e:
                print(f"⚠️ 데이터 수집 중지 오류: {e}")
            
            print("✅ 완전한 시스템 종료 완료")
        
        except Exception as e:
            print(f"❌ 시스템 종료 중 오류: {e}")


def main():
    """메인 함수"""
    print("🚀 완전한 로봇 시스템 시작")
    print("🎯 걷기 패턴 + 옴니휠 바퀴 제어")
    print("🤖 V2/V3 걷기 + 전진/후진/좌우/회전")
    print("🎮 통합 제어 모드 지원")
    
    demo = None
    
    try:
        demo = CompleteSystemDemo()
        
        print("\n🎮 실행 모드 선택:")
        print("1: 키보드 인터페이스 (실시간 - 추천)")
        print("2: 텍스트 명령어 모드")
        print("\n🎯 주요 기능:")
        print("  🚶 걷기: V2 연속 밀어내기, V3 4포인트 보행")
        print("  🚗 바퀴: 전진/후진/좌우/회전 (옴니휠)")
        print("  🎮 통합: 걷기+바퀴 동시 제어")
        
        try:
            choice = input("\n선택하세요 (1/2): ").strip()
        except KeyboardInterrupt:
            print("\n사용자가 취소했습니다.")
            return 0
        
        if choice == '1':
            print("\n🎮 키보드 모드:")
            print("  🚶 걷기: 2, 3 (각 버전)")
            print("  🚗 바퀴: W(전진), A(좌측), D(우측), 백스페이스(후진)")
            print("  🚗 회전: [(좌회전), E(우회전), 스페이스(정지)")
            print("  🎮 통합: F(전진), B(후진), L(좌측), R(우측)")
            print("  📊 기타: S(상태), H(홈), Help(도움말)")
            demo.run_demo()
        elif choice == '2':
            print("\n📝 텍스트 모드:")
            print("  🚶 걷기: v2, v3, walk_test")
            print("  🚗 바퀴: forward, left, turn_right, wheel_test")
            print("  🎮 통합: combo_forward, combo_left")
            print("  📊 기타: status, home, help")
            demo.run_text_commands()
        else:
            print("기본값으로 키보드 모드 실행")
            demo.run_demo()
    
    except KeyboardInterrupt:
        print("\n⚠️ 사용자가 중단했습니다")
    except Exception as e:
        print(f"❌ 메인 함수 실행 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if demo:
            try:
                demo.shutdown()
            except Exception as e:
                print(f"❌ 최종 종료 중 오류: {e}")
    
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
        print(f"\n🎯 완전한 시스템 데모 요약:")
        print(f"  ✅ 🚶 걷기: V2 연속 밀어내기, V3 4포인트 보행")
        print(f"  ✅ 🚗 바퀴: 전진/후진/좌우이동/좌우회전")
        print(f"  ✅ 🎮 통합: 걷기+바퀴 동시 제어")
        print(f"  ✅ 📷 카메라: 실시간 이미지 및 통계")
        print(f"  ✅ 🦾 모터: 각도 제어 및 범위 테스트")
        print(f"\n🏆 모든 기능이 통합된 완전한 로봇 시스템!")
        sys.exit(exit_code)
    except Exception as e:
        print(f"❌ 프로그램 실행 중 치명적 오류: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)