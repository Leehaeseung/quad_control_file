#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
바퀴 제어 데모
옴니휠을 이용한 홀로노믹 이동 테스트
키보드 인터페이스로 실시간 제어
"""

import os
import sys
import time
import threading
from config import *
from motor_sensor import MotorSensor
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


class WheelDemo:
    def __init__(self):
        """바퀴 데모 초기화"""
        print("🚗 바퀴 제어 데모 초기화 중...")
        
        self.motor_sensor = None
        self.wheel_controller = None
        self.is_running = False
        self.monitoring_mode = False
        
        print("✅ 바퀴 데모 초기화 완료")
    
    def initialize_system(self):
        """시스템 초기화"""
        print("🔧 바퀴 시스템 초기화 중...")
        
        try:
            self.motor_sensor = MotorSensor()
            
            if self.motor_sensor.connect():
                self.wheel_controller = WheelController(self.motor_sensor)
                
                if self.wheel_controller.is_enabled:
                    print("✅ 바퀴 시스템 초기화 성공")
                    return True
                else:
                    print("❌ 바퀴 컨트롤러 초기화 실패")
                    return False
            else:
                print("❌ 모터 연결 실패")
                return False
                
        except Exception as e:
            print(f"❌ 시스템 초기화 오류: {e}")
            return False
    
    def show_help(self):
        """도움말 표시"""
        print("\n" + "="*60)
        print("🚗 바퀴 제어 데모 (옴니휠 홀로노믹 이동)")
        print("="*60)
        print("🎮 기본 이동 (2초간):")
        print("  w: 전진")
        print("  s: 후진")
        print("  a: 좌측 이동")
        print("  d: 우측 이동")
        print("  q: 좌회전 (반시계)")
        print("  e: 우회전 (시계)")
        print()
        print("🎮 고속 이동 (대문자, 45 RPM):")
        print("  W: 고속 전진")
        print("  S: 고속 후진")
        print("  A: 고속 좌측")
        print("  D: 고속 우측")
        print("  Q: 고속 좌회전")
        print("  E: 고속 우회전")
        print()
        print("🛑 제어:")
        print("  스페이스: 바퀴 정지")
        print("  x: 비상 정지")
        print()
        print("📊 정보:")
        print("  i: 바퀴 상태 정보")
        print("  m: 모니터링 모드 토글")
        print("  t: 모든 이동 테스트")
        print()
        print("🔧 기타:")
        print("  h: 이 도움말")
        print("  ESC/z: 종료")
        print("="*60)
    
    def show_wheel_status(self):
        """바퀴 상태 출력"""
        if self.wheel_controller:
            self.wheel_controller.print_wheel_status()
        else:
            print("❌ 바퀴 컨트롤러가 초기화되지 않음")
    
    def test_all_movements(self):
        """모든 이동 패턴 테스트"""
        if self.wheel_controller:
            self.wheel_controller.test_all_movements()
        else:
            print("❌ 바퀴 컨트롤러가 초기화되지 않음")
    
    def toggle_monitoring(self):
        """모니터링 모드 토글"""
        self.monitoring_mode = not self.monitoring_mode
        if self.monitoring_mode:
            print("🔄 자동 모니터링 시작 (3초마다)")
        else:
            print("⏹️ 자동 모니터링 중지")
    
    def monitoring_loop(self):
        """모니터링 루프"""
        def monitor():
            while self.is_running:
                if self.monitoring_mode and self.wheel_controller:
                    print("\n" + "="*30 + " 자동 모니터링 " + "="*30)
                    self.wheel_controller.print_wheel_status()
                
                time.sleep(3)
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        return monitor_thread
    
    def run_keyboard_demo(self):
        """키보드 제어 데모"""
        if not self.initialize_system():
            return False
        
        self.show_help()
        self.is_running = True
        
        # 모니터링 스레드 시작
        monitor_thread = self.monitoring_loop()
        
        print("\n🎮 키보드 제어 시작! (도움말: h)")
        
        try:
            while self.is_running:
                key = getch()
                
                if key == chr(27) or key == 'z':  # ESC 또는 z
                    print("\n데모 종료...")
                    break
                
                elif key == ' ':  # 스페이스 - 정지
                    self.wheel_controller.stop()
                
                elif key == 'x':  # 비상 정지
                    self.wheel_controller.emergency_stop()
                
                # 기본 속도 이동 (소문자)
                elif key == 'w':  # 전진
                    self.wheel_controller.move_forward(WHEEL_DEFAULT_SPEED, 2.0)
                elif key == 's':  # 후진
                    self.wheel_controller.move_backward(WHEEL_DEFAULT_SPEED, 2.0)
                elif key == 'a':  # 좌측
                    self.wheel_controller.move_left(WHEEL_DEFAULT_SPEED, 2.0)
                elif key == 'd':  # 우측
                    self.wheel_controller.move_right(WHEEL_DEFAULT_SPEED, 2.0)
                elif key == 'q':  # 좌회전
                    self.wheel_controller.turn_left(WHEEL_DEFAULT_SPEED, 2.0)
                elif key == 'e':  # 우회전
                    self.wheel_controller.turn_right(WHEEL_DEFAULT_SPEED, 2.0)
                
                # 고속 이동 (대문자)
                elif key == 'W':  # 고속 전진
                    self.wheel_controller.move_forward(WHEEL_FAST_SPEED, 2.0)
                elif key == 'S':  # 고속 후진
                    self.wheel_controller.move_backward(WHEEL_FAST_SPEED, 2.0)
                elif key == 'A':  # 고속 좌측
                    self.wheel_controller.move_left(WHEEL_FAST_SPEED, 2.0)
                elif key == 'D':  # 고속 우측
                    self.wheel_controller.move_right(WHEEL_FAST_SPEED, 2.0)
                elif key == 'Q':  # 고속 좌회전
                    self.wheel_controller.turn_left(WHEEL_FAST_SPEED, 2.0)
                elif key == 'E':  # 고속 우회전
                    self.wheel_controller.turn_right(WHEEL_FAST_SPEED, 2.0)
                
                # 정보 및 제어
                elif key == 'i':  # 상태 정보
                    self.show_wheel_status()
                elif key == 'm':  # 모니터링 토글
                    self.toggle_monitoring()
                elif key == 't':  # 전체 테스트
                    self.test_all_movements()
                elif key == 'h':  # 도움말
                    self.show_help()
                
                else:
                    print(f"알 수 없는 키: '{key}' (h: 도움말)")
        
        except KeyboardInterrupt:
            print("\n⚠️ 키보드 인터럽트로 종료")
        
        finally:
            self.is_running = False
            if self.wheel_controller:
                self.wheel_controller.stop()
            if self.motor_sensor:
                self.motor_sensor.disconnect()
        
        return True
    
    def run_text_commands(self):
        """텍스트 명령어 모드"""
        if not self.initialize_system():
            return False
        
        print("\n🚗 바퀴 제어 텍스트 명령어 모드")
        print("명령어:")
        print("  forward, backward, left, right: 기본 이동")
        print("  turn_left, turn_right: 회전")
        print("  fast_<direction>: 고속 이동 (예: fast_forward)")
        print("  stop: 정지")
        print("  status: 상태 확인")
        print("  test: 전체 테스트")
        print("  custom: 사용자 정의 이동")
        print("  help: 도움말")
        print("  quit: 종료")
        
        try:
            while True:
                cmd = input("\n명령어 입력: ").strip().lower()
                
                if cmd in ['quit', 'q', 'exit']:
                    break
                
                elif cmd == 'forward':
                    self.wheel_controller.move_forward(WHEEL_DEFAULT_SPEED, 2.0)
                elif cmd == 'backward':
                    self.wheel_controller.move_backward(WHEEL_DEFAULT_SPEED, 2.0)
                elif cmd == 'left':
                    self.wheel_controller.move_left(WHEEL_DEFAULT_SPEED, 2.0)
                elif cmd == 'right':
                    self.wheel_controller.move_right(WHEEL_DEFAULT_SPEED, 2.0)
                elif cmd == 'turn_left':
                    self.wheel_controller.turn_left(WHEEL_DEFAULT_SPEED, 2.0)
                elif cmd == 'turn_right':
                    self.wheel_controller.turn_right(WHEEL_DEFAULT_SPEED, 2.0)
                
                elif cmd == 'fast_forward':
                    self.wheel_controller.move_forward(WHEEL_FAST_SPEED, 2.0)
                elif cmd == 'fast_backward':
                    self.wheel_controller.move_backward(WHEEL_FAST_SPEED, 2.0)
                elif cmd == 'fast_left':
                    self.wheel_controller.move_left(WHEEL_FAST_SPEED, 2.0)
                elif cmd == 'fast_right':
                    self.wheel_controller.move_right(WHEEL_FAST_SPEED, 2.0)
                elif cmd == 'fast_turn_left':
                    self.wheel_controller.turn_left(WHEEL_FAST_SPEED, 2.0)
                elif cmd == 'fast_turn_right':
                    self.wheel_controller.turn_right(WHEEL_FAST_SPEED, 2.0)
                
                elif cmd == 'stop':
                    self.wheel_controller.stop()
                elif cmd == 'status':
                    self.show_wheel_status()
                elif cmd == 'test':
                    self.test_all_movements()
                elif cmd == 'help':
                    self.show_help()
                
                elif cmd == 'custom':
                    try:
                        print("사용자 정의 이동 (vx vy omega):")
                        vx = float(input("전후 속도 (vx, -50~50): "))
                        vy = float(input("좌우 속도 (vy, -50~50): "))
                        omega = float(input("회전 속도 (omega, -50~50): "))
                        duration = float(input("지속 시간 (초, 0=무한): "))
                        
                        self.wheel_controller.move_custom(vx, vy, omega, duration)
                    except ValueError:
                        print("❌ 숫자를 입력해주세요")
                
                else:
                    print("알 수 없는 명령어. 'help' 입력시 도움말")
        
        except KeyboardInterrupt:
            print("\n⚠️ 데모 중단")
        
        finally:
            if self.wheel_controller:
                self.wheel_controller.stop()
            if self.motor_sensor:
                self.motor_sensor.disconnect()
        
        return True
    
    def shutdown(self):
        """시스템 안전 종료"""
        print("🛑 바퀴 데모 종료 중...")
        
        self.is_running = False
        
        if self.wheel_controller:
            self.wheel_controller.stop()
        
        if self.motor_sensor:
            self.motor_sensor.disconnect()
        
        print("✅ 바퀴 데모 종료 완료")


def main():
    """메인 함수"""
    print("🚗 바퀴 제어 데모 시작")
    print("🎯 옴니휠 홀로노믹 이동 테스트")
    print("📐 전진/후진/좌우/회전 지원")
    
    demo = None
    
    try:
        demo = WheelDemo()
        
        print("\n🎮 실행 모드 선택:")
        print("1: 키보드 인터페이스 (실시간 제어)")
        print("2: 텍스트 명령어 모드")
        
        try:
            choice = input("\n선택하세요 (1/2): ").strip()
        except KeyboardInterrupt:
            print("\n사용자가 취소했습니다.")
            return 0
        
        if choice == '1':
            print("\n🎮 키보드 모드:")
            print("  WASD: 기본 이동")
            print("  QE: 회전")
            print("  대문자: 고속 이동")
            print("  스페이스: 정지")
            demo.run_keyboard_demo()
        elif choice == '2':
            print("\n📝 텍스트 모드:")
            print("  forward, left, turn_right 등")
            print("  fast_forward, custom 등")
            demo.run_text_commands()
        else:
            print("기본값으로 키보드 모드 실행")
            demo.run_keyboard_demo()
    
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
        print(f"\n🎯 바퀴 제어 데모 요약:")
        print(f"  ✅ 전진/후진: W/S 키")
        print(f"  ✅ 좌우 이동: A/D 키") 
        print(f"  ✅ 회전: Q/E 키")
        print(f"  ✅ 옴니휠 홀로노믹 이동 완료!")
        sys.exit(exit_code)
    except Exception as e:
        print(f"❌ 프로그램 실행 중 치명적 오류: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)