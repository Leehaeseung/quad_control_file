#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센서 테스트 모듈
키보드 인터페이스로 센서 상태 확인 및 테스트
"""

import os
import sys
import time
import cv2
from config import *
from data_combiner import DataCombiner

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


class SensorTest:
    def __init__(self):
        """센서 테스트 시스템 초기화"""
        print("🧪 센서 테스트 시스템 초기화...")
        
        self.combiner = DataCombiner()
        self.is_monitoring = False
        self.running = True
        
        print("✅ 센서 테스트 시스템 준비 완료")
    
    def start_system(self):
        """테스트 시스템 시작"""
        print("🚀 센서 테스트 시스템 시작...")
        
        if self.combiner.start_data_collection():
            print("✅ 시스템 시작 완료!")
            return True
        else:
            print("❌ 시스템 시작 실패")
            return False
    
    def show_help(self):
        """도움말 출력"""
        print("\n" + "="*60)
        print("🎮 센서 테스트 시스템 명령어")
        print("="*60)
        print("📊 센서 상태:")
        print("  s: 시스템 상태 출력")
        print("  v: Isaac Lab 값 출력")
        print("  m: 모니터링 모드 토글 (1초마다 자동 출력)")
        print()
        print("📷 카메라:")
        print("  i: 이미지 표시 (3초간)")
        print("  c: 카메라 통계 출력")
        print()
        print("🦾 모터:")
        print("  0-9: 모든 모터를 해당 각도로 이동 (0°, 3°, 6°, ..., 27°)")
        print("  j: 조인트 상태 출력")
        print()
        print("💾 데이터:")
        print("  save: 현재 데이터 저장")
        print("  log: 센서 로그 저장")
        print()
        print("🔧 시스템:")
        print("  h: 이 도움말 표시")
        print("  q/ESC: 종료")
        print("="*60)
    
    def show_system_status(self):
        """시스템 상태 출력"""
        # 최신 데이터로 업데이트
        self.combiner.update_data()
        
        # 상태 출력
        self.combiner.print_system_status()
    
    def show_isaac_values(self):
        """Isaac Lab 값 출력"""
        self.combiner.update_data()
        values_str = self.combiner.get_isaac_values_string()
        print(f"\n📝 Isaac Lab 형식 값:")
        print(f"   {values_str}")
        print()
    
    def show_camera_image(self):
        """카메라 이미지 표시"""
        if not self.combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        print("📷 카메라 이미지 표시 중... (3초간, ESC로 종료)")
        
        start_time = time.time()
        while time.time() - start_time < 3.0:
            # 최신 데이터 업데이트
            self.combiner.update_data()
            
            # 결합 이미지 가져오기
            combined = self.combiner.camera.get_combined_image()
            if combined is not None:
                cv2.imshow('Sensor Test - Camera', combined)
                
                # ESC 키 체크
                key = cv2.waitKey(30) & 0xFF
                if key == 27:  # ESC
                    break
            else:
                print("❌ 이미지를 가져올 수 없음")
                break
        
        cv2.destroyAllWindows()
        print("📷 이미지 표시 종료")
    
    def show_camera_stats(self):
        """카메라 통계 출력"""
        if not self.combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        self.combiner.update_data()
        stats = self.combiner.camera.get_depth_statistics()
        
        print(f"\n📊 카메라 통계:")
        print(f"  해상도: {CAMERA_WIDTH}×{CAMERA_HEIGHT}")
        print(f"  최소 깊이: {stats['min_depth']:.3f}m")
        print(f"  최대 깊이: {stats['max_depth']:.3f}m")
        print(f"  평균 깊이: {stats['mean_depth']:.3f}m")
        print(f"  유효 픽셀: {stats['valid_pixels']}/{stats['total_pixels']} ({stats['valid_pixels']/stats['total_pixels']*100:.1f}%)")
        print()
    
    def show_joint_status(self):
        """조인트 상태 출력"""
        if not self.combiner.motor:
            print("❌ 모터가 연결되지 않음")
            return
        
        self.combiner.update_data()
        motor_data = self.combiner.motor.get_isaac_lab_data()
        
        print(f"\n🦾 조인트 상태 (Isaac Lab 순서):")
        print(f"{'조인트':<10} {'모터ID':<8} {'위치(rad)':<12} {'위치(deg)':<12} {'속도(rad/s)':<12}")
        print("-" * 60)
        
        for i in range(8):
            joint_name = JOINT_NAMES[i]
            motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[i]
            pos_rad = motor_data['joint_positions'][i]
            pos_deg = pos_rad * RAD_TO_DEG
            vel_rad = motor_data['joint_velocities'][i]
            
            print(f"{joint_name:<10} {motor_id:<8} {pos_rad:<12.4f} {pos_deg:<12.1f} {vel_rad:<12.4f}")
        print()
    
    def move_all_motors(self, angle_deg):
        """모든 모터를 같은 각도로 이동"""
        if not self.combiner.motor:
            print("❌ 모터가 연결되지 않음")
            return
        
        print(f"🎯 모든 모터를 {angle_deg}°로 이동 중...")
        
        angles = [angle_deg] * 8
        if self.combiner.motor.move_all_joints(angles):
            print(f"✅ 모터 이동 명령 전송 완료")
            
            # 1초 후 상태 확인
            time.sleep(1)
            self.show_joint_status()
        else:
            print("❌ 모터 이동 실패")
    
    def save_current_data(self):
        """현재 데이터 저장"""
        print("💾 현재 데이터 저장 중...")
        
        self.combiner.update_data()
        
        if self.combiner.save_combined_obs():
            print("✅ Isaac Lab 텐서 저장 완료")
        else:
            print("❌ 텐서 저장 실패")
    
    def save_sensor_log(self):
        """센서 로그 저장"""
        print("📝 센서 로그 저장 중...")
        
        self.combiner.update_data()
        
        if self.combiner.save_sensor_log():
            print("✅ 센서 로그 저장 완료")
        else:
            print("❌ 센서 로그 저장 실패")
    
    def toggle_monitoring(self):
        """모니터링 모드 토글"""
        self.is_monitoring = not self.is_monitoring
        
        if self.is_monitoring:
            print("🔄 모니터링 모드 시작 (1초마다 자동 출력)")
        else:
            print("⏹️ 모니터링 모드 중지")
    
    def monitoring_loop(self):
        """모니터링 루프 (별도 스레드에서 실행)"""
        import threading
        
        def monitor():
            while self.running:
                if self.is_monitoring:
                    print("\n" + "="*30 + " 자동 모니터링 " + "="*30)
                    self.show_isaac_values()
                    
                time.sleep(1)
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        return monitor_thread
    
    def run_interactive_test(self):
        """대화형 테스트 실행"""
        if not self.start_system():
            return False
        
        self.show_help()
        
        # 모니터링 스레드 시작
        monitor_thread = self.monitoring_loop()
        
        # 키 매핑
        motor_angle_keys = {str(i): i * 3 for i in range(10)}  # 0°, 3°, ..., 27°
        
        print("\n🎮 대화형 테스트 시작! 명령어를 입력하세요:")
        
        try:
            while self.running:
                key = getch()
                
                if key == chr(27) or key == 'q':  # ESC 또는 q
                    print("\n종료합니다...")
                    break
                
                elif key == 's':
                    self.show_system_status()
                
                elif key == 'v':
                    self.show_isaac_values()
                
                elif key == 'm':
                    self.toggle_monitoring()
                
                elif key == 'i':
                    self.show_camera_image()
                
                elif key == 'c':
                    self.show_camera_stats()
                
                elif key == 'j':
                    self.show_joint_status()
                
                elif key in motor_angle_keys:
                    angle = motor_angle_keys[key]
                    self.move_all_motors(angle)
                
                elif key == 'h':
                    self.show_help()
                
                else:
                    # 다중 문자 명령어 처리
                    if key in ['s', 'l']:  # save, log의 첫 글자
                        print(f"'{key}' 입력됨. 전체 명령어:")
                        print("  'save' 입력 후 Enter: 데이터 저장")
                        print("  'log' 입력 후 Enter: 로그 저장")
                        
                        # 간단히 처리
                        if key == 's':
                            print("데이터 저장을 위해 's' + 'a' + 'v' + 'e'를 연속 입력하세요")
                        
                    else:
                        print(f"알 수 없는 키: '{key}' (h: 도움말)")
        
        except KeyboardInterrupt:
            print("\n⚠️ 키보드 인터럽트로 종료")
        
        finally:
            self.running = False
            self.combiner.close()
    
    def run_simple_commands(self):
        """간단한 명령어 모드"""
        if not self.start_system():
            return False
        
        print("\n🎮 간단 명령어 모드 (Enter로 입력)")
        print("명령어: status, values, image, joints, save, log, help, quit")
        
        try:
            while True:
                cmd = input("\n명령어 입력: ").strip().lower()
                
                if cmd in ['quit', 'q', 'exit']:
                    break
                elif cmd in ['status', 's']:
                    self.show_system_status()
                elif cmd in ['values', 'v']:
                    self.show_isaac_values()
                elif cmd in ['image', 'i']:
                    self.show_camera_image()
                elif cmd in ['joints', 'j']:
                    self.show_joint_status()
                elif cmd == 'save':
                    self.save_current_data()
                elif cmd == 'log':
                    self.save_sensor_log()
                elif cmd in ['help', 'h']:
                    self.show_help()
                elif cmd.isdigit():
                    angle = int(cmd) * 3
                    if 0 <= angle <= 27:
                        self.move_all_motors(angle)
                    else:
                        print("각도는 0~9 사이 숫자로 입력하세요")
                else:
                    print("알 수 없는 명령어. 'help' 입력시 도움말")
        
        except KeyboardInterrupt:
            print("\n⚠️ 테스트 중단")
        
        finally:
            self.combiner.close()


def main():
    """메인 함수"""
    print("🧪 센서 테스트 시스템")
    print("1: 키보드 인터페이스 (실시간)")
    print("2: 간단 명령어 모드")
    
    try:
        choice = input("선택하세요 (1/2): ").strip()
        
        tester = SensorTest()
        
        if choice == '1':
            tester.run_interactive_test()
        elif choice == '2':
            tester.run_simple_commands()
        else:
            print("기본값으로 간단 명령어 모드 실행")
            tester.run_simple_commands()
            
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()