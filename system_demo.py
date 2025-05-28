#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
시스템 데모 - 자연스러운 걷기 기능 포함
AI 모델 없이 모든 센서와 모터를 테스트하는 통합 데모
새로운 70도 무릎 데이터로 자연스러운 바닥 밀어내기 보행 구현
"""

import os
import sys
import time
import cv2
import math
import threading
import numpy as np
from config import *
from data_combiner import DataCombiner
from robot_controller import RobotController

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

# 왼쪽 조인트 정의
LEFT_JOINTS = [0, 1, 2, 3]  # lb_leg, lb_knee, lf_leg, lf_knee


class NaturalWalkingController:
    """자연스러운 걷기 컨트롤러 - 바닥 밀어내기 최적화"""
    
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.walking = False
        self.walk_thread = None
        
        # 확장된 측정 데이터 (KNEE 70도까지, LEG 49도 유지)
        self.motor_data = [
            {"motor": 0, "knee": 14.5, "leg": 139},
            {"motor": 7, "knee": 17, "leg": 145},
            {"motor": 14, "knee": 21.8, "leg": 152},
            {"motor": 21, "knee": 25, "leg": 159},
            {"motor": 28, "knee": 33, "leg": 165},
            {"motor": 35, "knee": 35, "leg": 172},
            {"motor": 42, "knee": 39.1, "leg": 180},
            {"motor": 49, "knee": 42, "leg": 187},
            # KNEE만 확장된 데이터 (LEG는 187도 고정)
            {"motor": 56, "knee": 48, "leg": 187},
            {"motor": 63, "knee": 54, "leg": 187},
            {"motor": 70, "knee": 60, "leg": 187}
        ]
        
        print("🚶 자연스러운 걷기 컨트롤러 초기화 완료")
        print("  📐 LEG: 0°~49° (자연스러운 움직임)")
        print("  📐 KNEE: 0°~70° (확장된 범위)")
        print("  🌍 바닥 밀어내기 최적화")
    
    def interpolate_motor_data(self, motor_angle, is_knee=False):
        """모터 각도 보간 (KNEE만 70도까지 확장)"""
        max_angle = 70 if is_knee else 49
        motor_angle = max(0, min(max_angle, motor_angle))
        
        for i in range(len(self.motor_data) - 1):
            if motor_angle <= self.motor_data[i + 1]["motor"]:
                data1 = self.motor_data[i]
                data2 = self.motor_data[i + 1]
                
                if data1["motor"] == data2["motor"]:
                    return data1
                
                t = (motor_angle - data1["motor"]) / (data2["motor"] - data1["motor"])
                return {
                    "leg": data1["leg"] + (data2["leg"] - data1["leg"]) * t,
                    "knee": data1["knee"] + (data2["knee"] - data1["knee"]) * t
                }
        
        return self.motor_data[-1]
    
    def get_efficient_walking_angles(self, phase):
        """효율적인 4단계 걷기 패턴"""
        # 걷기 4단계별 최적화
        if phase < 0.25:
            # 1단계: 들어올리기 - 무릎 접고 다리 세우기
            t = phase / 0.25
            leg_angle = 24.5 + 15 * math.sin(t * math.pi)  # 다리 세우기
            knee_angle = 35 + 25 * math.sin(t * math.pi)   # 무릎 접기
            
        elif phase < 0.5:
            # 2단계: 앞으로 내딛기 - 무릎 접은 상태 유지
            t = (phase - 0.25) / 0.25
            leg_angle = 24.5 + 15 * (1 - t)              # 다리 점진적으로 내리기
            knee_angle = 35 + 25 * (1 - 0.3 * t)         # 무릎 약간 펴기 시작
            
        elif phase < 0.6:
            # 3단계: 바닥 접촉 - 무릎 최대한 접고 다리 세우기
            t = (phase - 0.5) / 0.1
            leg_angle = 24.5 + 10 * (1 - t)              # 다리 완전히 세우기
            knee_angle = 35 + 30 * (1 - t)               # 무릎 최대한 접기
            
        else:
            # 4단계: 뒤로 밀어내기 - 다리 올리고 무릎 펴기
            t = (phase - 0.6) / 0.4
            leg_angle = 24.5 - 20 * t                    # 다리 뒤쪽으로 올리기
            knee_angle = 35 - 15 * t                     # 무릎 펴서 뒤로 밀기
        
        return {
            "leg_motor": max(0, min(49, leg_angle)),
            "knee_motor": max(0, min(70, knee_angle))
        }
    
    def get_diagonal_gait_angles(self, phase):
        """대각선 보행 패턴 (자연스러운 연속 보행)"""
        # 세트1: 오른앞(6,7) + 왼뒤(0,1) - 연속 보행
        set1_phase = phase
        # 세트2: 왼앞(2,3) + 오른뒤(4,5) - 반사이클 차이
        set2_phase = (phase + 0.5) % 1.0
        
        # 각 세트의 효율적인 각도 계산
        set1_angles = self.get_efficient_walking_angles(set1_phase)
        set2_angles = self.get_efficient_walking_angles(set2_phase)
        
        # 8개 조인트 각도 배열 (Isaac Lab 순서)
        joint_angles = [0] * 8
        
        # 세트1: 왼뒤(0,1) + 오른앞(6,7)
        joint_angles[0] = set1_angles["leg_motor"]   # lb_leg
        joint_angles[1] = set1_angles["knee_motor"]  # lb_knee
        joint_angles[6] = set1_angles["leg_motor"]   # rf_leg
        joint_angles[7] = set1_angles["knee_motor"]  # rf_knee
        
        # 세트2: 왼앞(2,3) + 오른뒤(4,5)
        joint_angles[2] = set2_angles["leg_motor"]   # lf_leg
        joint_angles[3] = set2_angles["knee_motor"]  # lf_knee
        joint_angles[4] = set2_angles["leg_motor"]   # rb_leg
        joint_angles[5] = set2_angles["knee_motor"]  # rb_knee
        
        return joint_angles
    
    def start_natural_walking(self, duration=10.0, speed=1.0):
        """자연스러운 걷기 시작"""
        if self.walking:
            print("⚠️ 이미 걷기 중입니다")
            return
        
        print(f"🚶 효율적인 4단계 걷기 시작!")
        print(f"  1단계: 무릎 접기 + 다리 세우기")
        print(f"  2단계: 앞으로 내딛기 (무릎 접은 상태)")
        print(f"  3단계: 바닥 접촉 (무릎 최대 접기)")
        print(f"  4단계: 다리 올리기 + 무릎 펴서 밀기")
        
        self.walking = True
        
        def natural_walk_loop():
            start_time = time.time()
            cycle_count = 0
            
            try:
                while self.walking and (time.time() - start_time) < duration:
                    elapsed = time.time() - start_time
                    phase = (elapsed * speed * 0.4) % 1.0  # 0.4Hz 자연스러운 속도
                    
                    # 효율적인 대각선 보행 각도 계산
                    joint_angles = self.get_diagonal_gait_angles(phase)
                    
                    # 모터 명령 실행 (motor_sensor에서 자동 방향 반전)
                    if not self.robot.motor_sensor.move_all_joints(joint_angles):
                        print("⚠️ 걷기 중 모터 오류")
                        break
                    
                    # 상태 출력 (3초마다)
                    new_cycle = int(elapsed * speed * 0.4)
                    if new_cycle > cycle_count:
                        cycle_count = new_cycle
                        
                        # 각 세트의 상태 계산
                        set1_phase = phase
                        set2_phase = (phase + 0.5) % 1.0
                        
                        def get_phase_status(p):
                            if p < 0.25:
                                return "🦵 들어올리기 (무릎접기+다리세우기)"
                            elif p < 0.5:
                                return "👣 앞으로 내딛기 (무릎접은상태)"
                            elif p < 0.6:
                                return "⬇️ 바닥접촉 (무릎최대접기)"
                            else:
                                return "💪 뒤로밀기 (다리올리기+무릎펴기)"
                        
                        set1_status = get_phase_status(set1_phase)
                        set2_status = get_phase_status(set2_phase)
                        
                        print(f"🔄 사이클 {cycle_count}: 세트1({set1_status}) vs 세트2({set2_status})")
                        print(f"   위상: {set1_phase:.2f} vs {set2_phase:.2f}")
                    
                    # 30Hz 제어 (부드러운 움직임)
                    time.sleep(1.0 / 30.0)
            
            except Exception as e:
                print(f"❌ 자연스러운 걷기 중 오류: {e}")
                import traceback
                traceback.print_exc()
            
            finally:
                self.walking = False
                print("🛑 자연스러운 걷기 종료")
        
        self.walk_thread = threading.Thread(target=natural_walk_loop, daemon=True)
        self.walk_thread.start()
    
    def stop_walking(self):
        """걷기 중지"""
        if self.walking:
            self.walking = False
            print("⏹️ 자연스러운 걷기 중지됨")
        else:
            print("⚠️ 걷기 중이 아닙니다")
    
    def test_knee_range(self):
        """무릎 범위 테스트 (70도까지)"""
        if not self.robot.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        print("🦵 무릎 범위 테스트 시작 (0°→70°)")
        
        test_angles = [0, 20, 40, 60, 70, 60, 40, 20, 0]
        
        for i, knee_angle in enumerate(test_angles):
            print(f"스텝 {i+1}/{len(test_angles)}: 무릎 {knee_angle}°")
            
            # 모든 무릎을 동일한 각도로 (다리는 기본 각도)
            joint_angles = [24.5] * 8  # 다리 기본 각도
            joint_angles[1] = knee_angle  # lb_knee
            joint_angles[3] = knee_angle  # lf_knee  
            joint_angles[5] = knee_angle  # rb_knee
            joint_angles[7] = knee_angle  # rf_knee
            
            if self.robot.motor_sensor.move_all_joints(joint_angles):
                print(f"✅ 무릎 {knee_angle}° 이동 성공")
            else:
                print(f"❌ 무릎 {knee_angle}° 이동 실패")
            
            time.sleep(1.5)
        
        print("✅ 무릎 범위 테스트 완료!")


class SystemDemo:
    def __init__(self):
        """시스템 데모 초기화"""
        print("🎮 시스템 데모 초기화 중...")
        
        # 모듈형 구조 사용
        self.data_combiner = DataCombiner()
        self.robot_controller = RobotController()
        
        # 자연스러운 걷기 컨트롤러 추가
        self.walking_controller = None
        
        # 데모 상태
        self.is_running = False
        self.monitoring_mode = False
        
        print("✅ 시스템 데모 초기화 완료")
    
    def initialize_system(self):
        """시스템 초기화"""
        print("🔧 시스템 초기화 중...")
        
        success_count = 0
        
        # 센서 시스템 초기화
        if self.data_combiner.start_data_collection():
            print("✅ 센서 시스템 초기화 성공")
            success_count += 1
        else:
            print("❌ 센서 시스템 초기화 실패")
        
        # 로봇 제어기 초기화
        if self.robot_controller.connect_motors():
            if self.robot_controller.enable_control():
                print("✅ 로봇 제어기 초기화 성공")
                success_count += 1
                
                # 자연스러운 걷기 컨트롤러 초기화
                self.walking_controller = NaturalWalkingController(self.robot_controller)
                print("✅ 자연스러운 걷기 컨트롤러 초기화 성공")
            else:
                print("⚠️ 로봇 제어 활성화 실패")
        else:
            print("❌ 로봇 제어기 초기화 실패")
        
        print(f"\n📊 초기화 결과: {success_count}/2 성공")
        return success_count > 0
    
    def show_help(self):
        """도움말 표시"""
        print("\n" + "="*70)
        print("🎮 시스템 데모 명령어 (자연스러운 걷기 + 자동 좌우 대칭)")
        print("="*70)
        print("📊 시스템:")
        print("  s: 전체 시스템 상태 출력")
        print("  i: Isaac Lab 텐서 정보 출력")
        print("  m: 모니터링 모드 토글 (1초마다 자동 출력)")
        print()
        print("🦾 로봇 제어 (자동 좌우 대칭):")
        print("  0-9: 모든 모터를 해당 각도로 이동")
        print("       왼쪽: 0°, 7°, 14°, 21°, 28°, 35°, 42°, 49°, 56°, 63° (반시계)")
        print("       오른쪽: 0°, -7°, -14°, -21°, -28°, -35°, -42°, -49°, -56°, -63° (시계)")
        print("  h: 홈 포지션으로 이동 (0도)")
        print("  t: 테스트 패턴 실행 (자동 좌우 대칭)")
        print()
        print("🚶 자연스러운 걷기 제어 (NEW!):")
        print("  w: 자연스러운 걷기 시작 (10초간) - 바닥 밀어내기 최적화")
        print("  W: 걷기 중지")
        print("  x: 걷기 테스트 (5초간)")
        print("  k: 무릎 범위 테스트 (0°~70°)")
        print("  z: 걷기 상태 확인")
        print()
        print("📷 카메라:")
        print("  v: 카메라 이미지 표시 (3초간)")
        print("  c: 카메라 통계 출력")
        print()
        print("💾 데이터:")
        print("  save: Isaac Lab 텐서 저장")
        print("  log: 센서 로그 저장")
        print()
        print("🔧 기타:")
        print("  help: 이 도움말 표시")
        print("  ESC/q: 종료")
        print("="*70)
    
    def show_system_status(self):
        """전체 시스템 상태 출력"""
        print("\n🔍 전체 시스템 상태:")
        print("="*60)
        
        # 데이터 수집기 상태
        self.data_combiner.update_data()
        self.data_combiner.print_system_status()
        
        # 로봇 제어기 상태
        if self.robot_controller.is_connected:
            control_stats = self.robot_controller.get_control_stats()
            print(f"\n🦾 로봇 제어기 (자동 좌우 대칭):")
            print(f"  연결됨: ✅")
            print(f"  제어 활성화: {'✅' if control_stats['is_control_enabled'] else '❌'}")
            print(f"  총 명령 수: {control_stats['total_commands']}")
            print(f"  성공률: {control_stats['success_rate_percent']:.1f}%")
            print(f"  좌우 처리: motor_sensor.py에서 자동 처리")
        else:
            print(f"\n🦾 로봇 제어기: ❌ 연결 안됨")
        
        # 자연스러운 걷기 컨트롤러 상태
        if self.walking_controller:
            print(f"\n🚶 자연스러운 걷기 컨트롤러:")
            print(f"  초기화됨: ✅")
            print(f"  걷기 중: {'✅' if self.walking_controller.walking else '❌'}")
            print(f"  패턴: 대각선 연속 보행 (반사이클 차이)")
            print(f"  특징: 바닥 밀어내기 최적화")
            print(f"  LEG 범위: 0°~49° (자연스러운 움직임)")
            print(f"  KNEE 범위: 0°~70° (확장된 범위)")
        else:
            print(f"\n🚶 자연스러운 걷기 컨트롤러: ❌ 초기화 안됨")
        
        print("="*60)
    
    def show_isaac_tensor_info(self):
        """Isaac Lab 텐서 정보 출력"""
        self.data_combiner.update_data()
        tensor = self.data_combiner.get_latest_isaac_tensor()
        
        if tensor is not None:
            info = self.data_combiner.converter.get_tensor_info(tensor)
            values_str = self.data_combiner.get_isaac_values_string()
            
            print(f"\n📊 Isaac Lab 텐서 정보:")
            print(f"  형태: {info['shape']}")
            print(f"  데이터 타입: {info['dtype']}")
            print(f"  깊이 채널 범위: {info['depth_range'][0]:.3f}~{info['depth_range'][1]:.3f}m")
            print(f"  베이스 채널 범위: {info['base_range'][0]:.3f}~{info['base_range'][1]:.3f}")
            print(f"  조인트 채널 범위: {info['joint_range'][0]:.3f}~{info['joint_range'][1]:.3f}")
            print(f"\n📝 Isaac Lab 형식 값:")
            print(f"  {values_str}")
        else:
            print("❌ Isaac Lab 텐서 데이터 없음")
    
    def show_camera_image(self):
        """카메라 이미지 표시"""
        if not self.data_combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        print("📷 카메라 이미지 표시 중... (3초간, ESC로 종료)")
        
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.data_combiner.update_data()
            
            # 결합 이미지 가져오기
            combined = self.data_combiner.camera.get_combined_image()
            if combined is not None:
                cv2.imshow('System Demo - Camera', combined)
                
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
        if not self.data_combiner.camera:
            print("❌ 카메라가 연결되지 않음")
            return
        
        self.data_combiner.update_data()
        stats = self.data_combiner.camera.get_depth_statistics()
        
        print(f"\n📊 카메라 통계:")
        print(f"  해상도: {CAMERA_WIDTH}×{CAMERA_HEIGHT}")
        print(f"  FPS: {CAMERA_FPS}")
        print(f"  최소 깊이: {stats['min_depth']:.3f}m")
        print(f"  최대 깊이: {stats['max_depth']:.3f}m")
        print(f"  평균 깊이: {stats['mean_depth']:.3f}m")
        print(f"  유효 픽셀: {stats['valid_pixels']}/{stats['total_pixels']} ({stats['valid_pixels']/stats['total_pixels']*100:.1f}%)")
    
    def move_all_motors(self, target_angle_deg):
        """모든 모터를 각도로 이동 (좌우 대칭 자동 처리)"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        print(f"🎯 모든 모터를 {target_angle_deg}°로 이동 중...")
        print(f"   왼쪽 다리: +{target_angle_deg}° (반시계)")
        print(f"   오른쪽 다리: +{target_angle_deg}° → -{target_angle_deg}° (시계, 자동 처리)")
        
        # 모든 조인트에 동일한 각도 전달 (motor_sensor.py에서 자동 처리)
        angles = [target_angle_deg] * 8
        
        # 모터에 직접 명령 전송
        if self.robot_controller.motor_sensor.move_all_joints(angles):
            print(f"✅ 자동 좌우 대칭 모터 이동 완료")
            time.sleep(1)
            self.show_joint_status()
        else:
            print("❌ 자동 좌우 대칭 모터 이동 실패")
    
    def move_to_home(self):
        """홈 포지션으로 이동"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        print("🏠 홈 포지션으로 이동 중...")
        
        # 모든 모터를 0도로
        home_angles = [0.0] * 8
        
        if self.robot_controller.motor_sensor.move_all_joints(home_angles):
            print("✅ 홈 포지션 이동 완료")
            time.sleep(1)
            self.show_joint_status()
        else:
            print("❌ 홈 포지션 이동 실패")
    
    def run_test_pattern(self):
        """테스트 패턴 실행 (자동 좌우 대칭)"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        print("🎯 자동 좌우 대칭 테스트 패턴 실행 중...")
        
        # 테스트 각도 패턴
        test_angles = [0, 10, 20, 30, 20, 10, 0]
        
        for i, angle in enumerate(test_angles):
            print(f"스텝 {i+1}/{len(test_angles)}: {angle}°")
            print(f"  왼쪽: +{angle}° (반시계), 오른쪽: +{angle}° → -{angle}° (시계)")
            self.move_all_motors(angle)
            time.sleep(1.5)
        
        print("✅ 자동 좌우 대칭 테스트 패턴 완료!")
    
    def start_natural_walking(self, duration=10.0):
        """자연스러운 걷기 시작"""
        if not self.walking_controller:
            print("❌ 자연스러운 걷기 컨트롤러가 초기화되지 않음")
            return
        
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        print("🚶 자연스러운 바닥 밀어내기 걷기 시작!")
        print("  🌍 특징: 바닥 접촉 시간 최적화")
        print("  📐 LEG: 0°~49° (자연스러운 움직임)")
        print("  📐 KNEE: 0°~70° (확장된 범위)")
        print("  🔄 패턴: 대각선 연속 보행")
        
        self.walking_controller.start_natural_walking(duration)
    
    def test_natural_walking(self, duration=5.0):
        """자연스러운 걷기 테스트 (짧은 시간)"""
        if not self.walking_controller:
            print("❌ 자연스러운 걷기 컨트롤러가 초기화되지 않음")
            return
        
        print(f"🧪 자연스러운 걷기 테스트 ({duration}초간)")
        self.walking_controller.start_natural_walking(duration, speed=0.8)
    
    def test_knee_range(self):
        """무릎 범위 테스트"""
        if not self.walking_controller:
            print("❌ 자연스러운 걷기 컨트롤러가 초기화되지 않음")
            return
        
        self.walking_controller.test_knee_range()
    
    def stop_walking(self):
        """걷기 중지"""
        if self.walking_controller:
            self.walking_controller.stop_walking()
        else:
            print("❌ 자연스러운 걷기 컨트롤러가 초기화되지 않음")
    
    def check_walking_status(self):
        """걷기 상태 확인"""
        if self.walking_controller:
            status = "걷기 중" if self.walking_controller.walking else "정지"
            print(f"🚶 자연스러운 걷기 상태: {status}")
            if self.walking_controller.walking:
                print("  🌍 패턴: 바닥 밀어내기 최적화")
                print("  🔄 방식: 대각선 연속 보행")
                print("  📐 범위: LEG(49°) + KNEE(70°)")
        else:
            print("❌ 자연스러운 걷기 컨트롤러가 초기화되지 않음")
    
    def show_joint_status(self):
        """조인트 상태 출력"""
        if not self.data_combiner.motor:
            print("❌ 모터 센서가 연결되지 않음")
            return
        
        self.data_combiner.update_data()
        motor_data = self.data_combiner.motor.get_isaac_lab_data()
        
        print(f"\n🦾 조인트 상태 (자동 좌우 대칭):")
        print(f"{'조인트':<10} {'위치':<8} {'모터ID':<8} {'각도(deg)':<12} {'방향':<12} {'속도(rad/s)':<12}")
        print("-" * 80)
        
        for i in range(8):
            joint_name = JOINT_NAMES[i]
            side = "왼쪽" if i in LEFT_JOINTS else "오른쪽"
            motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[i]
            pos_rad = motor_data['joint_positions'][i]
            pos_deg = pos_rad * RAD_TO_DEG
            direction = "시계" if JOINT_DIRECTION_INVERT[i] else "반시계"
            vel_rad = motor_data['joint_velocities'][i]
            
            print(f"{joint_name:<10} {side:<8} {motor_id:<8} {pos_deg:<12.1f} {direction:<12} {vel_rad:<12.4f}")
    
    def save_data(self):
        """데이터 저장"""
        print("💾 데이터 저장 중...")
        
        self.data_combiner.update_data()
        
        success_count = 0
        if self.data_combiner.save_combined_obs():
            print("✅ Isaac Lab 텐서 저장 완료")
            success_count += 1
        
        if self.data_combiner.save_sensor_log():
            print("✅ 센서 로그 저장 완료")
            success_count += 1
        
        if success_count == 0:
            print("❌ 데이터 저장 실패")
        else:
            print(f"📁 저장 위치: {SAVE_DIRECTORY}")
    
    def toggle_monitoring(self):
        """모니터링 모드 토글"""
        self.monitoring_mode = not self.monitoring_mode
        
        if self.monitoring_mode:
            print("🔄 자동 모니터링 시작 (1초마다 상태 출력)")
        else:
            print("⏹️ 자동 모니터링 중지")
    
    def monitoring_loop(self):
        """모니터링 루프 (백그라운드)"""
        import threading
        
        def monitor():
            while self.is_running:
                if self.monitoring_mode:
                    print("\n" + "="*30 + " 자동 모니터링 " + "="*30)
                    self.show_isaac_tensor_info()
                    time.sleep(1)
                else:
                    time.sleep(0.1)
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        return monitor_thread
    
    def run_demo(self):
        """데모 실행"""
        if not self.initialize_system():
            print("❌ 시스템 초기화 실패")
            return False
        
        self.show_help()
        self.is_running = True
        
        # 모니터링 스레드 시작
        monitor_thread = self.monitoring_loop()
        
        # 키 매핑 (0-9키로 0°, 7°, 14°, ..., 63° 각도)
        motor_angle_keys = {str(i): i * 7 for i in range(10)}
        
        print("\n🎮 시스템 데모 시작! 명령어를 입력하세요:")
        
        try:
            while self.is_running:
                key = getch()
                
                if key == chr(27) or key == 'q':  # ESC 또는 q
                    print("\n데모 종료...")
                    break
                elif key == 's':
                    self.show_system_status()
                
                elif key == 'i':
                    self.show_isaac_tensor_info()
                
                elif key == 'm':
                    self.toggle_monitoring()
                
                elif key == 'w':
                    self.start_natural_walking(duration=10.0)

                elif key == 'W':
                    self.stop_walking()
                
                elif key == 'x':
                    self.test_natural_walking(duration=5.0)
                
                elif key == 'k':
                    self.test_knee_range()
                
                elif key == 'z':
                    self.check_walking_status()
                
                elif key == 'v':
                    self.show_camera_image()
                
                elif key == 'c':
                    self.show_camera_stats()
                
                elif key == 'h':
                    self.move_to_home()
                
                elif key == 't':
                    self.run_test_pattern()
                
                elif key in motor_angle_keys:
                    angle = motor_angle_keys[key]
                    # 안전 범위 (0-49도)
                    angle = min(angle, 49)
                    self.move_all_motors(angle)
                
                elif key == 'j':
                    self.show_joint_status()
                
                else:
                    print(f"🔤 '{key}' 입력됨 - 알 수 없는 명령어 (help: 도움말)")
        
        except KeyboardInterrupt:
            print("\n⚠️ 키보드 인터럽트")
        
        finally:
            self.is_running = False
            self.monitoring_mode = False
        
        return True
    
    def run_text_commands(self):
        """텍스트 명령어 모드"""
        if not self.initialize_system():
            print("❌ 시스템 초기화 실패")
            return False
        
        print("\n🎮 텍스트 명령어 모드 (자연스러운 걷기)")
        print("명령어: status, tensor, camera, move <angle>, home, test, walk, knee, save, help, quit")
        print("새로운 기능:")
        print("  walk: 자연스러운 걷기 시작")
        print("  knee: 무릎 범위 테스트 (0°~70°)")
        
        try:
            while True:
                cmd = input("\n명령어 입력: ").strip().lower()
                
                if cmd in ['quit', 'q', 'exit']:
                    break
                elif cmd in ['status', 's']:
                    self.show_system_status()
                elif cmd in ['tensor', 'i']:
                    self.show_isaac_tensor_info()
                elif cmd in ['camera', 'v']:
                    self.show_camera_image()
                elif cmd == 'home':
                    self.move_to_home()
                elif cmd == 'test':
                    self.run_test_pattern()
                elif cmd == 'walk':
                    self.start_natural_walking(duration=10.0)
                elif cmd == 'knee':
                    self.test_knee_range()
                elif cmd == 'joints':
                    self.show_joint_status()
                elif cmd == 'save':
                    self.save_data()
                elif cmd in ['help', 'h']:
                    self.show_help()
                elif cmd.startswith('move '):
                    try:
                        angle = int(cmd.split()[1])
                        if 0 <= angle <= 49:  # 안전 범위
                            self.move_all_motors(angle)
                        else:
                            print("각도는 0-49도 사이여야 합니다")
                    except (IndexError, ValueError):
                        print("사용법: move <각도>")
                        print("예시: move 15")
                else:
                    print("알 수 없는 명령어. 'help' 입력시 도움말")
        
        except KeyboardInterrupt:
            print("\n⚠️ 데모 중단")
        
        return True
    
    def shutdown(self):
        """시스템 종료"""
        print("🛑 시스템 데모 종료 중...")
        
        self.is_running = False
        self.monitoring_mode = False
        
        # 걷기 중지
        if self.walking_controller and self.walking_controller.walking:
            print("🚶 걷기 중지 중...")
            self.walking_controller.stop_walking()
            time.sleep(1)
        
        # 홈 포지션으로 복귀 (안전)
        if self.robot_controller.is_connected:
            print("🏠 홈 포지션으로 복귀...")
            self.move_to_home()
            self.robot_controller.disconnect()
        
        # 데이터 수집 중지
        self.data_combiner.close()
        
        print("✅ 시스템 데모 종료 완료")


def main():
    """메인 함수"""
    print("🎮 시스템 데모 시작 (자연스러운 걷기 + 자동 좌우 대칭)")
    
    demo = SystemDemo()
    
    try:
        print("실행 모드 선택:")
        print("1: 키보드 인터페이스 (실시간)")
        print("2: 텍스트 명령어 모드")
        
        choice = input("선택하세요 (1/2): ").strip()
        
        if choice == '1':
            demo.run_demo()
        elif choice == '2':
            demo.run_text_commands()
        else:
            print("기본값으로 텍스트 명령어 모드 실행")
            demo.run_text_commands()
    
    except KeyboardInterrupt:
        print("\n⚠️ 사용자가 중단했습니다")
    
    except Exception as e:
        print(f"❌ 데모 실행 오류: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        demo.shutdown()


if __name__ == "__main__":
    main()