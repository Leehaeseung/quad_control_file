#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
시스템 데모 - 다양한 걷기 버전들 (총 5가지)
실제 로봇 안정성을 위한 다양한 걷기 패턴 제공
LEG 범위: -20°~63° (뒤로 더 밀기)
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


class MultipleWalkingController:
    """다양한 걷기 버전 컨트롤러 - 5가지 패턴"""
    
    def __init__(self, robot_controller):
        """다양한 걷기 컨트롤러 초기화"""
        self.robot = robot_controller
        self.walking = False
        self.walk_thread = None
        self.current_version = 1
        
        # 확장된 측정 데이터 (LEG: -20°~63°, KNEE: 0°~63°)
        self.motor_data = [
            {"motor": -20, "knee": 5, "leg": 120},    # 뒤로 확장
            {"motor": -10, "knee": 8, "leg": 130},    # 뒤로 확장
            {"motor": 0, "knee": 14.5, "leg": 139},
            {"motor": 7, "knee": 17, "leg": 145},
            {"motor": 14, "knee": 21.8, "leg": 152},
            {"motor": 21, "knee": 25, "leg": 159},
            {"motor": 28, "knee": 33, "leg": 165},
            {"motor": 35, "knee": 35, "leg": 172},
            {"motor": 42, "knee": 39.1, "leg": 180},
            {"motor": 49, "knee": 42, "leg": 187},
            {"motor": 56, "knee": 48, "leg": 195},
            {"motor": 63, "knee": 55, "leg": 202}
        ]
        
        print("🚶 다양한 걷기 컨트롤러 초기화 완료")
        print("  📐 LEG: -20°~63° (뒤로 밀기 강화)")
        print("  📐 KNEE: 0°~63° (무릎 범위)")
        print("  🎯 5가지 걷기 버전 제공")
    
    def interpolate_motor_data(self, motor_angle):
        """모터 각도 보간 (LEG -20°~63°)"""
        try:
            motor_angle = max(-20, min(90, motor_angle))
            
            for i in range(len(self.motor_data) - 1):
                if motor_angle <= self.motor_data[i + 1]["motor"]:
                    data1 = self.motor_data[i]
                    data2 = self.motor_data[i + 1]
                    
                    if data1["motor"] == data2["motor"]:
                        return data1
                    
                    t = (motor_angle - data1["motor"]) / (data2["motor"] - data1["motor"])
                    return {
                        "knee": data1["knee"] + (data2["knee"] - data1["knee"]) * t,
                        "leg": data1["leg"] + (data2["leg"] - data1["leg"]) * t
                    }
            
            return self.motor_data[-1]
        except Exception as e:
            print(f"❌ 보간 오류: {e}")
            return {"knee": 14.5, "leg": 139}
    
    def get_walking_v1_slow_stable(self, phase):
        """버전1: 느리고 안정적인 걷기 (바닥 접촉 중시)"""
        try:
            # 느린 속도, 긴 바닥 접촉 시간
            leg_center = 21.5   # 중심점 (약간 뒤쪽)
            knee_center = 31.5
            
            # 바닥 접촉 강화
            if phase < 0.3:  # 들어올리기 (30%)
                leg_angle = leg_center + 15 * math.sin(phase * math.pi / 0.3)
                knee_angle = knee_center + 20 * math.sin(phase * math.pi / 0.3)
            elif phase < 0.7:  # 바닥 접촉 및 밀기 (40% - 길게!)
                t = (phase - 0.3) / 0.4
                leg_angle = leg_center + 15 - 35 * t  # 앞→뒤로 천천히
                knee_angle = knee_center + 5 * math.cos(t * math.pi)  # 무릎 안정
            else:  # 복귀 (30%)
                t = (phase - 0.7) / 0.3
                leg_angle = leg_center - 20 + 35 * t
                knee_angle = knee_center + 10 * math.sin(t * math.pi)
            
            return {
                "leg_motor": max(-20, min(63, leg_angle)),
                "knee_motor": max(0, min(90, knee_angle))
            }
        except Exception as e:
            print(f"❌ V1 계산 오류: {e}")
            return {"leg_motor": 21.5, "knee_motor": 31.5}
    
    def get_walking_v2_continuous_push(self, phase):
        """버전2: 연속 밀어내기 (멈춤 없는 연속성)"""
        try:
            leg_center = 21.5
            knee_center = 31.5
            
            # 연속적인 사인파 (멈춤 없음)
            angle = 2 * math.pi * phase
            
            # 다리: 뒤로 밀기 강화
            leg_angle = leg_center + 25 * math.cos(angle) + 15 * math.cos(2 * angle)
            
            # 무릎: 부드러운 연속 움직임
            knee_angle = knee_center + 20 * math.cos(angle + math.pi/4) + 10 * math.sin(3 * angle)
            
            return {
                "leg_motor": max(-20, min(63, leg_angle)),
                "knee_motor": max(0, min(90, knee_angle))
            }
        except Exception as e:
            print(f"❌ V2 계산 오류: {e}")
            return {"leg_motor": 21.5, "knee_motor": 31.5}
    
    def get_walking_v3_balance_focused(self, phase):
        """버전3: 균형 중심 (넘어짐 방지)"""
        try:
            leg_center = 31.5   # 중심을 앞쪽으로 (균형)
            knee_center = 25    # 무릎 낮게 (안정성)
            
            # 4단계 명확한 구분
            if phase < 0.25:    # 무릎 들기
                leg_angle = leg_center
                knee_angle = knee_center + 25 * (phase / 0.25)
            elif phase < 0.5:   # 앞으로 내딛기
                t = (phase - 0.25) / 0.25
                leg_angle = leg_center + 20 * t
                knee_angle = knee_center + 25 * (1 - 0.5 * t)
            elif phase < 0.75:  # 바닥 접촉 유지
                leg_angle = leg_center + 20
                knee_angle = knee_center + 12.5
            else:              # 뒤로 밀기
                t = (phase - 0.75) / 0.25
                leg_angle = leg_center + 20 - 40 * t
                knee_angle = knee_center + 12.5 * (1 - t)
            
            return {
                "leg_motor": max(-20, min(63, leg_angle)),
                "knee_motor": max(0, min(90, knee_angle))
            }
        except Exception as e:
            print(f"❌ V3 계산 오류: {e}")
            return {"leg_motor": 31.5, "knee_motor": 25}
    
    def get_walking_v4_strong_push(self, phase):
        """버전4: 강한 밀어내기 (추진력 중시)"""
        try:
            leg_center = 21.5
            knee_center = 31.5
            
            # 강한 뒤로 밀기에 집중
            if phase < 0.2:     # 빠른 들어올리기
                t = phase / 0.2
                leg_angle = leg_center + 30 * math.sin(t * math.pi)
                knee_angle = knee_center + 25 * math.sin(t * math.pi)
            elif phase < 0.4:   # 앞으로 빠르게
                t = (phase - 0.2) / 0.2
                leg_angle = leg_center + 30 - 20 * t
                knee_angle = knee_center + 25 - 15 * t
            elif phase < 0.8:   # 길고 강한 밀어내기 (40%)
                t = (phase - 0.4) / 0.4
                leg_angle = leg_center + 10 - 50 * t  # 강하게 뒤로!
                knee_angle = knee_center + 10 - 15 * t
            else:              # 빠른 복귀
                t = (phase - 0.8) / 0.2
                leg_angle = leg_center - 40 + 70 * t
                knee_angle = knee_center - 5 + 30 * t
            
            return {
                "leg_motor": max(-20, min(63, leg_angle)),
                "knee_motor": max(0, min(63, knee_angle))
            }
        except Exception as e:
            print(f"❌ V4 계산 오류: {e}")
            return {"leg_motor": 21.5, "knee_motor": 31.5}
    
    def get_walking_v5_natural_gait(self, phase):
        """버전5: 자연스러운 보행 (인간과 유사)"""
        try:
            leg_center = 21.5
            knee_center = 31.5
            
            # 자연스러운 보행 패턴
            angle = 2 * math.pi * phase
            
            # 다리: 자연스러운 타원 (앞뒤 길게, 위아래 짧게)
            leg_angle = leg_center + 30 * math.cos(angle) + 8 * math.cos(3 * angle)
            
            # 무릎: 들어올릴 때만 구부리기
            if 0.2 < phase < 0.6:  # 스윙 페이즈에서만 무릎 구부림
                knee_lift = 20 * math.sin((phase - 0.2) * math.pi / 0.4)
            else:
                knee_lift = 0
            
            knee_angle = knee_center + knee_lift + 5 * math.cos(angle + math.pi/6)
            
            return {
                "leg_motor": max(-20, min(63, leg_angle)),
                "knee_motor": max(0, min(90, knee_angle))
            }
        except Exception as e:
            print(f"❌ V5 계산 오류: {e}")
            return {"leg_motor": 21.5, "knee_motor": 31.5}
    
    def get_diagonal_gait_angles(self, phase, version=1):
        """대각선 보행 패턴 (버전별)"""
        try:
            # 세트1: LB(왼뒤) + RF(오른앞)
            set1_phase = phase % 1.0
            # 세트2: LF(왼앞) + RB(오른뒤) - 반사이클 차이
            set2_phase = (phase + 0.5) % 1.0
            
            # 버전별 각도 계산
            if version == 1:
                set1_angles = self.get_walking_v1_slow_stable(set1_phase)
                set2_angles = self.get_walking_v1_slow_stable(set2_phase)
            elif version == 2:
                set1_angles = self.get_walking_v2_continuous_push(set1_phase)
                set2_angles = self.get_walking_v2_continuous_push(set2_phase)
            elif version == 3:
                set1_angles = self.get_walking_v3_balance_focused(set1_phase)
                set2_angles = self.get_walking_v3_balance_focused(set2_phase)
            elif version == 4:
                set1_angles = self.get_walking_v4_strong_push(set1_phase)
                set2_angles = self.get_walking_v4_strong_push(set2_phase)
            elif version == 5:
                set1_angles = self.get_walking_v5_natural_gait(set1_phase)
                set2_angles = self.get_walking_v5_natural_gait(set2_phase)
            else:
                # 기본값
                set1_angles = {"leg_motor": 21.5, "knee_motor": 31.5}
                set2_angles = {"leg_motor": 21.5, "knee_motor": 31.5}
            
            # 8개 조인트 각도 배열
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
        except Exception as e:
            print(f"❌ 대각선 보행 오류: {e}")
            return [21.5] * 8  # 안전 기본값
    
    def start_walking_version(self, version, duration=10.0, speed=1.0):
        """특정 버전의 걷기 시작"""
        if self.walking:
            print("⚠️ 이미 걷기 중입니다")
            return
        
        version_names = {
            1: "느리고 안정적인 걷기 (바닥 접촉 중시)",
            2: "연속 밀어내기 (멈춤 없는 연속성)",
            3: "균형 중심 (넘어짐 방지)",
            4: "강한 밀어내기 (추진력 중시)",
            5: "자연스러운 보행 (인간과 유사)"
        }
        
        self.current_version = version
        version_name = version_names.get(version, "알 수 없는 버전")
        
        print(f"🚶 걷기 버전 {version} 시작: {version_name}")
        print(f"  📐 LEG: -20°~63° (뒤로 밀기 강화)")
        print(f"  ⏱️ {duration}초간, 속도 {speed}x")
        
        self.walking = True
        
        def walk_loop():
            """걷기 루프"""
            start_time = time.time()
            cycle_count = 0
            error_count = 0
            max_errors = 3
            
            try:
                while self.walking and (time.time() - start_time) < duration:
                    loop_start = time.time()
                    
                    try:
                        elapsed = time.time() - start_time
                        
                        # 버전별 속도 조정
                        if version == 1:  # 느린 버전
                            phase = (elapsed * speed * 0.3) % 1.0
                        elif version == 2:  # 연속 버전
                            phase = (elapsed * speed * 0.5) % 1.0
                        elif version == 3:  # 균형 버전
                            phase = (elapsed * speed * 0.4) % 1.0
                        elif version == 4:  # 강한 버전
                            phase = (elapsed * speed * 0.6) % 1.0
                        else:  # 자연 버전
                            phase = (elapsed * speed * 0.45) % 1.0
                        
                        # 대각선 보행 각도 계산
                        joint_angles = self.get_diagonal_gait_angles(phase, version)
                        
                        # 모터 명령 실행
                        if self.robot and self.robot.motor_sensor:
                            success = self.robot.motor_sensor.move_all_joints(joint_angles)
                            if not success:
                                error_count += 1
                                if error_count >= max_errors:
                                    print(f"🚨 모터 오류 {max_errors}회, 안전 정지")
                                    break
                        else:
                            break
                        
                        # 상태 출력 (3초마다)
                        new_cycle = int(elapsed)
                        if new_cycle > cycle_count and new_cycle % 3 == 0:
                            cycle_count = new_cycle
                            print(f"🔄 걷기 V{version}: {elapsed:.1f}초 경과, 위상 {phase:.2f}")
                    
                    except Exception as e:
                        error_count += 1
                        print(f"⚠️ 걷기 루프 오류 {error_count}/{max_errors}: {e}")
                        if error_count >= max_errors:
                            break
                    
                    # 부드러운 60Hz 제어
                    loop_time = time.time() - loop_start
                    sleep_time = (1.0 / 60.0) - loop_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
            
            except Exception as e:
                print(f"❌ 걷기 V{version} 중 오류: {e}")
            
            finally:
                self.walking = False
                print(f"🛑 걷기 V{version} 종료")
        
        self.walk_thread = threading.Thread(target=walk_loop, daemon=True)
        self.walk_thread.start()
    
    def stop_walking(self):
        """걷기 중지"""
        if self.walking:
            self.walking = False
            print("⏹️ 걷기 중지됨")
            if self.walk_thread and self.walk_thread.is_alive():
                self.walk_thread.join(timeout=2.0)
        else:
            print("⚠️ 걷기 중이 아닙니다")
    
    def test_all_versions(self, duration_each=5.0):
        """모든 버전 순차 테스트"""
        if self.walking:
            print("⚠️ 이미 걷기 중입니다")
            return
        
        print("🧪 모든 걷기 버전 순차 테스트 시작!")
        
        for version in range(1, 6):
            try:
                print(f"\n===== 버전 {version} 테스트 =====")
                self.start_walking_version(version, duration_each, speed=0.8)
                
                # 테스트 완료까지 대기
                while self.walking:
                    time.sleep(0.5)
                
                print(f"✅ 버전 {version} 테스트 완료")
                time.sleep(2)  # 버전간 휴식
                
            except Exception as e:
                print(f"❌ 버전 {version} 테스트 오류: {e}")
                self.stop_walking()
                time.sleep(1)
        
        print("🎉 모든 버전 테스트 완료!")


class SystemDemo:
    def __init__(self):
        """시스템 데모 초기화"""
        print("🎮 다양한 걷기 버전 시스템 데모 초기화 중...")
        
        self.data_combiner = DataCombiner()
        self.robot_controller = RobotController()
        self.walking_controller = None
        
        self.is_running = False
        self.monitoring_mode = False
        
        print("✅ 시스템 데모 초기화 완료")
    
    def initialize_system(self):
        """시스템 초기화"""
        print("🔧 시스템 초기화 중...")
        
        success_count = 0
        
        try:
            if self.data_combiner.start_data_collection():
                print("✅ 센서 시스템 초기화 성공")
                success_count += 1
            else:
                print("❌ 센서 시스템 초기화 실패")
        except Exception as e:
            print(f"❌ 센서 시스템 오류: {e}")
        
        try:
            if self.robot_controller.connect_motors():
                if self.robot_controller.enable_control():
                    print("✅ 로봇 제어기 초기화 성공")
                    success_count += 1
                    
                    self.walking_controller = MultipleWalkingController(self.robot_controller)
                    print("✅ 다양한 걷기 컨트롤러 초기화 성공")
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
        print("🎮 다양한 걷기 버전 테스트 (LEG: -20°~63°)")
        print("="*80)
        print("📊 시스템:")
        print("  s: 시스템 상태")
        print("  h: 홈 포지션")
        print("  m: 모니터링 토글")
        print()
        print("🚶 걷기 버전들 (10초간):")
        print("  1: 느리고 안정적 (바닥 접촉 중시) - 넘어짐 방지")
        print("  2: 연속 밀어내기 (멈춤 없는 연속성) - 매끄러운 움직임")
        print("  3: 균형 중심 (넘어짐 방지) - 안정성 최우선")
        print("  4: 강한 밀어내기 (추진력 중시) - 빠른 전진")
        print("  5: 자연스러운 보행 (인간과 유사) - 자연스러움")
        print()
        print("🧪 테스트:")
        print("  t: 모든 버전 순차 테스트 (각 5초)")
        print("  x: 현재 걷기 중지")
        print("  z: 걷기 상태 확인")
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
        print("\n🔍 다양한 걷기 버전 시스템 상태:")
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
                print(f"\n🚶 다양한 걷기 컨트롤러:")
                print(f"  초기화됨: ✅")
                print(f"  걷기 중: {'✅' if self.walking_controller.walking else '❌'}")
                print(f"  현재 버전: V{self.walking_controller.current_version}")
                print(f"  LEG 범위: -20°~63° (뒤로 밀기 강화)")
                print(f"  KNEE 범위: 0°~63°")
                print(f"  총 5가지 버전 제공")
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
        """안전한 범위 테스트 (-20°~63°)"""
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        try:
            print("🧪 확장된 범위 테스트 (-20°~63°)")
            test_angles = [-20, -10, 0, 15, 30, 50, 70, 90, 70, 50, 15, 0, -10, -20]
            
            for i, angle in enumerate(test_angles):
                try:
                    print(f"스텝 {i+1}/{len(test_angles)}: {angle}°")
                    joint_angles = [angle] * 8
                    
                    if self.robot_controller.motor_sensor.move_all_joints(joint_angles):
                        print(f"✅ {angle}° 이동 성공")
                    else:
                        print(f"❌ {angle}° 이동 실패")
                    
                    time.sleep(1.2)
                except Exception as e:
                    print(f"⚠️ 스텝 {i+1} 오류: {e}")
                    continue
            
            print("✅ 범위 테스트 완료!")
        except Exception as e:
            print(f"❌ 범위 테스트 오류: {e}")
    
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
                    cv2.imshow('Multiple Walking Demo - Camera', combined)
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
    
    def start_walking_version(self, version):
        """특정 버전 걷기 시작"""
        if not self.walking_controller:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
            return
        
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return
        
        try:
            self.walking_controller.start_walking_version(version, duration=10.0, speed=1.0)
        except Exception as e:
            print(f"❌ 걷기 V{version} 시작 오류: {e}")
    
    def test_all_versions(self):
        """모든 버전 순차 테스트"""
        if not self.walking_controller:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
            return
        
        try:
            self.walking_controller.test_all_versions(duration_each=5.0)
        except Exception as e:
            print(f"❌ 전체 버전 테스트 오류: {e}")
    
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
                    1: "느리고 안정적 (바닥 접촉 중시)",
                    2: "연속 밀어내기 (멈춤 없는 연속성)", 
                    3: "균형 중심 (넘어짐 방지)",
                    4: "강한 밀어내기 (추진력 중시)",
                    5: "자연스러운 보행 (인간과 유사)"
                }
                
                print(f"🚶 걷기 상태: {status}")
                print(f"  현재 버전: V{current_v} - {version_names.get(current_v, '알 수 없음')}")
                print(f"  LEG 범위: -20°~63° (뒤로 밀기 강화)")
                print(f"  KNEE 범위: 0°~63°")
            except Exception as e:
                print(f"❌ 걷기 상태 확인 오류: {e}")
        else:
            print("❌ 걷기 컨트롤러가 초기화되지 않음")
    
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
        
        print("\n🎮 다양한 걷기 버전 데모 시작!")
        print("숫자 1-5로 걷기 버전 선택하세요!")
        
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
                    elif key in ['1', '2', '3', '4', '5']:
                        version = int(key)
                        self.start_walking_version(version)
                    elif key == 't':
                        self.test_all_versions()
                    elif key == 'x':
                        self.stop_walking()
                    elif key == 'z':
                        self.check_walking_status()
                    elif key == 'r':
                        self.test_range_safely()
                    elif key == 'v':
                        self.show_camera_image()
                    elif key == 'c':
                        self.show_camera_stats()
                    elif key in motor_angle_keys:
                        angle = motor_angle_keys[key]
                        self.move_all_motors(angle)
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
        
        print("\n🎮 텍스트 명령어 모드 (다양한 걷기 버전)")
        print("명령어:")
        print("  v1, v2, v3, v4, v5: 걷기 버전 1-5 시작")
        print("  test: 모든 버전 순차 테스트")
        print("  stop: 걷기 중지")
        print("  status: 시스템 상태")
        print("  home: 홈 포지션")
        print("  range: 범위 테스트")
        print("  help: 도움말")
        print("  quit: 종료")
        
        version_names = {
            "v1": (1, "느리고 안정적 (바닥 접촉 중시)"),
            "v2": (2, "연속 밀어내기 (멈춤 없는 연속성)"),
            "v3": (3, "균형 중심 (넘어짐 방지)"),
            "v4": (4, "강한 밀어내기 (추진력 중시)"),
            "v5": (5, "자연스러운 보행 (인간과 유사)")
        }
        
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
                    elif cmd in version_names:
                        version_num, version_desc = version_names[cmd]
                        print(f"🚶 시작: V{version_num} - {version_desc}")
                        self.start_walking_version(version_num)
                    elif cmd == 'test':
                        print("🧪 모든 버전 순차 테스트 시작...")
                        self.test_all_versions()
                    elif cmd in ['stop', 'x']:
                        self.stop_walking()
                    elif cmd in ['range', 'r']:
                        self.test_range_safely()
                    elif cmd in ['camera', 'v']:
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
                        print("걷기 버전: v1, v2, v3, v4, v5")
                
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
        print("🛑 다양한 걷기 버전 시스템 종료 중...")
        
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
            
            print("✅ 다양한 걷기 버전 시스템 종료 완료")
        
        except Exception as e:
            print(f"❌ 시스템 종료 중 오류: {e}")


def main():
    """메인 함수"""
    print("🚀 다양한 걷기 버전 시스템 시작")
    print("🎯 5가지 걷기 패턴 제공")
    print("📐 LEG 범위: -20°~63° (뒤로 밀기 강화)")
    print("🛡️ 완벽한 안정성 보장")
    
    demo = None
    
    try:
        demo = SystemDemo()
        
        print("\n🎮 실행 모드 선택:")
        print("1: 키보드 인터페이스 (실시간 - 숫자 1-5로 버전 선택)")
        print("2: 텍스트 명령어 모드 (v1, v2, v3, v4, v5 명령어)")
        print("\n🚶 걷기 버전 설명:")
        print("  V1: 느리고 안정적 (바닥 접촉 중시) - 넘어짐 방지 최우선")
        print("  V2: 연속 밀어내기 (멈춤 없는 연속성) - 매끄러운 움직임")
        print("  V3: 균형 중심 (넘어짐 방지) - 안정성과 균형 최우선")
        print("  V4: 강한 밀어내기 (추진력 중시) - 빠른 전진력")
        print("  V5: 자연스러운 보행 (인간과 유사) - 자연스러운 걸음걸이")
        
        try:
            choice = input("\n선택하세요 (1/2): ").strip()
        except KeyboardInterrupt:
            print("\n사용자가 취소했습니다.")
            return 0
        
        if choice == '1':
            print("\n🎮 키보드 모드:")
            print("  숫자 1-5: 각 걷기 버전 시작")
            print("  t: 모든 버전 순차 테스트")
            print("  x: 걷기 중지")
            print("  help: 전체 도움말")
            demo.run_demo()
        elif choice == '2':
            print("\n📝 텍스트 모드:")
            print("  v1, v2, v3, v4, v5: 각 버전 시작")
            print("  test: 모든 버전 테스트")
            print("  help: 도움말")
            demo.run_text_commands()
        else:
            print("기본값으로 텍스트 명령어 모드 실행")
            demo.run_text_commands()
    
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
        print(f"\n🎯 걷기 버전 테스트 요약:")
        print(f"  V1: 안정성 중심 - 천천히 확실하게")
        print(f"  V2: 연속성 중심 - 매끄럽고 부드럽게") 
        print(f"  V3: 균형 중심 - 넘어지지 않게")
        print(f"  V4: 추진력 중심 - 강하게 밀어내기")
        print(f"  V5: 자연성 중심 - 인간처럼 자연스럽게")
        print(f"\n🏆 베스트 버전을 찾아서 실제 활용하세요!")
        sys.exit(exit_code)
    except Exception as e:
        print(f"❌ 프로그램 실행 중 치명적 오류: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
