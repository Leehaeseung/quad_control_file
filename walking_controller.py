#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
걷기 컨트롤러 - V2 + V3 (2가지 패턴)
보행 패턴 생성 및 실행 담당
"""

import time
import math
import threading
import numpy as np
from config import *


class WalkingController:
    """걷기 컨트롤러 - V2 + V3 (2가지 패턴)"""
    
    def __init__(self, robot_controller):
        """걷기 컨트롤러 초기화"""
        self.robot = robot_controller
        self.walking = False
        self.walk_thread = None
        self.current_version = 2
        
        print("🚶 걷기 컨트롤러 초기화 완료")
        print("  📐 LEG: -20°~63° (뒤로 밀기 강화)")
        print("  📐 KNEE: 0°~90° (무릎 범위 확장)")
        print("  🎯 V2 연속 밀어내기 + V3 12포인트 보행")
    
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
    
    def get_walking_v3_4point_cycle(self, phase):
        """버전3: 12포인트 직관적 보행 사이클"""
        try:
            # 12개 포인트로 한 사이클 정의 (직관적으로 설정)
            points = [
                {"phase": 0.0,  "leg": 10,  "knee": 0},    # 착지 시작
                {"phase": 0.12, "leg": 13,  "knee": 12},   # 초기 밀기
                {"phase": 0.24, "leg": 17, "knee": 30},   # 밀기 가속
                {"phase": 0.36, "leg": 22, "knee": 50},   # 중간 밀기
                {"phase": 0.5, "leg": 28, "knee": 70},   # 강한 밀기
                {"phase": 0.6, "leg": 38, "knee": 40},   # 최대 밀기
                {"phase": 0.68, "leg": 50, "knee": 0},   # 들어올리기 시작
                {"phase": 0.76, "leg": 38, "knee": 0},   # 무릎 구부리기
                {"phase": 0.84, "leg": 28, "knee": 0},   # 최대 들어올리기
                {"phase": 0.9, "leg": 18,  "knee": 0},   # 앞으로 스윙
                {"phase": 0.95, "leg": 14, "knee": 0},   # 착지 준비
                {"phase": 1.0,  "leg": 10,  "knee": 0}     # 착지 완료
            ]
            
            # 현재 phase에 맞는 구간 찾기
            for i in range(len(points) - 1):
                if phase >= points[i]["phase"] and phase <= points[i + 1]["phase"]:
                    p1 = points[i]
                    p2 = points[i + 1]
                    
                    # 구간 내에서 비율 계산
                    if p2["phase"] == p1["phase"]:
                        leg_motor = p1["leg"]
                        knee_motor = p1["knee"]
                    else:
                        t = (phase - p1["phase"]) / (p2["phase"] - p1["phase"])
                        
                        # 직선 보간 (단순하고 직관적)
                        leg_motor = p1["leg"] + (p2["leg"] - p1["leg"]) * t
                        knee_motor = p1["knee"] + (p2["knee"] - p1["knee"]) * t
                    
                    return {
                        "leg_motor": max(-20, min(63, leg_motor)),
                        "knee_motor": max(0, min(90, knee_motor))
                    }
            
            # 기본값
            return {"leg_motor": 0, "knee_motor": 0}
            
        except Exception as e:
            print(f"❌ V3 계산 오류: {e}")
            return {"leg_motor": 0, "knee_motor": 0}
    
    def _interpolate_points(self, points, phase, property):
        """포인트 보간 함수 (사용하지 않음 - 직관적 방식으로 대체)"""
        return 0
    
    def _ease_in_out(self, t):
        """부드러운 이징 함수 (사용하지 않음 - 직선 보간 사용)"""
        return t
    
    def get_diagonal_gait_angles(self, phase, version=2):
        """대각선 보행 패턴 (버전별)"""
        try:
            # 세트1: LB(왼뒤) + RF(오른앞)
            set1_phase = phase % 1.0
            # 세트2: LF(왼앞) + RB(오른뒤) - 반사이클 차이
            set2_phase = (phase + 0.5) % 1.0
            
            # 버전별 각도 계산
            if version == 2:
                set1_angles = self.get_walking_v2_continuous_push(set1_phase)
                set2_angles = self.get_walking_v2_continuous_push(set2_phase)
            elif version == 3:
                set1_angles = self.get_walking_v3_4point_cycle(set1_phase)
                set2_angles = self.get_walking_v3_4point_cycle(set2_phase)
            else:
                # 기본값 (V2)
                set1_angles = self.get_walking_v2_continuous_push(set1_phase)
                set2_angles = self.get_walking_v2_continuous_push(set2_phase)
            
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
            2: "연속 밀어내기 (멈춤 없는 연속성)",
            3: "12포인트 직관적 보행 사이클"
        }
        
        if version not in version_names:
            print(f"❌ 지원하지 않는 버전: V{version}")
            return
        
        self.current_version = version
        version_name = version_names[version]
        
        print(f"🚶 걷기 버전 {version} 시작: {version_name}")
        print(f"  📐 LEG: -20°~63° (뒤로 밀기 강화)")
        print(f"  📐 KNEE: 0°~90° (확장된 범위)")
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
                        if version == 2:  # 연속 버전
                            phase = (elapsed * speed * 0.5) % 1.0
                        elif version == 3:  # 12포인트 버전
                            phase = (elapsed * speed * 0.4) % 1.0  # 조금 더 천천히
                        else:
                            phase = (elapsed * speed * 0.5) % 1.0
                        
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
                                error_count = 0  # 성공 시 리셋
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
        
        for version in [2, 3]:
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
