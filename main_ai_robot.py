#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 로봇 메인 시스템
Isaac Lab 학습된 AI 모델과 실제 로봇을 연동하는 메인 실행 파일
"""

import os
import sys
import time
import signal
import numpy as np
from datetime import datetime
from config import *
from data_combiner import DataCombiner
from ai_model_interface import AIModelInterface
from robot_controller import RobotController


class AIRobotSystem:
    def __init__(self, model_path=None):
        """AI 로봇 시스템 초기화"""
        print("🤖 AI 로봇 시스템 초기화 중...")
        
        # 모듈들
        self.data_combiner = DataCombiner()
        self.ai_interface = AIModelInterface(model_path)
        self.robot_controller = RobotController()
        
        # 시스템 상태
        self.is_running = False
        self.control_enabled = False
        self.model_loaded = False
        
        # 성능 모니터링
        self.loop_count = 0
        self.start_time = None
        self.last_fps_time = None
        self.current_fps = 0.0
        
        # 안전 설정
        self.max_consecutive_failures = 10
        self.consecutive_failures = 0
        
        if DEBUG_MODE:
            print("✅ AI 로봇 시스템 초기화 완료")
    
    def setup_signal_handlers(self):
        """시그널 핸들러 설정 (안전한 종료)"""
        def signal_handler(signum, frame):
            print(f"\n🛑 시그널 {signum} 수신, 안전하게 종료 중...")
            self.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, signal_handler)  # 종료 요청
    
    def initialize_system(self, model_path=None):
        """전체 시스템 초기화"""
        print("🔧 시스템 초기화 중...")
        
        success_count = 0
        
        # 1. 센서 시스템 초기화
        if self.data_combiner.start_data_collection():
            print("✅ 센서 시스템 초기화 성공")
            success_count += 1
        else:
            print("❌ 센서 시스템 초기화 실패")
        
        # 2. AI 모델 로드
        if model_path and self.ai_interface.load_model(model_path):
            print("✅ AI 모델 로드 성공")
            self.model_loaded = True
            success_count += 1
        else:
            print("⚠️ AI 모델 로드 실패 또는 경로 없음")
            self.model_loaded = False
        
        # 3. 로봇 제어기 연결
        if self.robot_controller.connect_motors():
            print("✅ 로봇 제어기 연결 성공")
            success_count += 1
        else:
            print("❌ 로봇 제어기 연결 실패")
        
        print(f"\n📊 시스템 초기화 결과: {success_count}/3 성공")
        
        return success_count >= 2  # 최소 2개 이상 성공해야 동작
    
    def enable_ai_control(self):
        """AI 제어 활성화"""
        if not self.model_loaded:
            print("❌ AI 모델이 로드되지 않음")
            return False
        
        if not self.robot_controller.is_connected:
            print("❌ 로봇이 연결되지 않음")
            return False
        
        if not self.robot_controller.enable_control():
            print("❌ 로봇 제어 활성화 실패")
            return False
        
        self.control_enabled = True
        print("🟢 AI 제어 활성화")
        return True
    
    def disable_ai_control(self):
        """AI 제어 비활성화"""
        self.control_enabled = False
        self.robot_controller.disable_control()
        print("🔴 AI 제어 비활성화")
    
    def run_ai_control_loop(self, duration_seconds=None, enable_control=False):
        """AI 제어 메인 루프"""
        if not self.data_combiner.is_running:
            print("❌ 데이터 수집이 실행되지 않음")
            return False
        
        print(f"🚀 AI 제어 루프 시작 ({SYSTEM_FREQUENCY}Hz)")
        if duration_seconds:
            print(f"⏱️ {duration_seconds}초간 실행")
        
        if enable_control:
            if not self.enable_ai_control():
                print("⚠️ AI 제어 활성화 실패, 모니터링 모드로 실행")
                enable_control = False
        
        # 제어 모드 확인
        if enable_control and self.control_enabled:
            print("🤖 완전한 AI 제어 모드 - 실제 로봇이 움직입니다!")
        elif self.model_loaded:
            print("👁️ AI 모니터링 모드 - 추론만 실행, 제어 없음")
        else:
            print("📊 센서 모니터링 모드 - 센서 데이터만 수집")
        
        self.is_running = True
        self.start_time = time.time()
        self.last_fps_time = self.start_time
        self.loop_count = 0
        self.consecutive_failures = 0
        
        try:
            while self.is_running:
                loop_start_time = time.time()
                
                # 메인 AI 제어 사이클
                success = self._ai_control_cycle()
                
                # 실패 카운터 관리
                if not success:
                    self.consecutive_failures += 1
                    if self.consecutive_failures >= self.max_consecutive_failures:
                        print(f"🚨 연속 실패 {self.max_consecutive_failures}회 도달, 안전 정지")
                        if self.control_enabled:
                            self.disable_ai_control()
                        break
                else:
                    self.consecutive_failures = 0
                
                # 실행 시간 체크
                if duration_seconds:
                    elapsed = time.time() - self.start_time
                    if elapsed >= duration_seconds:
                        print(f"⏰ 설정된 실행 시간 {duration_seconds}초 완료")
                        break
                
                # FPS 계산 및 출력
                self.loop_count += 1
                if self.loop_count % 30 == 0:  # 30번마다 FPS 계산
                    current_time = time.time()
                    if self.last_fps_time:
                        self.current_fps = 30 / (current_time - self.last_fps_time)
                    self.last_fps_time = current_time
                    
                    if DEBUG_MODE:
                        status = "제어" if self.control_enabled else "모니터링"
                        print(f"🔄 AI 루프 FPS: {self.current_fps:.1f} ({status} 모드)")
                
                # 주기 맞춤
                loop_time = time.time() - loop_start_time
                sleep_time = (1.0 / SYSTEM_FREQUENCY) - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n⚠️ 사용자가 중단했습니다")
        
        except Exception as e:
            print(f"❌ AI 제어 루프 오류: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.is_running = False
            if self.control_enabled:
                self.disable_ai_control()
            
            # 최종 통계 출력
            self._print_final_stats()
        
        return True
    
    def _ai_control_cycle(self):
        """단일 AI 제어 사이클"""
        try:
            # 1. 센서 데이터 업데이트
            if not self.data_combiner.update_data():
                if VERBOSE_LOGGING:
                    print("⚠️ 센서 데이터 업데이트 실패")
                return False
            
            # 2. Isaac Lab 텐서 가져오기
            isaac_tensor = self.data_combiner.get_latest_isaac_tensor()
            if isaac_tensor is None:
                if VERBOSE_LOGGING:
                    print("⚠️ Isaac Lab 텐서 없음")
                return False
            
            # 3. AI 모델 추론
            if self.model_loaded:
                prediction = self.ai_interface.predict(isaac_tensor)
                
                if prediction['success']:
                    isaac_joint_angles = prediction['isaac_joint_angles']
                    
                    # Isaac Lab 각도 → Dynamixel 각도 매핑 (radian → degree)
                    dynamixel_angles_deg = []
                    for isaac_idx in range(8):
                        angle_rad = isaac_joint_angles[isaac_idx]
                        angle_deg = angle_rad * RAD_TO_DEG
                        # 안전 범위 적용
                        safe_angle = np.clip(angle_deg, MOTOR_MIN_ANGLE, MOTOR_MAX_ANGLE)
                        dynamixel_angles_deg.append(safe_angle)
                    
                    # 디버그 출력
                    if VERBOSE_LOGGING or self.loop_count % 30 == 0:
                        print(f"🎯 AI 예측 각도:")
                        for i, (isaac_angle, dxl_angle) in enumerate(zip(isaac_joint_angles, dynamixel_angles_deg)):
                            joint_name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"joint_{i}"
                            motor_id = ISAAC_TO_DYNAMIXEL_MAPPING[i] if i < len(ISAAC_TO_DYNAMIXEL_MAPPING) else "?"
                            print(f"   {joint_name} (ID{motor_id}): {isaac_angle:.4f}rad → {dxl_angle:.1f}°")
                    
                    # 🚀 실제 모터 제어 실행 (활성화됨!)
                    if self.control_enabled:
                        success = self.robot_controller.execute_joint_command(dynamixel_angles_deg)
                        if not success:
                            if VERBOSE_LOGGING:
                                print("⚠️ 모터 제어 실패")
                            return False
                        else:
                            if VERBOSE_LOGGING:
                                print("✅ 모터 제어 성공")
                    else:
                        # 모니터링 모드에서는 출력만
                        if self.loop_count % 30 == 0:
                            print(f"👁️ 모니터링 모드 - 제어 명령: {[f'{a:.1f}°' for a in dynamixel_angles_deg]}")
                
                else:
                    if VERBOSE_LOGGING:
                        print("⚠️ AI 추론 실패")
                    return False
            else:
                # AI 모델이 없는 경우 - 센서 데이터만 출력
                if self.loop_count % 60 == 0:  # 2초마다
                    isaac_data = self.data_combiner.get_latest_isaac_data()
                    if isaac_data:
                        values_str = self.data_combiner.converter.format_isaac_values_string(isaac_data)
                        print(f"📊 센서 데이터: {values_str}")
            
            return True
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"AI 제어 사이클 오류: {e}")
                import traceback
                traceback.print_exc()
            return False

    
    def run_monitoring_mode(self, duration_seconds=10):
        """모니터링 모드 (AI 추론만, 제어 없음)"""
        print("👁️ 모니터링 모드 실행")
        return self.run_ai_control_loop(duration_seconds=duration_seconds, enable_control=False)
    
    def run_full_ai_control(self, duration_seconds=None):
        """완전한 AI 제어 모드"""
        print("🤖 완전한 AI 제어 모드 실행")
        return self.run_ai_control_loop(duration_seconds=duration_seconds, enable_control=True)
    
    def _print_final_stats(self):
        """최종 통계 출력"""
        if self.start_time:
            total_time = time.time() - self.start_time
            avg_fps = self.loop_count / total_time if total_time > 0 else 0
            
            print(f"\n📈 최종 통계:")
            print(f"   총 실행 시간: {total_time:.1f}초")
            print(f"   총 루프 횟수: {self.loop_count}")
            print(f"   평균 FPS: {avg_fps:.1f}")
            print(f"   연속 실패 횟수: {self.consecutive_failures}")
            
            # AI 모델 성능 통계
            if self.model_loaded:
                ai_stats = self.ai_interface.get_performance_stats()
                print(f"   AI 추론 횟수: {ai_stats['total_inferences']}")
                print(f"   AI 평균 시간: {ai_stats['average_time']*1000:.1f}ms")
            
            # 로봇 제어 통계
            if self.robot_controller.is_connected:
                control_stats = self.robot_controller.get_control_stats()
                print(f"   제어 명령 횟수: {control_stats['total_commands']}")
                print(f"   제어 성공률: {control_stats['success_rate_percent']:.1f}%")
    
    def print_system_status(self):
        """시스템 상태 출력"""
        print("\n🔍 AI 로봇 시스템 상태:")
        print("-" * 50)
        print(f"실행 중: {'✅' if self.is_running else '❌'}")
        print(f"AI 모델 로드됨: {'✅' if self.model_loaded else '❌'}")
        print(f"AI 제어 활성화: {'✅' if self.control_enabled else '❌'}")
        print(f"현재 FPS: {self.current_fps:.1f}")
        print(f"연속 실패: {self.consecutive_failures}/{self.max_consecutive_failures}")
        
        # 각 모듈 상태
        self.data_combiner.print_system_status()
        
        print("-" * 50)
    
    def shutdown(self):
        """시스템 안전 종료"""
        print("🛑 AI 로봇 시스템 종료 중...")
        
        self.is_running = False
        
        if self.control_enabled:
            self.disable_ai_control()
        
        # 홈 포지션으로 이동 (안전)
        if self.robot_controller.is_connected:
            print("🏠 홈 포지션으로 복귀 중...")
            self.robot_controller.enable_control()
            self.robot_controller.move_to_home_position()
            self.robot_controller.disconnect()
        
        # 모든 모듈 정리
        self.data_combiner.close()
        
        if self.ai_interface:
            self.ai_interface.unload_model()
        
        print("✅ AI 로봇 시스템 종료 완료")


def main():
    """메인 함수"""
    print("🤖 AI 로봇 시스템 시작")
    
    # 명령행 인수 처리
    model_path = None
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
        if not os.path.exists(model_path):
            print(f"❌ 모델 파일을 찾을 수 없음: {model_path}")
            model_path = None
    
    if not model_path:
        print("⚠️ AI 모델 경로가 지정되지 않음")
        print("사용법: python main_ai_robot.py <model_path.pt>")
        print("모니터링 모드로 실행합니다...")
    
    # AI 로봇 시스템 초기화
    robot_system = AIRobotSystem(model_path)
    robot_system.setup_signal_handlers()
    
    try:
        # 시스템 초기화
        if not robot_system.initialize_system(model_path):
            print("❌ 시스템 초기화 실패")
            return 1
        
        # 사용자 선택
        print("\n🎮 실행 모드 선택:")
        print("1: 모니터링 모드 (10초간, AI 추론만)")
        print("2: 전체 AI 제어 모드 (무한, 실제 제어)")
        print("3: 시스템 상태 확인 후 종료")
        
        try:
            choice = input("선택하세요 (1-3): ").strip()
        except KeyboardInterrupt:
            print("\n사용자가 취소했습니다.")
            return 0
        
        if choice == '1':
            robot_system.run_monitoring_mode(duration_seconds=10)
        elif choice == '2':
            print("⚠️ 주의: 실제 로봇이 움직일 수 있습니다!")
            confirm = input("계속하시겠습니까? (y/N): ").strip().lower()
            if confirm == 'y':
                robot_system.run_full_ai_control()
            else:
                print("취소되었습니다.")
        elif choice == '3':
            robot_system.print_system_status()
        else:
            print("잘못된 선택입니다.")
        
    except KeyboardInterrupt:
        print("\n⚠️ 사용자가 중단했습니다")
    
    except Exception as e:
        print(f"❌ 예상치 못한 오류: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        robot_system.shutdown()
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)