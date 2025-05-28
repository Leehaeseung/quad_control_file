#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
데이터 결합기
모든 센서를 통합하고 Isaac Lab 텐서를 생성하는 메인 클래스
"""

import os
import time
import threading
import numpy as np
from datetime import datetime
from config import *
from imu_sensor import IMUSensor
from motor_sensor import MotorSensor  
from camera_sensor import CameraSensor
from isaac_converter import IsaacConverter


class DataCombiner:
    def __init__(self):
        """데이터 결합기 초기화"""
        print("🚀 데이터 결합기 초기화 중...")
        
        # 센서 모듈들
        self.imu = None
        self.motor = None
        self.camera = None
        
        # Isaac Lab 변환기
        self.converter = IsaacConverter()
        
        # 시스템 상태
        self.is_running = False
        self.data_ready = False
        
        # 최신 데이터 저장
        self.latest_isaac_data = None
        self.latest_tensor = None
        self.data_lock = threading.Lock()
        
        # 성능 모니터링
        self.update_count = 0
        self.last_update_time = None
        self.fps_counter = 0
        
        if DEBUG_MODE:
            print("✅ 데이터 결합기 초기화 완료")
    
    def initialize_sensors(self):
        """모든 센서 초기화"""
        print("🔧 센서 초기화 중...")
        
        sensor_success = {"imu": False, "motor": False, "camera": False}
        
        # IMU 초기화
        try:
            self.imu = IMUSensor()
            sensor_success["imu"] = True
            print("✅ IMU 센서 초기화 완료")
        except Exception as e:
            print(f"⚠️ IMU 센서 초기화 실패: {e}")
            self.imu = None
        
        # 모터 초기화
        try:
            self.motor = MotorSensor()
            if self.motor.connect():
                sensor_success["motor"] = True
                print("✅ 모터 센서 초기화 완료")
            else:
                print("⚠️ 모터 센서 연결 실패")
                self.motor = None
        except Exception as e:
            print(f"⚠️ 모터 센서 초기화 실패: {e}")
            self.motor = None
        
        # 카메라 초기화
        try:
            self.camera = CameraSensor()
            if self.camera.start():
                sensor_success["camera"] = True
                print("✅ 카메라 센서 초기화 완료")
            else:
                print("⚠️ 카메라 센서 시작 실패")
                self.camera = None
        except Exception as e:
            print(f"⚠️ 카메라 센서 초기화 실패: {e}")
            self.camera = None
        
        # 결과 요약
        success_count = sum(sensor_success.values())
        print(f"\n📊 센서 초기화 결과: {success_count}/3 성공")
        for sensor_name, success in sensor_success.items():
            status = "✅" if success else "❌"
            print(f"  {status} {sensor_name.upper()}")
        
        return success_count > 0
    
    def start_data_collection(self):
        """데이터 수집 시작"""
        if not self.initialize_sensors():
            print("❌ 센서 초기화 실패로 데이터 수집을 시작할 수 없습니다")
            return False
        
        self.is_running = True
        self.last_update_time = time.time()
        
        print(f"🔥 데이터 수집 시작 (주기: {1000/SYSTEM_FREQUENCY:.1f}ms)")
        return True
    
    def update_data(self):
        """센서 데이터 업데이트 및 Isaac Lab 텐서 생성"""
        if not self.is_running:
            return False
        
        try:
            # 각 센서에서 데이터 수집
            imu_data = self.imu.get_isaac_lab_data() if self.imu else None
            motor_data = self.motor.get_isaac_lab_data() if self.motor else None
            camera_data = self.camera.get_isaac_lab_data() if self.camera else None
            
            # Isaac Lab 형식으로 변환
            isaac_data = self.converter.sensors_to_isaac_format(imu_data, motor_data, camera_data)
            
            # 텐서 생성
            isaac_tensor = self.converter.create_isaac_tensor(isaac_data)
            
            # 데이터 저장 (스레드 안전)
            with self.data_lock:
                self.latest_isaac_data = isaac_data
                self.latest_tensor = isaac_tensor
                self.data_ready = True
            
            # 성능 카운터 업데이트
            self.update_count += 1
            current_time = time.time()
            
            if self.last_update_time and (current_time - self.last_update_time) >= 1.0:
                self.fps_counter = self.update_count / (current_time - self.last_update_time)
                self.update_count = 0
                self.last_update_time = current_time
            
            return True
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"데이터 업데이트 오류: {e}")
            return False
    
    def get_latest_isaac_tensor(self):
        """최신 Isaac Lab 텐서 반환"""
        with self.data_lock:
            if self.latest_tensor is not None:
                return self.latest_tensor.copy()
            else:
                return np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.float32)
    
    def get_latest_isaac_data(self):
        """최신 Isaac Lab 데이터 반환"""
        with self.data_lock:
            if self.latest_isaac_data is not None:
                return self.latest_isaac_data.copy()
            else:
                return None
    
    def get_isaac_values_string(self):
        """Isaac Lab 형식 문자열 반환"""
        isaac_data = self.get_latest_isaac_data()
        if isaac_data:
            return self.converter.format_isaac_values_string(isaac_data)
        else:
            return "데이터 없음"
    
    def save_combined_obs(self, filepath=COMBINED_OBS_PATH):
        """Isaac Lab 호환 형식으로 텐서 저장"""
        tensor = self.get_latest_isaac_tensor()
        
        if tensor is not None:
            # 저장 디렉토리 생성
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            # 텐서 저장
            np.save(filepath, tensor)
            
            if DEBUG_MODE:
                info = self.converter.get_tensor_info(tensor)
                print(f"✅ Isaac Lab 텐서 저장: {filepath}")
                print(f"   형태: {info['shape']}")
                print(f"   깊이 범위: {info['depth_range'][0]:.3f}~{info['depth_range'][1]:.3f}m")
                print(f"   베이스 범위: {info['base_range'][0]:.3f}~{info['base_range'][1]:.3f}")
                print(f"   조인트 범위: {info['joint_range'][0]:.3f}~{info['joint_range'][1]:.3f}")
            
            return True
        else:
            print("❌ 저장할 텐서 데이터 없음")
            return False
    
    def save_sensor_log(self, filepath=SENSOR_LOG_PATH):
        """센서 로그 저장"""
        isaac_data = self.get_latest_isaac_data()
        
        if isaac_data:
            try:
                os.makedirs(os.path.dirname(filepath), exist_ok=True)
                
                with open(filepath, 'a') as f:
                    timestamp = datetime.now().isoformat()
                    values_str = self.converter.format_isaac_values_string(isaac_data)
                    
                    f.write(f"timestamp: {timestamp}\n")
                    f.write(f"{values_str}\n")
                    f.write(f"fps: {self.fps_counter:.1f}\n")
                    f.write("-" * 80 + "\n")
                
                return True
            except Exception as e:
                print(f"❌ 센서 로그 저장 실패: {e}")
                return False
        else:
            return False
    
    def get_system_status(self):
        """시스템 상태 정보 반환"""
        return {
            'is_running': self.is_running,
            'data_ready': self.data_ready,
            'imu_connected': self.imu.is_connected() if self.imu else False,
            'motor_connected': self.motor.is_connected if self.motor else False,  # 수정됨
            'camera_connected': self.camera.is_connected() if self.camera else False,
            'fps': self.fps_counter,
            'update_count': self.update_count
        }
    
    def print_system_status(self):
        """시스템 상태 출력"""
        status = self.get_system_status()
        
        print("\n📊 시스템 상태:")
        print("-" * 50)
        print(f"실행 중: {'✅' if status['is_running'] else '❌'}")
        print(f"데이터 준비: {'✅' if status['data_ready'] else '❌'}")
        print(f"FPS: {status['fps']:.1f}")
        print()
        print("센서 연결 상태:")
        print(f"  IMU: {'✅' if status['imu_connected'] else '❌'}")
        print(f"  모터: {'✅' if status['motor_connected'] else '❌'}")  
        print(f"  카메라: {'✅' if status['camera_connected'] else '❌'}")
        
        # Isaac Lab 값 출력
        if self.data_ready:
            print(f"\nIsaac Lab 값:")
            print(f"  {self.get_isaac_values_string()}")
        
        print("-" * 50)
    
    def run_continuous_update(self, duration_seconds=None):
        """지속적인 데이터 업데이트 실행"""
        if not self.start_data_collection():
            return False
        
        print(f"🔄 지속적 업데이트 시작 ({SYSTEM_FREQUENCY}Hz)")
        if duration_seconds:
            print(f"⏱️ {duration_seconds}초간 실행")
        
        start_time = time.time()
        
        try:
            while self.is_running:
                self.update_data()
                
                # 지속 시간 체크
                if duration_seconds and (time.time() - start_time) > duration_seconds:
                    break
                
                # 주기 맞춤
                time.sleep(1.0 / SYSTEM_FREQUENCY)
                
        except KeyboardInterrupt:
            print("\n⚠️ 사용자가 중단했습니다")
        
        return True
    
    def close(self):
        """모든 센서 연결 해제"""
        print("🔌 데이터 결합기 종료 중...")
        
        self.is_running = False
        
        if self.imu:
            self.imu.close()
        
        if self.motor:
            self.motor.disconnect()
        
        if self.camera:
            self.camera.stop()
        
        print("🔌 데이터 결합기 종료 완료")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 데이터 결합기 테스트 시작...")
    
    combiner = DataCombiner()
    
    try:
        # 5초간 데이터 수집 테스트
        if combiner.run_continuous_update(duration_seconds=5):
            print("\n📊 최종 상태:")
            combiner.print_system_status()
            
            # 데이터 저장 테스트
            print("\n💾 데이터 저장 테스트...")
            combiner.save_combined_obs()
            combiner.save_sensor_log()
        else:
            print("❌ 데이터 수집 실패")
            
    except Exception as e:
        print(f"테스트 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        combiner.close()