#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 센서 모듈
선형속도, 각속도, 중력 투영 데이터를 제공
Isaac Lab 호환 단위로 출력
"""

import time
import threading
import numpy as np
import serial
from scipy.spatial.transform import Rotation
from config import *


class IMUSensor:
    def __init__(self, port=IMU_PORT, baudrate=IMU_BAUDRATE):
        """IMU 센서 초기화"""
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1
            )
            
            # Isaac Lab 호환 데이터 저장
            self.base_linear_velocity = [0.0, 0.0, 0.0]     # m/s
            self.base_angular_velocity = [0.0, 0.0, 0.0]    # rad/s
            self.projected_gravity = [0.0, 0.0, -1.0]       # 정규화된 단위벡터
            
            # 내부 계산용 변수들
            self.prev_velocity = np.array([0.0, 0.0, 0.0])
            self.prev_time = None
            self.velocity_filter = np.array([0.0, 0.0, 0.0])
            self.buffer = ""
            
            # 스레드 안전성
            self.lock = threading.Lock()
            self.running = True
            
            # 백그라운드 업데이트 스레드
            self.update_thread = threading.Thread(target=self._update_thread, daemon=True)
            self.update_thread.start()
            
            # IMU 연속 데이터 모드 시작
            self.ser.write(b"<scd>")
            
            if DEBUG_MODE:
                print(f"✅ IMU 센서 초기화 완료: {port}")
                
        except Exception as e:
            print(f"❌ IMU 센서 초기화 실패: {e}")
            raise
    
    def _update_thread(self):
        """백그라운드에서 IMU 데이터 지속 업데이트"""
        while self.running:
            self._read_and_parse_data()
            time.sleep(SENSOR_UPDATE_RATE)
    
    def _read_and_parse_data(self):
        """IMU 데이터 읽기 및 파싱"""
        if not self.ser.in_waiting:
            return
            
        try:
            data = self.ser.read(self.ser.in_waiting)
            text_data = data.decode('utf-8', errors='replace')
            self.buffer += text_data
            
            lines = self.buffer.split('\n')
            if len(lines) > 1:
                for line in lines[:-1]:
                    if line.strip().startswith('*'):
                        self._parse_imu_data(line.strip())
                self.buffer = lines[-1]
                        
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"IMU 데이터 읽기 오류: {e}")
    
    def _parse_imu_data(self, data_line):
        """IMU 데이터 파싱 및 Isaac Lab 형식으로 변환"""
        try:
            # 시간 계산
            current_time = time.time()
            if self.prev_time is None:
                self.prev_time = current_time
                dt = 1.0 / 30.0  # 기본값
            else:
                dt = current_time - self.prev_time
                self.prev_time = current_time
            
            # 데이터 파싱
            if not data_line.startswith('*'):
                return
            
            values = data_line[1:].split(',')
            if len(values) < 10:
                return
            
            # 원본 데이터 추출
            quaternion = [float(values[0]), float(values[1]), float(values[2]), float(values[3])]
            gyro = [float(values[4]), float(values[5]), float(values[6])]      # deg/s
            accel = [float(values[7]), float(values[8]), float(values[9])]     # g
            
            with self.lock:
                # 1. 각속도 변환: deg/s → rad/s
                self.base_angular_velocity = [g * DEG_TO_RAD for g in gyro]
                
                # 2. 중력 투영 계산 및 정규화
                quat = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]  # [x,y,z,w]
                rotation = Rotation.from_quat(quat)
                
                gravity_world = np.array([0, 0, 9.81])  # 월드 중력
                gravity_body = rotation.apply(gravity_world, inverse=True)
                
                # 중력 벡터 정규화 (Isaac Lab 요구사항)
                gravity_norm = np.linalg.norm(gravity_body)
                if gravity_norm > 0:
                    self.projected_gravity = (gravity_body / gravity_norm).tolist()
                else:
                    self.projected_gravity = [0.0, 0.0, -1.0]
                
                # 3. 선형속도 계산 (가속도 적분)
                accel_ms2 = np.array(accel) * 9.81  # g → m/s²
                accel_without_gravity = accel_ms2 - gravity_body
                
                # 노이즈 제거
                accel_filtered = np.where(
                    np.abs(accel_without_gravity) < IMU_ACCELERATION_THRESHOLD,
                    0, accel_without_gravity
                )
                
                # 월드 좌표계로 변환
                accel_world = rotation.apply(accel_filtered)
                
                # 속도 적분 및 감쇠
                velocity = self.prev_velocity + accel_world * dt
                velocity = velocity * IMU_VELOCITY_DECAY
                
                # 저역통과 필터 적용
                self.velocity_filter = (IMU_VELOCITY_ALPHA * velocity + 
                                      (1 - IMU_VELOCITY_ALPHA) * self.velocity_filter)
                
                self.base_linear_velocity = self.velocity_filter.tolist()
                self.prev_velocity = velocity
                
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"IMU 데이터 파싱 오류: {e}")
    
    def get_isaac_lab_data(self):
        """Isaac Lab 호환 형식으로 데이터 반환"""
        with self.lock:
            return {
                'base_linear_velocity': self.base_linear_velocity.copy(),    # m/s
                'base_angular_velocity': self.base_angular_velocity.copy(),  # rad/s  
                'projected_gravity': self.projected_gravity.copy()           # 정규화된 단위벡터
            }
    
    def get_base_linear_velocity(self):
        """선형속도 반환 (m/s)"""
        with self.lock:
            return self.base_linear_velocity.copy()
    
    def get_base_angular_velocity(self):
        """각속도 반환 (rad/s)"""
        with self.lock:
            return self.base_angular_velocity.copy()
    
    def get_projected_gravity(self):
        """정규화된 중력 투영 벡터 반환"""
        with self.lock:
            return self.projected_gravity.copy()
    
    def is_connected(self):
        """연결 상태 확인"""
        return self.ser and self.ser.is_open and self.running
    
    def close(self):
        """IMU 센서 연결 종료"""
        self.running = False
        
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"<stp>")  # 연속 데이터 중지
                time.sleep(0.1)
                self.ser.close()
            except:
                pass
                
        if DEBUG_MODE:
            print("🔌 IMU 센서 연결 해제")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 IMU 센서 테스트 시작...")
    
    try:
        imu = IMUSensor()
        
        for i in range(10):
            data = imu.get_isaac_lab_data()
            print(f"Frame {i+1}:")
            print(f"  선속도 (m/s): {data['base_linear_velocity']}")
            print(f"  각속도 (rad/s): {data['base_angular_velocity']}")
            print(f"  중력투영: {data['projected_gravity']}")
            print()
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("테스트 중단")
    except Exception as e:
        print(f"테스트 오류: {e}")
    finally:
        if 'imu' in locals():
            imu.close()